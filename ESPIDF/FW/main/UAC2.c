// UAC2 front end

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "nvs.h"
#include "sdkconfig.h"

#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "driver/i2s_tdm.h"
#include "esp_clk_tree.h"
#include "led_strip.h"
#include "usb_device_uac.h"
#include "tusb.h"
#include "UAC2.h"
#include "Display.h"
#include "PCM1865.h"

#if __has_include("uac_config.h")
#include "uac_config.h"
#define UAC_CFG_FROM_COMPONENT 1
#endif

#ifndef MIC_CHANNEL_NUM
#ifdef CONFIG_UAC_MIC_CHANNEL_NUM
#define MIC_CHANNEL_NUM CONFIG_UAC_MIC_CHANNEL_NUM
#else
#define MIC_CHANNEL_NUM 4
#endif
#endif

#ifndef SPEAK_CHANNEL_NUM
#ifdef CONFIG_UAC_SPEAKER_CHANNEL_NUM
#define SPEAK_CHANNEL_NUM CONFIG_UAC_SPEAKER_CHANNEL_NUM
#else
#define SPEAK_CHANNEL_NUM 2
#endif
#endif

#ifndef DEFAULT_SAMPLE_RATE
#ifdef CONFIG_UAC_SAMPLE_RATE
#define DEFAULT_SAMPLE_RATE CONFIG_UAC_SAMPLE_RATE
#else
#define DEFAULT_SAMPLE_RATE 96000
#endif
#endif

#define DAC_RATE 96000
#define ASRC_NOMINAL_RATIO ((float)DEFAULT_SAMPLE_RATE / (float)DAC_RATE)

#if MIC_CHANNEL_NUM > 4
#error "PCM1865 TDM capture is configured for at most 4 microphone channels"
#endif

#define ADC_RATE 48000
#define ADC_TDM_TOTAL_SLOTS 8
#define ADC_TDM_ACTIVE_MASK 0x0F
#define ADC_BCK_GPIO GPIO_NUM_13
#define ADC_WS_GPIO GPIO_NUM_14
#define ADC_DIN_GPIO GPIO_NUM_12
#define ADC_DMA_DESC_NUM 6
#define ADC_DMA_FRAME_NUM 128
#define ADC_RING_FRAMES 4096
#define ADC_TARGET_FRAMES 512
#define ADC_RATIO_NOMINAL ((float)ADC_RATE / (float)DEFAULT_SAMPLE_RATE)
#define ADC_RATIO_LIMIT 0.02f
#define ADC_SERVO_KP 0.05f
#define ADC_SERVO_KI 1.0e-4f
#define ADC_SERVO_ALPHA 0.03f
#define ADC_IDLE_RESET_US 100000

#define ADC_TDM_WS_POL 1
#define ADC_TDM_BIT_SHIFT 0

#ifndef CONFIG_UAC_BIT_RESOLUTION
#define CONFIG_UAC_BIT_RESOLUTION 16
#endif
#ifndef CONFIG_UAC_BYTES_PER_SAMPLE
#define CONFIG_UAC_BYTES_PER_SAMPLE 2
#endif

#define USB_SAMPLE_BYTES CONFIG_UAC_BYTES_PER_SAMPLE
#define USB_FRAME_BYTES (USB_SAMPLE_BYTES * 2)
#define PCM_FRAME_BYTES 8

#define ASRC_BLOCK_FRAMES 128
#define ASRC_TAPS 32
#define ASRC_PHASES 256
#define ASRC_CUTOFF 0.44f
#define ASRC_KAISER_BETA 7.0f
#define ASRC_KP 0.05f
#define ASRC_KI 4.0e-5f
#define ASRC_EF_ALPHA 0.03f
#define ASRC_INTEG_LIMIT 0.05f
#define ASRC_RATIO_LIMIT 0.05f

// USB runtime config (channel counts, sample rates, FIFO sizes and statistics)
typedef struct {
    bool (*pull)(void *ctx, int32_t *l, int32_t *r);
    size_t (*avail)(void *ctx);
    void *ctx;
    float tab[ASRC_PHASES + 1][ASRC_TAPS];
    float hist[2][2 * ASRC_TAPS];
    int wp;
    float frac;
    float efilt;
    float integ;
    float ratio;
    size_t target;
} asrc_t;

static int8_t s_channel_gain_db[6];

#define UAC_SEL_COUNT 7

// ASRC block (fractional-phase Bessel interpolator bridging USB packet jitter to the DAC I2S stream)
static uint8_t uac_sel_to_channel(uint8_t sel)
{
    return sel == 0 ? 4 : (uint8_t)(sel - 1);
}

static float asrc_bessel_i0(float x)
{
    float sum = 1.0f, term = 1.0f, hx = x * 0.5f;

    for (int k = 1; k < 40; k++) {
        term *= (hx / (float)k);
        sum += term * term;
        if (term * term < 1.0e-9f * sum) {
            break;
        }
    }
    return sum;
}

static void asrc_build_table(asrc_t *a)
{
    const float pi = 3.14159265359f;
    const int center = ASRC_TAPS / 2 - 1;
    const float half = (float)(ASRC_TAPS / 2);
    const float i0b = asrc_bessel_i0(ASRC_KAISER_BETA);

    for (int ph = 0; ph <= ASRC_PHASES; ph++) {
        float frac = (float)ph / (float)ASRC_PHASES;
        float sum = 0.0f;

        for (int k = 0; k < ASRC_TAPS; k++) {
            float d = (float)(k - center) - frac;
            float u = d / half;
            float w = 0.0f;

            if (u > -1.0f && u < 1.0f) {
                w = asrc_bessel_i0(ASRC_KAISER_BETA * sqrtf(1.0f - u * u)) / i0b;
            }

            float x = 2.0f * ASRC_CUTOFF * d;
            float s = (fabsf(x) < 1.0e-6f) ? 1.0f : sinf(pi * x) / (pi * x);

            a->tab[ph][k] = s * w;
            sum += s * w;
        }

        for (int k = 0; k < ASRC_TAPS; k++) {
            a->tab[ph][k] /= sum;
        }
    }
}

static void asrc_init(asrc_t *a,
                      bool (*pull)(void *, int32_t *, int32_t *),
                      size_t (*avail)(void *),
                      void *ctx,
                      size_t target)
{
    memset(a, 0, sizeof(*a));
    a->pull = pull;
    a->avail = avail;
    a->ctx = ctx;
    a->target = target;
    a->ratio = ASRC_NOMINAL_RATIO;
    asrc_build_table(a);
}

static void asrc_shift_in(asrc_t *a, int32_t l, int32_t r)
{
    a->hist[0][a->wp] = a->hist[0][a->wp + ASRC_TAPS] = (float)l;
    a->hist[1][a->wp] = a->hist[1][a->wp + ASRC_TAPS] = (float)r;
    a->wp = (a->wp + 1) % ASRC_TAPS;
}

static bool asrc_prime(asrc_t *a)
{
    for (int i = 0; i < ASRC_TAPS; i++) {
        int32_t l, r;
        if (!a->pull(a->ctx, &l, &r)) {
            return false;
        }
        asrc_shift_in(a, l, r);
    }
    a->frac = 0.0f;
    a->efilt = ((float)a->avail(a->ctx) - (float)a->target) / (float)a->target;
    return true;
}

static void asrc_update_ratio(asrc_t *a)
{
    float e = ((float)a->avail(a->ctx) - (float)a->target) / (float)a->target;

    if (e > 1.0f) e = 1.0f;
    if (e < -1.0f) e = -1.0f;

    a->efilt += ASRC_EF_ALPHA * (e - a->efilt);

    a->integ += ASRC_KI * a->efilt;
    if (a->integ > ASRC_INTEG_LIMIT) a->integ = ASRC_INTEG_LIMIT;
    if (a->integ < -ASRC_INTEG_LIMIT) a->integ = -ASRC_INTEG_LIMIT;

    a->ratio = ASRC_NOMINAL_RATIO *
               (1.0f + a->integ + ASRC_KP * a->efilt);
    if (a->ratio > ASRC_NOMINAL_RATIO * (1.0f + ASRC_RATIO_LIMIT))
        a->ratio = ASRC_NOMINAL_RATIO * (1.0f + ASRC_RATIO_LIMIT);
    if (a->ratio < ASRC_NOMINAL_RATIO * (1.0f - ASRC_RATIO_LIMIT))
        a->ratio = ASRC_NOMINAL_RATIO * (1.0f - ASRC_RATIO_LIMIT);
}

static size_t asrc_run(asrc_t *a, int32_t *out, size_t n, float gain)
{
    float c[ASRC_TAPS];
    float channel_gain[2]={gain,gain};
    for(int ch=0;ch<2;ch++) channel_gain[ch]*=powf(10.0f,(float)s_channel_gain_db[4+ch]*0.05f);

    for (size_t i = 0; i < n; i++) {
        float pos = a->frac * (float)ASRC_PHASES;
        int ph = (int)pos;
        float pf = pos - (float)ph;

        if (ph > ASRC_PHASES - 1) {
            ph = ASRC_PHASES - 1;
            pf = 1.0f;
        }

        const float *r0 = a->tab[ph];
        const float *r1 = a->tab[ph + 1];

        for (int k = 0; k < ASRC_TAPS; k++) {
            c[k] = r0[k] + (r1[k] - r0[k]) * pf;
        }

        for (int ch = 0; ch < 2; ch++) {
            const float *hh = &a->hist[ch][a->wp];
            float acc = 0.0f;

            for (int k = 0; k < ASRC_TAPS; k++) {
                acc += c[k] * hh[k];
            }

            float v = acc * channel_gain[ch];
            if (v > 8388607.0f) v = 8388607.0f;
            if (v < -8388608.0f) v = -8388608.0f;
            out[2 * i + ch] = (int32_t)(v >= 0.0f ? v + 0.5f : v - 0.5f);
        }

        a->frac += a->ratio;
        while (a->frac >= 1.0f) {
            int32_t l, r;
            if (!a->pull(a->ctx, &l, &r)) {
                return i + 1;
            }
            asrc_shift_in(a, l, r);
            a->frac -= 1.0f;
        }
    }
    return n;
}

static const char *TAG = "p4_uac";

#define LED_GPIO 51
#define DAC_BCK_GPIO GPIO_NUM_4
#define DAC_WS_GPIO GPIO_NUM_11
#define DAC_DIN_GPIO GPIO_NUM_5

#define DAC_FRAME_BYTES 8

#define RING_BYTES (64 * 1024)
#define TARGET_FILL_FRAMES 4096
#define CHUNK_FRAMES 256
#define MAX_USB_PACKET_FRAMES 128
#define I2S_WRITE_TIMEOUT_MS 100
#define UAC_OUTPUT_GAIN_CAP 0.50f
#define SPECTRUM_N 4096
#define SPECTRUM_BANDS 16

static led_strip_handle_t s_led;
static volatile bool s_uac_ready;
static StreamBufferHandle_t s_audio_stream;
static i2s_chan_handle_t s_dac_tx;
static i2s_chan_handle_t s_adc_rx;

static int32_t s_adc_ring[ADC_RING_FRAMES][MIC_CHANNEL_NUM];
static uint32_t s_adc_wr;
static uint32_t s_adc_rd;
static int32_t s_adc_dma[ADC_DMA_DESC_NUM * ADC_DMA_FRAME_NUM * MIC_CHANNEL_NUM];
static float s_adc_frac;
static float s_adc_efilt;
static float s_adc_integ;
static float s_adc_ratio = ADC_RATIO_NOMINAL;
static bool s_adc_started;
static volatile int64_t s_adc_last_cb_us;
static int64_t s_input_decay_us;

static volatile uint32_t s_adc_ring_overflows;
static volatile uint32_t s_adc_starve_frames;

static volatile uint32_t s_adc_drained_frames;
static uint32_t s_adc_prev_drained;
static uint32_t s_adc_log_dt_ms;

static volatile uint32_t s_adc_drain_calls;
static volatile uint32_t s_adc_drain_zero;
static volatile uint32_t s_adc_drain_got_bytes;
static volatile uint32_t s_adc_consumed_frames;
static volatile uint32_t s_adc_occ_sum;
static volatile uint32_t s_adc_occ_n;
static volatile uint32_t s_adc_occ_min;
static volatile uint32_t s_adc_occ_max;
static volatile uint32_t s_adc_cb_calls;
static volatile uint32_t s_adc_cb_bytes;
static uint32_t s_adc_prev_drain_calls, s_adc_prev_drain_zero, s_adc_prev_got;
static uint32_t s_adc_prev_consumed, s_adc_prev_occ_sum, s_adc_prev_occ_n;
static uint32_t s_adc_prev_occ_min = UINT32_MAX, s_adc_prev_occ_max;
static uint32_t s_adc_prev_cb_calls, s_adc_prev_cb_bytes;

static float s_in_rms_sq[MIC_CHANNEL_NUM];
static volatile uint8_t s_input_levels[MIC_CHANNEL_NUM];

static volatile int64_t s_last_audio_us;
static bool s_playback_reset_requested;
static volatile bool s_ctrl_flash;
static volatile uint32_t s_volume = 100;
static volatile bool s_encoder_device_gain;
static volatile uint8_t s_encoder_channel;
static volatile bool s_mute;

static volatile uint32_t s_drop_count;
static volatile uint32_t s_underrun_count;
static volatile uint32_t s_dbg_fill;
static volatile int32_t s_dbg_ppm;

static volatile uint32_t s_rx_calls;
static volatile uint32_t s_rx_frames;
static volatile uint32_t s_rx_bytes;
static volatile uint32_t s_rx_len_min;
static volatile uint32_t s_rx_len_max;
static volatile uint32_t s_rx_gap_max_us;
static volatile int64_t s_rx_prev_us;
static volatile uint32_t s_playback_starves;
static volatile uint32_t s_i2s_write_max_us;
static volatile uint32_t s_i2s_write_errors;
static uint32_t s_last_reported_drops;
static uint32_t s_last_reported_underruns;

static int32_t s_chunk[CHUNK_FRAMES * 2];
static size_t s_chunk_len;
static size_t s_chunk_pos;

static int32_t s_spectrum_samples[SPECTRUM_N];
static float s_spectrum_window[SPECTRUM_N];
static uint32_t s_spectrum_write;
static uint32_t s_spectrum_count;
static uint32_t s_spectrum_samples_since_update;
static float s_spectrum_window_sum;
static float s_spectrum_dbfs[SPECTRUM_BANDS];
static uint8_t s_spectrum_buffer_source=0xff;

// path helpers like drift stats and spectrum feed, ect.
static void spectrum_add_sample(uint8_t source,int32_t sample)
{
    if(!display_spectrum_active() || source!=display_spectrum_source())return;
    if(source!=s_spectrum_buffer_source){
        s_spectrum_buffer_source=source;
        s_spectrum_write=0;
        s_spectrum_count=0;
        s_spectrum_samples_since_update=0;
    }
    s_spectrum_samples[s_spectrum_write] = sample;
    s_spectrum_write = (s_spectrum_write + 1) % SPECTRUM_N;
    if (s_spectrum_count < SPECTRUM_N) s_spectrum_count++;
    if (++s_spectrum_samples_since_update < DEFAULT_SAMPLE_RATE / 2) return;
    s_spectrum_samples_since_update = 0;
    if (s_spectrum_count < SPECTRUM_N) return;
    static bool window_ready;
    if (!window_ready) {
        for (int i=0;i<SPECTRUM_N;i++) {
            s_spectrum_window[i]=0.5f-0.5f*cosf(6.28318530718f*(float)i/(float)(SPECTRUM_N-1));
            s_spectrum_window_sum+=s_spectrum_window[i];
        }
        window_ready=true;
    }

    for (int band=0;band<SPECTRUM_BANDS;band++) {
        float frequency=20.0f*powf(1000.0f,(float)band/(float)(SPECTRUM_BANDS-1));
        float omega=6.28318530718f*frequency/(float)DEFAULT_SAMPLE_RATE;
        float coeff=2.0f*cosf(omega), q1=0.0f, q2=0.0f;
        for (int i=0;i<SPECTRUM_N;i++) {
            uint32_t index=(s_spectrum_write+(uint32_t)i)%SPECTRUM_N;
            float sample=(float)s_spectrum_samples[index]/8388607.0f;
            float q0=sample*s_spectrum_window[i]+coeff*q1-q2;
            q2=q1; q1=q0;
        }
        float power=q1*q1+q2*q2-coeff*q1*q2;
        if(power<0.0f)power=0.0f;
        float amplitude=s_spectrum_window_sum>0.0f?2.0f*sqrtf(power)/s_spectrum_window_sum:0.0f;
        s_spectrum_dbfs[band]=amplitude>1.0e-6f?20.0f*log10f(amplitude):-120.0f;
    }
    display_set_spectrum(s_spectrum_dbfs);
}

static int32_t decode_usb_sample(const uint8_t *p)
{
#if CONFIG_UAC_BYTES_PER_SAMPLE == 2
    int16_t sample = (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
    return (int32_t)sample * 256;
#elif CONFIG_UAC_BYTES_PER_SAMPLE == 3
    uint32_t raw = (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                   ((uint32_t)p[2] << 16);
    if (raw & 0x00800000U) raw |= 0xFF000000U;
    return (int32_t)raw;
#elif CONFIG_UAC_BYTES_PER_SAMPLE == 4
    uint32_t raw = (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                   ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
#if CONFIG_UAC_BIT_RESOLUTION == 24
    return (int32_t)raw / 256;
#else
    return (int32_t)raw / 256;
#endif
#else
#error "Unsupported CONFIG_UAC_BYTES_PER_SAMPLE"
#endif
}

// speaker endpoint callback to FIFO
static esp_err_t uac_output_cb(uint8_t *buf, size_t len, void *ctx)
{
    (void)ctx;

    if (len == 0) {
        __atomic_store_n(&s_playback_reset_requested, true, __ATOMIC_RELEASE);
        return ESP_OK;
    }

    int64_t now = esp_timer_get_time();

    if (s_rx_prev_us != 0) {
        uint32_t gap = (uint32_t)(now - s_rx_prev_us);
        if (gap > s_rx_gap_max_us) {
            s_rx_gap_max_us = gap;
        }
    }
    s_rx_prev_us = now;
    s_rx_calls++;
    s_rx_frames += (uint32_t)(len / USB_FRAME_BYTES);
    s_rx_bytes += (uint32_t)len;
    if (len != 0) {
        uint32_t old_min = s_rx_len_min;
        if (old_min == 0 || len < old_min) s_rx_len_min = (uint32_t)len;
        if (len > s_rx_len_max) s_rx_len_max = (uint32_t)len;
    }

    s_last_audio_us = now;

    if (!buf || !s_audio_stream) {
        return ESP_OK;
    }

    if ((len % USB_FRAME_BYTES) != 0) {
        s_drop_count++;
        return ESP_OK;
    }

    size_t frames = len / USB_FRAME_BYTES;
    if (frames > MAX_USB_PACKET_FRAMES) {
        s_drop_count++;
        return ESP_OK;
    }

    int32_t converted[MAX_USB_PACKET_FRAMES * 2];
    for (size_t frame = 0; frame < frames; frame++) {
        const uint8_t *src = &buf[frame * USB_FRAME_BYTES];
        converted[2 * frame] = decode_usb_sample(src);
        converted[2 * frame + 1] = decode_usb_sample(src + USB_SAMPLE_BYTES);
    }

    size_t space = xStreamBufferSpacesAvailable(s_audio_stream);
    if (space < PCM_FRAME_BYTES) {
        s_drop_count += (uint32_t)frames;
        return ESP_OK;
    }

    size_t fit_frames = frames;
    if (space / PCM_FRAME_BYTES < frames) {
        fit_frames = space / PCM_FRAME_BYTES;
        s_drop_count += (uint32_t)(frames - fit_frames);
    }

    size_t send_len = fit_frames * PCM_FRAME_BYTES;
    if (xStreamBufferSend(s_audio_stream, converted, send_len, 0) != send_len) {
        s_drop_count++;
    }

    return ESP_OK;
}

// DAC source hooks for the ASRC
static bool in_pull(void *ctx, int32_t *l, int32_t *r)
{
    (void)ctx;

    if (s_chunk_pos >= s_chunk_len) {
        size_t n = xStreamBufferReceive(
            s_audio_stream, s_chunk, sizeof(s_chunk), 0);

        s_chunk_len = n / PCM_FRAME_BYTES;
        s_chunk_pos = 0;

        if (s_chunk_len == 0) {
            return false;
        }
    }

    *l = s_chunk[2 * s_chunk_pos];
    *r = s_chunk[2 * s_chunk_pos + 1];
    s_chunk_pos++;
    return true;
}

static size_t in_avail(void *ctx)
{
    (void)ctx;
    return xStreamBufferBytesAvailable(s_audio_stream) / PCM_FRAME_BYTES +
           (s_chunk_len - s_chunk_pos);
}

static void encode_usb_sample24(uint8_t *p, int32_t sample)
{
#if CONFIG_UAC_BYTES_PER_SAMPLE == 2
    uint16_t value = (uint16_t)(sample >> 8);
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
#elif CONFIG_UAC_BYTES_PER_SAMPLE == 3
    uint32_t value = (uint32_t)sample & 0x00FFFFFFU;
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
    p[2] = (uint8_t)(value >> 16);
#elif CONFIG_UAC_BYTES_PER_SAMPLE == 4

    uint32_t value = (uint32_t)sample << 8;
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
    p[2] = (uint8_t)(value >> 16);
    p[3] = (uint8_t)(value >> 24);
#else
#error "Unsupported CONFIG_UAC_BYTES_PER_SAMPLE"
#endif
}

// Mic path plumbing, per-frame buffers, DMA drain ect.
static void adc_input_reset(void)
{
    s_adc_wr = 0;
    s_adc_rd = 0;
    s_adc_frac = 0.0f;
    s_adc_efilt = 0.0f;
    s_adc_integ = 0.0f;
    s_adc_ratio = ADC_RATIO_NOMINAL;
    s_adc_started = false;
    s_adc_ring_overflows = 0;
    s_adc_starve_frames = 0;
    s_adc_drained_frames = 0;
    s_adc_drain_calls = 0;
    s_adc_drain_zero = 0;
    s_adc_drain_got_bytes = 0;
    s_adc_consumed_frames = 0;
    s_adc_occ_sum = 0;
    s_adc_occ_n = 0;
    s_adc_occ_min = UINT32_MAX;
    s_adc_occ_max = 0;
    s_adc_cb_calls = 0;
    s_adc_cb_bytes = 0;
    memset(s_in_rms_sq, 0, sizeof(s_in_rms_sq));
    for (int ch = 0; ch < MIC_CHANNEL_NUM; ch++) {
        s_input_levels[ch] = 0;
    }
}

static size_t adc_drain_dma(void)
{
    size_t got = 0;
    s_adc_drain_calls++;

    (void)i2s_channel_read(s_adc_rx, s_adc_dma, sizeof(s_adc_dma), &got, 0);

    if (!got) {
        s_adc_drain_zero++;
    }
    s_adc_drain_got_bytes += (uint32_t)got;

    size_t frames = got / (MIC_CHANNEL_NUM * sizeof(int32_t));
    if (frames) {
        s_adc_drained_frames += (uint32_t)frames;
    }
    for (size_t i = 0; i < frames; i++) {
        const int32_t *src = &s_adc_dma[i * MIC_CHANNEL_NUM];
        int32_t *dst = s_adc_ring[s_adc_wr % ADC_RING_FRAMES];

        for (int ch = 0; ch < MIC_CHANNEL_NUM; ch++) {

            dst[ch] = src[ch] >> 8;
        }

        s_adc_wr++;
        if (s_adc_wr - s_adc_rd > ADC_RING_FRAMES) {
            s_adc_rd++;
            s_adc_ring_overflows++;
        }
    }

    return frames;
}

static void adc_flush_dma(void)
{
    for (int i = 0; i < 2 * ADC_DMA_DESC_NUM; i++) {
        size_t got = 0;
        (void)i2s_channel_read(s_adc_rx, s_adc_dma, sizeof(s_adc_dma), &got, 0);
        if (!got) {
            break;
        }
    }
}

static void adc_update_ratio(void)
{
    if (!s_adc_started) {
        return;
    }

    float occupancy = (float)(s_adc_wr - s_adc_rd);
    float error = (occupancy - (float)ADC_TARGET_FRAMES) / (float)ADC_TARGET_FRAMES;

    if (error > 1.0f) error = 1.0f;
    if (error < -1.0f) error = -1.0f;

    s_adc_efilt += ADC_SERVO_ALPHA * (error - s_adc_efilt);

    s_adc_integ += ADC_SERVO_KI * s_adc_efilt;
    if (s_adc_integ > ADC_RATIO_LIMIT) s_adc_integ = ADC_RATIO_LIMIT;
    if (s_adc_integ < -ADC_RATIO_LIMIT) s_adc_integ = -ADC_RATIO_LIMIT;

    float ratio = ADC_RATIO_NOMINAL *
                  (1.0f + s_adc_integ + ADC_SERVO_KP * s_adc_efilt);

    if (ratio > ADC_RATIO_NOMINAL * (1.0f + ADC_RATIO_LIMIT))
        ratio = ADC_RATIO_NOMINAL * (1.0f + ADC_RATIO_LIMIT);
    if (ratio < ADC_RATIO_NOMINAL * (1.0f - ADC_RATIO_LIMIT))
        ratio = ADC_RATIO_NOMINAL * (1.0f - ADC_RATIO_LIMIT);

    s_adc_ratio = ratio;
}

// packs frames into USB 24-bit packets
static esp_err_t uac_input_cb(
    uint8_t *buf,
    size_t len,
    size_t *bytes_read,
    void *ctx)
{
    (void)ctx;

    if (!buf || !bytes_read)
    {
        return ESP_OK;
    }

    *bytes_read = 0;

    const size_t frame_bytes = USB_SAMPLE_BYTES * MIC_CHANNEL_NUM;
    if (!frame_bytes)
    {
        return ESP_OK;
    }

    const size_t frames = len / frame_bytes;
    if (!frames)
    {
        return ESP_OK;
    }

    int64_t now = esp_timer_get_time();

    s_adc_cb_calls++;
    s_adc_cb_bytes += (uint32_t)len;

    if (s_adc_last_cb_us == 0 || now - s_adc_last_cb_us > ADC_IDLE_RESET_US) {
        adc_input_reset();
        if (s_adc_rx) {
            adc_flush_dma();
        }
    }
    s_adc_last_cb_us = now;

    if (s_adc_rx) {
        if (adc_drain_dma() > 0) {
            s_adc_started = true;
        }
        adc_update_ratio();
    }

    float gains[MIC_CHANNEL_NUM];
    for (int ch = 0; ch < MIC_CHANNEL_NUM; ch++) {
        gains[ch] = powf(10.0f, (float)s_channel_gain_db[ch] * 0.05f);
    }

    const float norm_scale = 1.0f / 8388608.0f;
    float sum_sq[MIC_CHANNEL_NUM] = {0};
    int32_t held[MIC_CHANNEL_NUM] = {0};
    uint8_t *out = buf;

    for (size_t f = 0; f < frames; f++) {
        uint32_t occupancy = s_adc_wr - s_adc_rd;
        int32_t sample[MIC_CHANNEL_NUM];

        s_adc_occ_sum += occupancy;
        s_adc_occ_n++;
        if (occupancy < s_adc_occ_min) s_adc_occ_min = occupancy;
        if (occupancy > s_adc_occ_max) s_adc_occ_max = occupancy;

        if (occupancy >= 2) {
            const int32_t *a = s_adc_ring[s_adc_rd % ADC_RING_FRAMES];
            const int32_t *b = s_adc_ring[(s_adc_rd + 1) % ADC_RING_FRAMES];

            for (int ch = 0; ch < MIC_CHANNEL_NUM; ch++) {
                float v = (float)a[ch] +
                          ((float)b[ch] - (float)a[ch]) * s_adc_frac;
                sample[ch] = (int32_t)(v >= 0.0f ? v + 0.5f : v - 0.5f);
            }
        } else if (occupancy == 1) {
            const int32_t *a = s_adc_ring[s_adc_rd % ADC_RING_FRAMES];

            for (int ch = 0; ch < MIC_CHANNEL_NUM; ch++) {
                sample[ch] = a[ch];
            }
        } else {
            for (int ch = 0; ch < MIC_CHANNEL_NUM; ch++) {
                sample[ch] = held[ch];
            }
            s_adc_starve_frames += 1;
        }

        for (int ch = 0; ch < MIC_CHANNEL_NUM; ch++) {
            float v = (float)sample[ch] * gains[ch];

            if (v > 8388607.0f) v = 8388607.0f;
            if (v < -8388608.0f) v = -8388608.0f;

            int32_t s24 = (int32_t)(v >= 0.0f ? v + 0.5f : v - 0.5f);
            held[ch] = s24;

            encode_usb_sample24(out, s24);
            spectrum_add_sample((uint8_t)(2 + ch), s24);
            out += USB_SAMPLE_BYTES;

            float n = (float)s24 * norm_scale;
            sum_sq[ch] += n * n;
        }

        s_adc_frac += s_adc_ratio;
        while (s_adc_frac >= 1.0f) {
            if (s_adc_wr - s_adc_rd > 1) {
                s_adc_rd++;
                s_adc_consumed_frames++;
                s_adc_frac -= 1.0f;
            } else {

                s_adc_frac = 0.0f;
                break;
            }
        }
    }

    float block_alpha = 1.0f - expf(-((float)frames /
                                     (float)DEFAULT_SAMPLE_RATE) / 0.30f);

    for (int ch = 0; ch < MIC_CHANNEL_NUM; ch++) {
        float block_rms_sq = sum_sq[ch] / (float)frames;
        s_in_rms_sq[ch] += (block_rms_sq - s_in_rms_sq[ch]) * block_alpha;

        float db = s_in_rms_sq[ch] > 1.0e-12f
                       ? 10.0f * log10f(s_in_rms_sq[ch])
                       : -120.0f;
        if (db < -60.0f) db = -60.0f;
        if (db > 0.0f) db = 0.0f;
        s_input_levels[ch] = (uint8_t)((db + 60.0f) * 100.0f / 60.0f);
    }

    *bytes_read = frames * frame_bytes;
    return ESP_OK;
}

// Host-side mute/volume callbacks
static void uac_mute_cb(uint32_t mute, void *ctx)
{
    (void)ctx;
    s_mute = (mute != 0);
    s_ctrl_flash = true;
}

static void uac_volume_cb(uint32_t volume, void *ctx)
{
    (void)ctx;

    s_volume = volume > 100 ? 100 : volume;
    s_ctrl_flash = true;
    display_set_main_volume((uint8_t)s_volume);
}

// Encoder tingting
void uac2_encoder_step(int direction)
{
    if(s_encoder_device_gain) {
        uint8_t sel=s_encoder_channel%UAC_SEL_COUNT;
        int first,last;
        if(sel==0){ first=4; last=5; }
        else { first=uac_sel_to_channel(sel); last=first; }
        int next=(int)s_channel_gain_db[first]+direction;
        if(next < -12)next=-12;
        if(next > 12)next=12;
        for(uint8_t ch=(uint8_t)first; ch<=(uint8_t)last; ch++) s_channel_gain_db[ch]=(int8_t)next;
        nvs_handle_t nvs;
        if(nvs_open("audio",NVS_READWRITE,&nvs)==ESP_OK) {
            for(uint8_t ch=(uint8_t)first; ch<=(uint8_t)last; ch++) {
                char key[8]; snprintf(key,sizeof(key),"ch%u",(unsigned)ch);
                if(nvs_set_i8(nvs,key,s_channel_gain_db[ch])==ESP_OK)(void)nvs_commit(nvs);
            }
            nvs_close(nvs);
        }
        display_set_channel_gain(sel,s_channel_gain_db[first]);
    } else {
        int next=(int)s_volume+(direction*2);
        if(next<0)next=0;
        if(next>100)next=100;
        s_volume=(uint32_t)next;
        display_set_main_volume((uint8_t)s_volume);
    }
}

void uac2_encoder_click(void)
{
    if(s_encoder_device_gain)s_encoder_channel=(uint8_t)((s_encoder_channel+1)%UAC_SEL_COUNT);
    else s_encoder_channel=0;
    s_encoder_device_gain=true;
    display_set_channel_gain(s_encoder_channel,
                             s_channel_gain_db[uac_sel_to_channel(s_encoder_channel)]);
}

#define UAC2_MEDIA_PLAY_PAUSE 0x00CDu
#define UAC2_MEDIA_NEXT_TRACK 0x00B5u
#define UAC2_MEDIA_PREV_TRACK 0x00B6u

extern bool usb_desc_hid_send_usage(uint16_t usage);

static esp_timer_handle_t s_media_release_timer;
static volatile bool s_media_press_pending;

static void uac2_media_release_cb(void *arg)
{
    (void)arg;

    usb_desc_hid_send_usage(0);
    s_media_press_pending = false;
}

void uac2_host_media(uint16_t usage)
{
    const esp_timer_create_args_t timer_args = {
        .callback = uac2_media_release_cb,
        .arg = NULL,
        .name = "media_rel",
    };

    if (s_media_release_timer == NULL) {
        if (esp_timer_create(&timer_args, &s_media_release_timer) != ESP_OK) return;
    }

    if (s_media_press_pending) {
        esp_timer_stop(s_media_release_timer);
        usb_desc_hid_send_usage(0);
        s_media_press_pending = false;
    }

    if (!usb_desc_hid_send_usage(usage)) return;
    s_media_press_pending = true;
    (void)esp_timer_start_once(s_media_release_timer, 30000);
}

void uac2_toggle_mute(void)
{
    s_mute=!s_mute;
    s_ctrl_flash=true;
    ESP_LOGI(TAG,"Host audio %s",s_mute?"muted":"unmuted");
}

void uac2_encoder_hold(void)
{
    uint8_t sel=s_encoder_channel%UAC_SEL_COUNT;
    int first,last;
    if(sel==0){ first=4; last=5; }
    else { first=uac_sel_to_channel(sel); last=first; }
    for(uint8_t ch=(uint8_t)first; ch<=(uint8_t)last; ch++) s_channel_gain_db[ch]=0;
    nvs_handle_t nvs;
    if(nvs_open("audio",NVS_READWRITE,&nvs)==ESP_OK) {
        for(uint8_t ch=(uint8_t)first; ch<=(uint8_t)last; ch++) {
            char key[8]; snprintf(key,sizeof(key),"ch%u",(unsigned)ch);
            if(nvs_set_i8(nvs,key,0)==ESP_OK)(void)nvs_commit(nvs);
        }
        nvs_close(nvs);
    }
    display_set_channel_gain(sel,0);
}

void uac2_toggle_encoder_mode(void)
{
    s_encoder_device_gain=!s_encoder_device_gain;
    ESP_LOGI(TAG,"Encoder controls %s gain",s_encoder_device_gain?"selected channel":"master");
}

// LED
static void led_set(uint8_t r, uint8_t g, uint8_t b)
{
    led_strip_set_pixel(s_led, 0, r, g, b);
    led_strip_refresh(s_led);
}

static void led_init(void)
{
    led_strip_config_t sc = {
        .strip_gpio_num = LED_GPIO,
        .max_leds = 1,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags.invert_out = false,
    };

    led_strip_rmt_config_t rc = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
        .flags.with_dma = false,
    };

    ESP_ERROR_CHECK(
        led_strip_new_rmt_device(&sc, &rc, &s_led));

    led_strip_clear(s_led);
}

static void led_task(void *arg)
{
    (void)arg;

    bool on = false;
    uint32_t tick = 0;
    bool have_prev = false;
    uint32_t prev_calls = 0;
    uint32_t prev_frames = 0;
    uint32_t prev_bytes = 0;
    int64_t prev_t = 0;

    led_set(20, 20, 20);
    vTaskDelay(pdMS_TO_TICKS(300));
    led_set(0, 0, 0);

    while (1)
    {
        int64_t last = s_last_audio_us;

        bool streaming =
            last != 0 &&
            (esp_timer_get_time() - last) < 300000;

        if (s_ctrl_flash)
        {
            s_ctrl_flash = false;
            led_set(0, 40, 0);
            vTaskDelay(pdMS_TO_TICKS(100));
            on = false;
        }

        if (streaming)
        {
            on = !on;
            led_set(0, 0, on ? 40 : 0);

            if ((++tick % 13) == 0)
            {
                int64_t tnow = esp_timer_get_time();
                uint32_t calls = s_rx_calls;
                uint32_t frames = s_rx_frames;
                uint32_t bytes = s_rx_bytes;

                if (have_prev)
                {
                    uint32_t dt_ms = (uint32_t)((tnow - prev_t) / 1000);
                    if (dt_ms == 0) dt_ms = 1;
                    s_adc_log_dt_ms = dt_ms;
                    uac_speaker_stats_t usb = {0};
                    if (uac_device_get_speaker_stats(&usb)) {
                        display_usb_stats_t us = {
                            .usb_packets_per_sec=usb.usb_packets_per_sec,
                            .usb_bytes_per_sec=usb.usb_bytes_per_sec,
                            .fifo_bytes_per_sec=usb.fifo_bytes_per_sec,
                            .task_packets_per_sec=usb.task_packets_per_sec,
                            .queue_high_water=usb.queue_high_water,
                            .queue_depth=usb.queue_depth,
                            .drops=usb.drops,
                        };
                        display_set_usb_stats(&us);
                    }
                    display_audio_stats_t ds = {
                        .bytes_per_sec = (uint32_t)((uint64_t)(bytes-prev_bytes)*1000ULL/dt_ms),
                        .fifo_bytes_per_sec = (uint32_t)((uint64_t)(bytes-prev_bytes)*1000ULL/dt_ms),
                        .packets_per_sec = (uint32_t)((uint64_t)(calls-prev_calls)*1000ULL/dt_ms),
                        .queue_high_water = usb.queue_high_water,
                        .drops = s_drop_count, .queue_frames = s_dbg_fill,
                        .asrc_ppm = s_dbg_ppm, .underruns = s_underrun_count,
                        .rx_calls_per_sec = (uint32_t)((uint64_t)(calls-prev_calls)*1000ULL/dt_ms),
                        .rx_frames_per_sec = (uint32_t)((uint64_t)(frames-prev_frames)*1000ULL/dt_ms),
                        .rx_len_min = s_rx_len_min, .rx_len_max = s_rx_len_max,
                        .rx_gap_max_us = s_rx_gap_max_us,
                        .i2s_write_max_us = s_i2s_write_max_us,
                        .i2s_write_errors = s_i2s_write_errors,
                    };
                    display_set_audio_stats(&ds);

                    ESP_LOGI(
                        TAG,
                        "rx: %lu calls/s, %lu B/s, %lu stereo-frames/s, "
                        "len=%lu..%lu B, max gap=%lu us | queue=%lu frames, "
                        "ASRC=%+ld ppm, underruns=%lu (starves=%lu), drops=%lu, "
                        "I2S write max=%lu us errors=%lu",
                        (unsigned long)((uint64_t)(calls - prev_calls) * 1000ULL / dt_ms),
                        (unsigned long)((uint64_t)(bytes - prev_bytes) * 1000ULL / dt_ms),
                        (unsigned long)((uint64_t)(frames - prev_frames) * 1000ULL / dt_ms),
                        (unsigned long)s_rx_len_min,
                        (unsigned long)s_rx_len_max,
                        (unsigned long)s_rx_gap_max_us,
                        (unsigned long)s_dbg_fill,
                        (long)s_dbg_ppm,
                        (unsigned long)s_underrun_count,
                        (unsigned long)s_playback_starves,
                        (unsigned long)s_drop_count,
                        (unsigned long)s_i2s_write_max_us,
                        (unsigned long)s_i2s_write_errors);

                    ESP_LOGI(
                        TAG,
                        "adc: in=%lu f/s (%lu kB/s DMA, %lu drains/s, %lu empty), "
                        "wr=%lu/s rd=%lu/s occ=%lu [%lu..%lu], cb=%lu/s x %lu B, "
                        "overflows=%lu, starves=%lu, upsampler=%+ld ppm (servo ±%ld ppm)",
                        (unsigned long)(s_adc_log_dt_ms
                            ? (uint32_t)((uint64_t)(s_adc_drained_frames - s_adc_prev_drained) * 1000u / s_adc_log_dt_ms)
                            : 0),
                        (unsigned long)(s_adc_log_dt_ms
                            ? (uint32_t)((uint64_t)(s_adc_drain_got_bytes - s_adc_prev_got) * 1000u / s_adc_log_dt_ms / 1024u)
                            : 0),
                        (unsigned long)(s_adc_log_dt_ms
                            ? (uint32_t)((uint64_t)(s_adc_drain_calls - s_adc_prev_drain_calls) * 1000u / s_adc_log_dt_ms)
                            : 0),
                        (unsigned long)(s_adc_log_dt_ms
                            ? (uint32_t)((uint64_t)(s_adc_drain_zero - s_adc_prev_drain_zero) * 1000u / s_adc_log_dt_ms)
                            : 0),
                        (unsigned long)(s_adc_log_dt_ms
                            ? (uint32_t)((uint64_t)(s_adc_drained_frames - s_adc_prev_drained) * 1000u / s_adc_log_dt_ms)
                            : 0),
                        (unsigned long)(s_adc_log_dt_ms
                            ? (uint32_t)((uint64_t)(s_adc_consumed_frames - s_adc_prev_consumed) * 1000u / s_adc_log_dt_ms)
                            : 0),
                        (unsigned long)(s_adc_occ_n ? (s_adc_occ_sum - s_adc_prev_occ_sum) /
                                                        ((s_adc_occ_n - s_adc_prev_occ_n) ? (s_adc_occ_n - s_adc_prev_occ_n) : 1)
                                                    : 0),
                        (unsigned long)s_adc_prev_occ_min,
                        (unsigned long)s_adc_prev_occ_max,
                        (unsigned long)(s_adc_log_dt_ms
                            ? (uint32_t)((uint64_t)(s_adc_cb_calls - s_adc_prev_cb_calls) * 1000u / s_adc_log_dt_ms)
                            : 0),
                        (unsigned long)(s_adc_cb_calls - s_adc_prev_cb_calls
                            ? (s_adc_cb_bytes - s_adc_prev_cb_bytes) / (s_adc_cb_calls - s_adc_prev_cb_calls)
                            : 0),
                        (unsigned long)s_adc_ring_overflows,
                        (unsigned long)s_adc_starve_frames,
                        (long)((s_adc_ratio / ADC_RATIO_NOMINAL - 1.0f) * 1.0e6f),
                        (long)(ADC_RATIO_LIMIT * 1.0e6f));
                    s_adc_prev_drained = s_adc_drained_frames;
                    s_adc_prev_drain_calls = s_adc_drain_calls;
                    s_adc_prev_drain_zero = s_adc_drain_zero;
                    s_adc_prev_got = s_adc_drain_got_bytes;
                    s_adc_prev_consumed = s_adc_consumed_frames;
                    s_adc_prev_occ_sum = s_adc_occ_sum;
                    s_adc_prev_occ_n = s_adc_occ_n;
                    s_adc_prev_occ_min = s_adc_occ_min;
                    s_adc_prev_occ_max = s_adc_occ_max;
                    s_adc_prev_cb_calls = s_adc_cb_calls;
                    s_adc_prev_cb_bytes = s_adc_cb_bytes;
                    s_adc_ring_overflows = 0;
                    s_adc_starve_frames = 0;

                    pcm1865_report_status();
                }

                s_rx_gap_max_us = 0;
                s_rx_len_min = 0;
                s_rx_len_max = 0;
                prev_calls = calls;
                prev_frames = frames;
                prev_bytes = bytes;
                prev_t = tnow;
                have_prev = true;
            }
        }
        else
        {
            have_prev = false;
            on = false;

            if (s_uac_ready) led_set(0, 40, 0);
            else led_set(0, 0, 0);
        }

        uint32_t drops = s_drop_count;

        if (drops != s_last_reported_drops)
        {
            ESP_LOGW(
                TAG,
                "USB audio packets dropped: %lu",
                (unsigned long)(drops - s_last_reported_drops));

            s_last_reported_drops = drops;
        }

        uint32_t under = s_underrun_count;

        if (under != s_last_reported_underruns)
        {
            ESP_LOGW(
                TAG,
                "Playback underruns: %lu",
                (unsigned long)(under - s_last_reported_underruns));

            s_last_reported_underruns = under;
        }

        vTaskDelay(pdMS_TO_TICKS(150));
    }
}

// Configs the I2S capture channel
static void adc_i2s_init(void)
{
    i2s_chan_config_t cc =
        I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);

    cc.dma_desc_num = ADC_DMA_DESC_NUM;
    cc.dma_frame_num = ADC_DMA_FRAME_NUM;
    cc.auto_clear_after_cb = false;

    ESP_ERROR_CHECK(i2s_new_channel(&cc, NULL, &s_adc_rx));

    i2s_tdm_config_t cfg = {
        .clk_cfg = I2S_TDM_CLK_DEFAULT_CONFIG(ADC_RATE),

        .slot_cfg = I2S_TDM_MSB_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_32BIT,
            I2S_SLOT_MODE_STEREO,
            ADC_TDM_ACTIVE_MASK),

        .gpio_cfg = {
            .mclk = GPIO_NUM_NC,
            .bclk = ADC_BCK_GPIO,
            .ws = ADC_WS_GPIO,
            .dout = GPIO_NUM_NC,
            .din = ADC_DIN_GPIO,
        },
    };

    cfg.slot_cfg.slot_bit_width = I2S_SLOT_BIT_WIDTH_32BIT;
    cfg.slot_cfg.total_slot = ADC_TDM_TOTAL_SLOTS;
    cfg.slot_cfg.ws_pol = ADC_TDM_WS_POL;
    cfg.slot_cfg.bit_shift = ADC_TDM_BIT_SHIFT;
    cfg.clk_cfg.clk_src = I2S_CLK_SRC_APLL;
    cfg.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_768;

    ESP_ERROR_CHECK(i2s_channel_init_tdm_mode(s_adc_rx, &cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(s_adc_rx));

    gpio_pullup_dis(ADC_DIN_GPIO);
    gpio_pulldown_en(ADC_DIN_GPIO);

    ESP_LOGI(
        TAG,
        "ADC I2S initialized: %d Hz TDM, %d slots x 32-bit, mask 0x%02X, BCK %d Hz",
        ADC_RATE,
        ADC_TDM_TOTAL_SLOTS,
        ADC_TDM_ACTIVE_MASK,
        ADC_RATE * ADC_TDM_TOTAL_SLOTS * 32);
}

// Configs the I2S playback
static void dac_i2s_init(void)
{
    static const size_t ring_candidates[] = {RING_BYTES, 48 * 1024, 32 * 1024};
    for (size_t i = 0; i < sizeof(ring_candidates) / sizeof(ring_candidates[0]); i++) {
        size_t sz = ring_candidates[i];

        size_t storage_sz = sz + 1;
        uint8_t *storage = heap_caps_malloc(storage_sz, MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
        if (!storage) {
            ESP_LOGW(TAG, "heap_caps_malloc(%u KiB in PSRAM) failed, trying smaller",
                     (unsigned)(storage_sz / 1024));
            continue;
        }
        StaticStreamBuffer_t *buf_struct = heap_caps_malloc(sizeof(StaticStreamBuffer_t),
                                                             MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!buf_struct) {
            free(storage);
            ESP_LOGW(TAG, "StaticStreamBuffer_t malloc failed");
            continue;
        }
        s_audio_stream = xStreamBufferCreateStatic(sz, PCM_FRAME_BYTES, storage, buf_struct);
        if (s_audio_stream) {
            if (sz != RING_BYTES) {
                ESP_LOGW(TAG, "Heap could not fit the %u KiB ring; using %u KiB "
                              "(slightly less burst headroom)",
                         (unsigned)(RING_BYTES / 1024), (unsigned)(sz / 1024));
            }
            break;
        }
        free(storage);
        free(buf_struct);
        ESP_LOGW(TAG, "xStreamBufferCreateStatic(%u KiB) failed (PSRAM free %u KiB, internal free %u KiB)",
                 (unsigned)(sz / 1024),
                 (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024),
                 (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024));
    }

    ESP_ERROR_CHECK(
        s_audio_stream ? ESP_OK : ESP_ERR_NO_MEM);

    i2s_chan_config_t cc =
        I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);

    cc.dma_desc_num = 6;
    cc.dma_frame_num = 256;

    cc.auto_clear_after_cb = true;

    ESP_ERROR_CHECK(
        i2s_new_channel(&cc, &s_dac_tx, NULL));

    i2s_std_config_t cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(DAC_RATE),

        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_32BIT,
            I2S_SLOT_MODE_STEREO),

        .gpio_cfg = {
            .mclk = GPIO_NUM_NC,
            .bclk = DAC_BCK_GPIO,
            .ws = DAC_WS_GPIO,
            .dout = DAC_DIN_GPIO,
            .din = GPIO_NUM_NC,
        },
    };

    cfg.clk_cfg.clk_src = I2S_CLK_SRC_APLL;

    cfg.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_384;

    ESP_ERROR_CHECK(
        i2s_channel_init_std_mode(s_dac_tx, &cfg));

    ESP_ERROR_CHECK(i2s_channel_enable(s_dac_tx));

    ESP_LOGI(
        TAG,
        "I2S initialized: %d Hz, stereo, 32-bit slots, Philips (USB input: %d Hz, %d-bit/%d-byte samples)",
        DAC_RATE,
        DEFAULT_SAMPLE_RATE,
        CONFIG_UAC_BIT_RESOLUTION,
        CONFIG_UAC_BYTES_PER_SAMPLE);
}

// primes and pumps the ASRC into I2S
static void dac_playback_task(void *arg)
{
    (void)arg;

    static asrc_t asrc;
    static int32_t out[ASRC_BLOCK_FRAMES * 2];

    bool running = false;
    bool fade_in_pending = true;

    asrc_init(&asrc, in_pull, in_avail, NULL, TARGET_FILL_FRAMES);

    while (1)
    {
        if (__atomic_exchange_n(&s_playback_reset_requested, false, __ATOMIC_ACQ_REL)) {
            xStreamBufferReset(s_audio_stream);
            s_chunk_len = 0;
            s_chunk_pos = 0;
            asrc_init(&asrc, in_pull, in_avail, NULL, TARGET_FILL_FRAMES);
            running = false;
        }

        size_t avail = in_avail(NULL);

        if (!running)
        {

            if (avail < TARGET_FILL_FRAMES || !asrc_prime(&asrc))
            {
                vTaskDelay(1);
                continue;
            }

            running = true;
            fade_in_pending = true;

            ESP_LOGI(
                TAG,
                "Playback started with %u frames queued",
                (unsigned)avail);
        }

        asrc_update_ratio(&asrc);

        s_dbg_fill = (uint32_t)avail;
        s_dbg_ppm = (int32_t)((asrc.ratio / ASRC_NOMINAL_RATIO - 1.0f) * 1.0e6f);

        float gain = s_mute ? 0.0f : ((float)s_volume / 100.0f) * UAC_OUTPUT_GAIN_CAP;

        size_t produced = asrc_run(&asrc, out, ASRC_BLOCK_FRAMES, gain);
        if (fade_in_pending && produced > 0) {
            size_t fade_frames=produced<64?produced:64;
            for(size_t i=0;i<fade_frames;i++) {
                float fade=(float)(i+1)/(float)fade_frames;
                out[2*i]=(int32_t)((float)out[2*i]*fade);
                out[2*i+1]=(int32_t)((float)out[2*i+1]*fade);
            }
            fade_in_pending=false;
        }
        bool starved=produced<ASRC_BLOCK_FRAMES;
        if(starved && produced>0) {
            size_t fade_frames=produced<64?produced:64;
            size_t fade_start=produced-fade_frames;
            for(size_t i=fade_start;i<produced;i++) {
                float fade=(float)(produced-1-i)/(float)fade_frames;
                out[2*i]=(int32_t)((float)out[2*i]*fade);
                out[2*i+1]=(int32_t)((float)out[2*i+1]*fade);
            }
        }

        uint32_t peak = 0;
        double sum_sq[2] = {0.0, 0.0};
        for (size_t i=0; i<produced; i++) {
            spectrum_add_sample(0,out[2*i]);
            spectrum_add_sample(1,out[2*i+1]);
            for (int ch=0; ch<2; ch++) {
                int32_t sample=out[2*i+ch];
                uint32_t magnitude=sample<0?(uint32_t)(-(int64_t)sample):(uint32_t)sample;
                if(magnitude>peak) peak=magnitude;
                double normalized=(double)sample/8388607.0;
                sum_sq[ch]+=normalized*normalized;
            }
        }

        static float rms_sq[2];
        static int64_t clipping_hold_until_us;
        float dbfs[2];
        if(produced){
            float rms_alpha=1.0f-expf(-((float)produced/(float)DEFAULT_SAMPLE_RATE)/0.35f);
            for(int ch=0;ch<2;ch++){
                float block_rms_sq=(float)(sum_sq[ch]/produced);
                rms_sq[ch]+=(block_rms_sq-rms_sq[ch])*rms_alpha;
                dbfs[ch]=rms_sq[ch]>1.0e-12f?10.0f*log10f(rms_sq[ch]):-120.0f;
            }
        } else { dbfs[0]=dbfs[1]=-120.0f; }
        if(peak>=8388607U) clipping_hold_until_us=esp_timer_get_time()+2000000;

        int64_t now_us = esp_timer_get_time();
        if (s_adc_last_cb_us == 0 || now_us - s_adc_last_cb_us > 200000) {
            if (now_us - s_input_decay_us > 100000) {
                s_input_decay_us = now_us;
                for (int ch = 0; ch < MIC_CHANNEL_NUM; ch++) {
                    s_input_levels[ch] = (uint8_t)(s_input_levels[ch] * 4 / 5);
                }
            }
        }

        uint8_t meter[6];
        for (int ch = 0; ch < MIC_CHANNEL_NUM; ch++) meter[ch] = s_input_levels[ch];
        meter[4] = 0;
        meter[5] = 0;
        display_set_audio_levels(meter);
        display_set_output_meter(dbfs[0],dbfs[1],esp_timer_get_time()<clipping_hold_until_us);

        if (starved) {
            running = false;
            fade_in_pending = true;
            s_underrun_count++;
            s_playback_starves++;
        }

        static int32_t dac_out[ASRC_BLOCK_FRAMES * 2];
        for (size_t i = 0; i < produced * 2; i++) {
            dac_out[i] = out[i] * 256;
        }

        const uint8_t *p = (const uint8_t *)dac_out;
        size_t remaining = produced * DAC_FRAME_BYTES;

        while (remaining > 0)
        {
            size_t written = 0;
            int64_t write_start_us = esp_timer_get_time();

            esp_err_t err = i2s_channel_write(
                s_dac_tx, p, remaining, &written, I2S_WRITE_TIMEOUT_MS);

            uint32_t write_us = (uint32_t)(esp_timer_get_time() - write_start_us);
            if (write_us > s_i2s_write_max_us) s_i2s_write_max_us = write_us;

            if (err != ESP_OK || written == 0)
            {
                s_i2s_write_errors++;
                ESP_LOGE(
                    TAG,
                    "I2S write failed: %s",
                    esp_err_to_name(err));
                running = false;
                break;
            }

            p += written;
            remaining -= written;
        }
    }
}

// audio + composite descriptors
static void uac2_usb_init(void)
{
    if (s_uac_ready) return;
    uac_device_config_t uac_cfg = {
        .skip_tinyusb_init = false,
        .output_cb = uac_output_cb,
        .input_cb = uac_input_cb,
        .set_mute_cb = uac_mute_cb,
        .set_volume_cb = uac_volume_cb,
        .cb_ctx = NULL,
    };
    ESP_ERROR_CHECK(uac_device_init(&uac_cfg));
    s_uac_ready = true;

    TaskHandle_t tusb_task = xTaskGetHandle("TinyUSB");
    if (tusb_task) {
        vTaskPrioritySet(tusb_task, 9);
        ESP_LOGI(TAG, "TinyUSB task priority set to 9");
    } else {
        ESP_LOGW(TAG, "TinyUSB task not found; priorities unchanged");
    }
    TaskHandle_t spk = xTaskGetHandle("usb_spk_task");
    if (spk) {
        vTaskPrioritySet(spk, 6);
        ESP_LOGI(TAG, "usb_spk_task priority set to 6 (below TinyUSB)");
    } else {
        ESP_LOGW(TAG, "usb_spk_task not found; priority unchanged");
    }
}

// starts the stack for boot-time host detection
void uac2_probe_start(void)
{
    uac2_usb_init();
}

bool uac2_high_speed_host_connected(void)
{
    return tud_mounted() && tud_speed_get() == TUSB_SPEED_HIGH &&
           !uac_device_is_suspended();
}

// initialises it all
void uac2_start(void)
{
    nvs_handle_t gain_nvs;
    if(nvs_open("audio",NVS_READONLY,&gain_nvs)==ESP_OK) {
        for(uint8_t i=0;i<6;i++) {
            char key[8]; snprintf(key,sizeof(key),"ch%u",(unsigned)i);
            int8_t value=0;
            if(nvs_get_i8(gain_nvs,key,&value)==ESP_OK) {
                if(value < -12)value=-12;
                if(value > 12)value=12;
                s_channel_gain_db[i]=value;
            }
        }
        nvs_close(gain_nvs);
    }
    display_set_channel_gain(0 , s_channel_gain_db[4]);
    display_set_main_volume((uint8_t)s_volume);
    led_init();

    ESP_LOGI(
        TAG,
        "USB config: speaker=%d ch, mic=%d ch, %d Hz, %d-bit (%d B/sample); DAC=%d Hz stereo 32-bit slots",
        SPEAK_CHANNEL_NUM,
        MIC_CHANNEL_NUM,
        DEFAULT_SAMPLE_RATE,
        CONFIG_UAC_BIT_RESOLUTION,
        CONFIG_UAC_BYTES_PER_SAMPLE,
        DAC_RATE);

    adc_i2s_init();
    if (pcm1865_init() != ESP_OK) {

        ESP_LOGW(TAG, "PCM1865 unavailable; microphone inputs will be silent");
    }

    dac_i2s_init();

    {
        uint32_t apll_hz = 0;
        esp_err_t aperr = esp_clk_tree_src_get_freq_hz(
            SOC_MOD_CLK_APLL, ESP_CLK_TREE_SRC_FREQ_PRECISION_EXACT, &apll_hz);
        ESP_LOGI(TAG, "shared APLL after both I2S inits: %lu Hz (err=%s, ADC+DAC both need mclk 36864000)",
                 (unsigned long)apll_hz, esp_err_to_name(aperr));
    }

    BaseType_t ok = xTaskCreatePinnedToCore(
        dac_playback_task,
        "dac_playback",
        6144,
        NULL,
        5,
        NULL,
        1);

    ESP_ERROR_CHECK(ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);

    ok = xTaskCreate(
        led_task,
        "led",
        3072,
        NULL,
        3,
        NULL);

    ESP_ERROR_CHECK(ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);

    uac2_usb_init();

    ESP_LOGI(
        TAG,
        "USB audio initialized; output routed to PCM5100A, input from PCM1865 (%s)",
        pcm1865_present() ? "4ch TDM 48 kHz" : "codec missing");
}
