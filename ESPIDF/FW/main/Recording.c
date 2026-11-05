// Standalone recorder in 24-bit WAV

#include "Recording.h"
#include "Standalone.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/unistd.h>
#include <dirent.h>
#include <inttypes.h>
#include <math.h>

#include "Display.h"
#include "PCM1865.h"
#include "StatusLeds.h"
#include "driver/gpio.h"
#include "driver/i2s_tdm.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdmmc_cmd.h"

#include "InputsPreset.h"

#define SD_MOUNT_POINT "/sdcard"
#define RECORDINGS_DIR SD_MOUNT_POINT "/Recordings"
#define AUDIO_FRAMES 512
#define MIC_CHANNEL_MAX 4
#define ADC_BCK_GPIO GPIO_NUM_13
#define ADC_WS_GPIO GPIO_NUM_14
#define ADC_DIN_GPIO GPIO_NUM_12
#define ADC_DMA_DESC_NUM 6
#define ADC_DMA_FRAME_NUM 128

#define RECORD_WRITE_QUEUE_LEN_BASE 16
#define PCM24_MAX 8388607
#define PCM24_MIN (-8388608)
#define PCM24_CLIP_THRESHOLD 8220835
#define RECORD_BLOCK_MAX_BYTES \
    (ADC_DMA_DESC_NUM * ADC_DMA_FRAME_NUM * MIC_CHANNEL_MAX * 3)
#define RECORD_BUFFER_WAIT_MS_BASE 5

static const char *TAG = "recording";

typedef enum {
    REC_IDLE,
    REC_STARTING,
    REC_RECORDING,
    REC_STOPPING,
    REC_ERROR
} recording_state_t;

typedef struct {
    uint32_t rate;
    uint32_t data_bytes;
    uint16_t channels;
    uint16_t bits;
    uint32_t data_offset;
} wav_header_info_t;

static volatile recording_state_t s_state = REC_IDLE;
static i2s_chan_handle_t s_adc_rx = NULL;
static bool s_adc_rx_enabled;
static FILE *s_wav_file = NULL;
static char s_current_filename[256];
static uint32_t s_frames_written = 0;
static uint64_t s_recording_start_us = 0;
static int32_t s_adc_dma[ADC_DMA_DESC_NUM * ADC_DMA_FRAME_NUM * MIC_CHANNEL_MAX];
static QueueHandle_t s_commands = NULL;
static TaskHandle_t s_recording_task = NULL;
static SemaphoreHandle_t s_recording_stopped = NULL;
static QueueHandle_t s_write_queue;
static QueueHandle_t s_free_buffer_queue;
static SemaphoreHandle_t s_writer_stopped;
static bool s_writer_task_running;

static uint8_t (*s_record_buffers)[RECORD_BLOCK_MAX_BYTES];
static uint32_t s_writer_max_write_us;
static float s_channel_rms_db[MIC_CHANNEL_MAX];
static volatile bool s_sd_mounted = false;
static int s_next_rec_number = 1;
static uint8_t s_rec_ch = 4;
static uint32_t s_rec_rate = 48000;
static bool s_rec_wxyz;
static volatile bool s_auto_gain_enabled;
static float s_auto_gain=1.0f;

// queue helpers for writer
static inline int recording_write_queue_len(void)
{
    if (s_rec_rate <= 48000) return RECORD_WRITE_QUEUE_LEN_BASE;
    if (s_rec_rate <= 96000) return RECORD_WRITE_QUEUE_LEN_BASE * 2;
    return RECORD_WRITE_QUEUE_LEN_BASE * 4;
}

static inline int recording_buffer_wait_ms(void)
{
    if (s_rec_rate <= 48000) return RECORD_BUFFER_WAIT_MS_BASE;
    if (s_rec_rate <= 96000) return RECORD_BUFFER_WAIT_MS_BASE * 2;
    return RECORD_BUFFER_WAIT_MS_BASE * 4;
}

typedef enum {
    CMD_REC_START_STOP,
    CMD_REC_STOP
} rec_command_t;

typedef struct {
    uint8_t *data;
    uint32_t bytes;
    size_t frames;
} recording_write_block_t;

// auto input gain
void recording_set_auto_gain(bool enabled)
{
    s_auto_gain_enabled=enabled;
}

bool recording_auto_gain_enabled(void)
{
    return s_auto_gain_enabled;
}

// gets next free REC number and other stuf
static bool ensure_recordings_dir(void)
{
    struct stat st;
    if (stat(RECORDINGS_DIR, &st) == 0) {
        if (S_ISDIR(st.st_mode)) return true;
    }
    if (mkdir(RECORDINGS_DIR, 0755) == 0) {
        ESP_LOGI(TAG, "Created Recordings directory");
        return true;
    }
    ESP_LOGE(TAG, "Failed to create Recordings directory");
    return false;
}

static void find_next_rec_number(void)
{
    DIR *dir = opendir(RECORDINGS_DIR);
    if (!dir) return;
    struct dirent *entry;
    int max_num = 0;
    while ((entry = readdir(dir))) {
        if (strncmp(entry->d_name, "REC", 3) == 0 && strlen(entry->d_name) >= 7) {
            char *end;
            long num = strtol(entry->d_name + 3, &end, 10);
            if (end != entry->d_name + 3 && *end == '.' && num > max_num) {
                max_num = (int)num;
            }
        }
    }
    closedir(dir);
    s_next_rec_number = max_num + 1;
}

// writes the RIFF/WAVE headers
static bool write_wav_header(FILE *f, uint32_t rate, uint16_t channels, uint16_t bits, uint32_t data_bytes)
{
    uint16_t block_align = channels * (bits / 8);
    uint32_t byte_rate = rate * block_align;
    uint32_t riff_size = 36 + data_bytes;

    fwrite("RIFF", 1, 4, f);
    uint32_t le32 = riff_size;
    fwrite(&le32, 1, 4, f);
    fwrite("WAVE", 1, 4, f);

    fwrite("fmt ", 1, 4, f);
    le32 = 16;
    fwrite(&le32, 1, 4, f);
    uint16_t le16 = 1;
    fwrite(&le16, 1, 2, f);
    le16 = channels;
    fwrite(&le16, 1, 2, f);
    le32 = rate;
    fwrite(&le32, 1, 4, f);
    le32 = byte_rate;
    fwrite(&le32, 1, 4, f);
    le16 = block_align;
    fwrite(&le16, 1, 2, f);
    le16 = bits;
    fwrite(&le16, 1, 2, f);

    fwrite("data", 1, 4, f);
    le32 = data_bytes;
    fwrite(&le32, 1, 4, f);

    return ferror(f) == 0;
}

static bool update_wav_header(FILE *f, uint32_t data_bytes)
{
    uint32_t riff_size = 36 + data_bytes;
    fseek(f, 4, SEEK_SET);
    fwrite(&riff_size, 1, 4, f);
    fseek(f, 40, SEEK_SET);
    fwrite(&data_bytes, 1, 4, f);
    fflush(f);
    return ferror(f) == 0;
}

// Sets up the recording channel in I2S
static esp_err_t adc_i2s_init(void)
{
    const input_preset_t *preset = inp_preset();

    i2s_chan_config_t cc = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    cc.dma_desc_num = ADC_DMA_DESC_NUM;
    cc.dma_frame_num = ADC_DMA_FRAME_NUM;
    cc.auto_clear_after_cb = false;
    ESP_RETURN_ON_ERROR(i2s_new_channel(&cc, NULL, &s_adc_rx), TAG, "I2S channel create");

    i2s_tdm_config_t cfg = {
        .clk_cfg = I2S_TDM_CLK_DEFAULT_CONFIG(preset->rate),
        .slot_cfg = I2S_TDM_MSB_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_32BIT,
            I2S_SLOT_MODE_STEREO,
            preset->mask),
        .gpio_cfg = {
            .mclk = GPIO_NUM_NC,
            .bclk = ADC_BCK_GPIO,
            .ws = ADC_WS_GPIO,
            .dout = GPIO_NUM_NC,
            .din = ADC_DIN_GPIO,
        },
    };
    cfg.slot_cfg.slot_bit_width = I2S_SLOT_BIT_WIDTH_32BIT;
    cfg.slot_cfg.total_slot = preset->total_slots;
    cfg.slot_cfg.ws_pol = 1;
    cfg.slot_cfg.bit_shift = 0;
    cfg.clk_cfg.clk_src = I2S_CLK_SRC_APLL;
    cfg.clk_cfg.mclk_multiple = preset->mclk_mult == 256u ?
        I2S_MCLK_MULTIPLE_256 : I2S_MCLK_MULTIPLE_768;

    esp_err_t err=i2s_channel_init_tdm_mode(s_adc_rx,&cfg);
    if(err!=ESP_OK) {
        i2s_del_channel(s_adc_rx);
        s_adc_rx=NULL;
        return err;
    }
    err=i2s_channel_enable(s_adc_rx);
    if(err!=ESP_OK) {
        i2s_del_channel(s_adc_rx);
        s_adc_rx=NULL;
        return err;
    }
    s_adc_rx_enabled=true;

    gpio_pullup_dis(ADC_DIN_GPIO);
    gpio_pulldown_en(ADC_DIN_GPIO);

    ESP_LOGI(TAG, "ADC I2S initialized: %lu Hz TDM, %u slots x 32-bit, %u ch (%s)",
             (unsigned long)preset->rate, preset->total_slots, preset->channels,
             preset->name);
    return ESP_OK;
}

// Releases the capture channel
static void adc_i2s_deinit(void)
{
    if (s_adc_rx) {
        if(s_adc_rx_enabled) (void)i2s_channel_disable(s_adc_rx);
        i2s_del_channel(s_adc_rx);
        s_adc_rx = NULL;
        s_adc_rx_enabled=false;
    }
}

// drains da RAM buffers to le WAV file
static void recording_writer_task(void *arg)
{
    (void)arg;
    recording_write_block_t block;
    while(xQueueReceive(s_write_queue,&block,portMAX_DELAY)==pdTRUE) {
        if(!block.data) break;
        if(s_state!=REC_ERROR && s_wav_file) {
            int64_t write_start_us=esp_timer_get_time();
            size_t written=fwrite(block.data,1,block.bytes,s_wav_file);
            uint32_t write_us=(uint32_t)(esp_timer_get_time()-write_start_us);
            if(write_us>s_writer_max_write_us) s_writer_max_write_us=write_us;
            if(written!=block.bytes) {
                ESP_LOGE(TAG,"File write error: %u/%u",
                         (unsigned)written,(unsigned)block.bytes);
                s_state=REC_ERROR;
            } else {
                s_frames_written+=(uint32_t)block.frames;
            }
        }
        if(xQueueSend(s_free_buffer_queue,&block.data,0)!=pdTRUE) {
            ESP_LOGE(TAG,"Recording buffer pool invariant failed");
            s_state=REC_ERROR;
        }
    }
    xSemaphoreGive(s_writer_stopped);
    vTaskDelete(NULL);
}

// flushes the last buffers and finalises WAV header
static void recording_writer_finish(void)
{
    if(!s_writer_task_running) return;
    recording_write_block_t stop={0};
    if(xQueueSend(s_write_queue,&stop,portMAX_DELAY)!=pdTRUE) {
        ESP_LOGE(TAG,"Could not stop recording file writer");
        return;
    }
    if(xSemaphoreTake(s_writer_stopped,portMAX_DELAY)!=pdTRUE) {
        ESP_LOGE(TAG,"Could not join recording file writer");
        return;
    }
    s_writer_task_running=false;
    vQueueDelete(s_write_queue);
    s_write_queue=NULL;
    vQueueDelete(s_free_buffer_queue);
    s_free_buffer_queue=NULL;
    vSemaphoreDelete(s_writer_stopped);
    s_writer_stopped=NULL;

    heap_caps_free(s_record_buffers);
    s_record_buffers=NULL;
}

// tears down half-started recordings and shows error on the display
static void recording_startup_failure(const char *status)
{
    recording_writer_finish();
    adc_i2s_deinit();
    standalone_recording_monitor_stop();
    if(s_wav_file) {
        (void)update_wav_header(s_wav_file,0);
        fclose(s_wav_file);
        s_wav_file=NULL;
    }
    s_state=REC_ERROR;
    display_set_standalone_status(status);
    if(s_recording_stopped) xSemaphoreGive(s_recording_stopped);
}

// REC engine codec & I2S init, command handling, metering, auto gain + buffer plumbing
static void recording_task(void *arg)
{
    (void)arg;
    rec_command_t cmd;
    size_t got = 0;

    esp_err_t err = pcm1865_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "PCM1865 init failed: %s", esp_err_to_name(err));
        recording_startup_failure("PCM1865 INIT FAILED");
        vTaskDelete(NULL);
        return;
    }

    err = adc_i2s_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ADC I2S init failed: %s", esp_err_to_name(err));
        recording_startup_failure("ADC I2S INIT FAILED");
        vTaskDelete(NULL);
        return;
    }

    if (inp_preset_apply() != ESP_OK) {
        ESP_LOGW(TAG, "PCM1865 preset apply failed; default routing kept");
    }

    const input_preset_t *preset = inp_preset();
    s_rec_ch = preset->channels;
    s_rec_rate = preset->rate;
    s_rec_wxyz = (preset->ambisonic_slots == 4) && inp_rec_wxyz();
    if (preset->ambisonic_slots == 4) {

        display_set_recording_labels("W", "X", "Y", "Z", 4);
    } else if (s_rec_ch == 4) {
        display_set_recording_labels(preset->slot_label[0], preset->slot_label[1],
                                     preset->slot_label[2], preset->slot_label[3], 4);
    } else {
        display_set_recording_labels(preset->slot_label[0], preset->slot_label[1],
                                     NULL, NULL, 2);
    }

    for (int i = 0; i < 2 * ADC_DMA_DESC_NUM; i++) {
        size_t dummy = 0;
        i2s_channel_read(s_adc_rx, s_adc_dma, sizeof(s_adc_dma), &dummy, 0);
    }

    if (status_leds_start() == ESP_OK) {
        status_leds_set_recording(true);
    } else {
        ESP_LOGW(TAG, "Status LEDs unavailable; recording continues without LED indication");
    }

    s_state = REC_RECORDING;
    display_set_recording_active(true);
    s_frames_written = 0;
    s_auto_gain=1.0f;
    s_recording_start_us = esp_timer_get_time();
    memset(s_channel_rms_db, 0, sizeof(s_channel_rms_db));

    ESP_LOGI(TAG, "Recording started: %s", s_current_filename);
    display_set_standalone_status("RECORDING...");

    while (s_state == REC_RECORDING) {

        if (xQueueReceive(s_commands, &cmd, 0) == pdTRUE) {
            if (cmd == CMD_REC_STOP) {
                s_state = REC_STOPPING;
                break;
            }
        }

        got = 0;
        esp_err_t read_err = i2s_channel_read(s_adc_rx, s_adc_dma, sizeof(s_adc_dma), &got, pdMS_TO_TICKS(100));
        if (read_err != ESP_OK && read_err != ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "I2S read error: %s", esp_err_to_name(read_err));
            s_state = REC_ERROR;
            break;
        }

        if (got == 0) continue;

        size_t frames = got / (s_rec_ch * sizeof(int32_t));
        if (frames == 0) continue;

        if (standalone_recording_monitor_is_enabled()) {
            size_t offset = 0;
            while (offset < frames) {
                size_t block = frames - offset;
                if (block > 256) block = 256;
                if (!standalone_recording_monitor_process(
                        &s_adc_dma[offset * s_rec_ch], block)) {
                    ESP_LOGE(TAG, "Live monitor output failed during recording");
                    standalone_recording_monitor_stop();
                    display_set_standalone_status("REC MONITOR ERROR");
                    break;
                }
                offset += block;
            }
        }

        uint32_t data_bytes = frames * s_rec_ch * 3;
        if(data_bytes>RECORD_BLOCK_MAX_BYTES) {
            ESP_LOGE(TAG,"Capture block exceeds recording buffer capacity: %lu > %u",
                     (unsigned long)data_bytes,(unsigned)RECORD_BLOCK_MAX_BYTES);
            s_state = REC_ERROR;
            break;
        }
        uint8_t *out_buf=NULL;
        if(xQueueReceive(s_free_buffer_queue,&out_buf,pdMS_TO_TICKS(recording_buffer_wait_ms()))!=pdTRUE) {
            ESP_LOGE(TAG,"Recording writer stalled: write queue depth=%u, max write=%lu us",
                     (unsigned)uxQueueMessagesWaiting(s_write_queue),
                     (unsigned long)s_writer_max_write_us);
            s_state=REC_ERROR;
            break;
        }

        uint8_t *out_ptr = out_buf;
        float meter_sum_sq[MIC_CHANNEL_MAX] = {0};
        double gain_sum_sq=0.0;
        float gain_peak=0.0f;
        bool clipping_this_block = false;

        for (size_t i = 0; i < frames; i++) {
            const int32_t *src = &s_adc_dma[i * s_rec_ch];
            int32_t sample[MIC_CHANNEL_MAX];

            for (int ch = 0; ch < s_rec_ch; ch++) {
                int32_t input_sample = src[ch] >> 8;
                if (input_sample >= PCM24_CLIP_THRESHOLD ||
                    input_sample <= -PCM24_CLIP_THRESHOLD) {
                    clipping_this_block = true;
                }
            }

            if (s_rec_wxyz) {

                const uint8_t *m = inp_mic_map();
                int32_t a = src[m[0]] >> 8, b = src[m[1]] >> 8;
                int32_t c = src[m[2]] >> 8, d = src[m[3]] >> 8;
                sample[0] = (a + b + c + d) / 2;
                sample[1] = (a + b - c - d) / 2;
                sample[2] = (a - b + c - d) / 2;
                sample[3] = (a - b - c + d) / 2;
            } else {

                for (int ch = 0; ch < s_rec_ch; ch++) sample[ch] = src[ch] >> 8;
            }

            int32_t meter_sample[MIC_CHANNEL_MAX];
            if (preset->ambisonic_slots == 4) {
                const uint8_t *m = inp_mic_map();
                int32_t a = src[m[0]] >> 8, b = src[m[1]] >> 8;
                int32_t c = src[m[2]] >> 8, d = src[m[3]] >> 8;
                meter_sample[0] = (a + b + c + d) / 2;
                meter_sample[1] = (a + b - c - d) / 2;
                meter_sample[2] = (a - b + c - d) / 2;
                meter_sample[3] = (a - b - c + d) / 2;
            } else {
                for (int ch = 0; ch < s_rec_ch; ch++)
                    meter_sample[ch] = sample[ch];
            }

            for (int ch = 0; ch < s_rec_ch; ch++) {
                int32_t s = sample[ch];
                float normalized=(float)s/8388608.0f;
                float magnitude=fabsf(normalized);
                gain_sum_sq+=(double)normalized*(double)normalized;
                if(magnitude>gain_peak) gain_peak=magnitude;

                if (s >= PCM24_CLIP_THRESHOLD || s <= -PCM24_CLIP_THRESHOLD) {
                    clipping_this_block = true;
                }
                if(s_auto_gain_enabled) {
                    s=(int32_t)((float)s*s_auto_gain);
                    if(s >= PCM24_CLIP_THRESHOLD || s <= -PCM24_CLIP_THRESHOLD)
                        clipping_this_block=true;
                }
                if (s > PCM24_MAX) s = PCM24_MAX;
                if (s < PCM24_MIN) s = PCM24_MIN;

                float meter_normalized = (float)meter_sample[ch] / 8388608.0f;
                meter_sum_sq[ch] += meter_normalized * meter_normalized;

                *out_ptr++ = (uint8_t)(s & 0xFF);
                *out_ptr++ = (uint8_t)((s >> 8) & 0xFF);
                *out_ptr++ = (uint8_t)((s >> 16) & 0xFF);
            }
        }

        if (clipping_this_block) {
            status_leds_flash_clip();
        }
        if(s_auto_gain_enabled && gain_peak>1.0e-4f) {
            const float rms=sqrtf((float)(gain_sum_sq/(double)(frames*s_rec_ch)));
            float desired_gain=0.12589254f/rms;
            if(desired_gain>8.0f) desired_gain=8.0f;
            if(desired_gain<0.1f) desired_gain=0.1f;
            float peak_limit=0.98f/gain_peak;
            if(desired_gain>peak_limit) desired_gain=peak_limit;
            if(desired_gain<0.1f) desired_gain=0.1f;
            const float block_seconds=(float)frames/(float)s_rec_rate;
            const float time_constant=desired_gain<s_auto_gain?0.05f:1.0f;
            const float alpha=expf(-block_seconds/time_constant);
            s_auto_gain=desired_gain+(s_auto_gain-desired_gain)*alpha;
            if(s_auto_gain>8.0f) s_auto_gain=8.0f;
            if(s_auto_gain<0.1f) s_auto_gain=0.1f;
        } else if(!s_auto_gain_enabled) {
            s_auto_gain=1.0f;
        }

        recording_write_block_t write_block={
            .data=out_buf,
            .bytes=data_bytes,
            .frames=frames,
        };
        if(xQueueSend(s_write_queue,&write_block,0)!=pdTRUE) {
            ESP_LOGE(TAG,"Recording pipeline overrun: write queue depth=%u, max write=%lu us",
                     (unsigned)uxQueueMessagesWaiting(s_write_queue),
                     (unsigned long)s_writer_max_write_us);
            (void)xQueueSend(s_free_buffer_queue,&out_buf,0);
            s_state=REC_ERROR;
            break;
        }

        for (int ch = 0; ch < s_rec_ch; ch++) {
            float rms_sq = meter_sum_sq[ch] / frames;
            float db = rms_sq > 1e-12f ? 10.0f * log10f(rms_sq) : -120.0f;

            float alpha = 1.0f - expf(-(float)frames / (float)s_rec_rate / 0.3f);
            s_channel_rms_db[ch] += (db - s_channel_rms_db[ch]) * alpha;
        }

        uint32_t elapsed_ms = (uint32_t)((esp_timer_get_time() - s_recording_start_us) / 1000);
        uint32_t elapsed_sec = elapsed_ms / 1000;
        uint32_t min = elapsed_sec / 60;
        uint32_t sec = elapsed_sec % 60;

        char status[64];
        if (s_rec_ch >= 4) {
            snprintf(status, sizeof(status), "REC %02lu:%02lu %+.0f %+.0f %+.0f %+.0f dB",
                     (unsigned long)min, (unsigned long)sec,
                     s_channel_rms_db[0], s_channel_rms_db[1],
                     s_channel_rms_db[2], s_channel_rms_db[3]);
        } else {
            snprintf(status, sizeof(status), "REC %02lu:%02lu %+.0f %+.0f dB",
                     (unsigned long)min, (unsigned long)sec,
                     s_channel_rms_db[0], s_channel_rms_db[1]);
        }
        display_set_standalone_status(status);

        uint8_t meter[6] = {0};
        for (int ch = 0; ch < s_rec_ch; ch++) {
            float db = s_channel_rms_db[ch];
            if (db < -60.0f) db = -60.0f;
            if (db > 0.0f) db = 0.0f;
            meter[ch] = (uint8_t)((db + 60.0f) * 100.0f / 60.0f);
        }
        display_set_audio_levels(meter);
    }

    recording_writer_finish();

    if (s_wav_file) {
        uint32_t data_bytes = s_frames_written * s_rec_ch * 3;
        update_wav_header(s_wav_file, data_bytes);
        fclose(s_wav_file);
        s_wav_file = NULL;
        ESP_LOGI(TAG, "Recording stopped: %s (%lu frames, %lu bytes)", s_current_filename,
                 (unsigned long)s_frames_written, (unsigned long)data_bytes);
    }

    adc_i2s_deinit();
    standalone_recording_monitor_stop();
    display_set_recording_active(false);
    status_leds_set_recording(false);
    status_leds_stop();

    if (s_state == REC_ERROR) {
        display_set_standalone_status("RECORDING ERROR");
    } else {
        char msg[64];
        uint32_t elapsed_ms = (uint32_t)((esp_timer_get_time() - s_recording_start_us) / 1000);
        uint32_t elapsed_sec = elapsed_ms / 1000;
        snprintf(msg, sizeof(msg), "SAVED %02lu:%02lu", (unsigned long)(elapsed_sec/60), (unsigned long)(elapsed_sec%60));
        display_set_standalone_status(msg);
    }

    s_state = REC_IDLE;
    xSemaphoreGive(s_recording_stopped);
    vTaskDelete(NULL);
}

// cleans error state & allocates buffers
esp_err_t recording_start(void)
{
    if (s_state == REC_ERROR) {
        if (s_recording_stopped) {
            if (xSemaphoreTake(s_recording_stopped, portMAX_DELAY) != pdTRUE)
                return ESP_FAIL;
            vSemaphoreDelete(s_recording_stopped);
            s_recording_stopped = NULL;
        }
        if (s_commands) {
            vQueueDelete(s_commands);
            s_commands = NULL;
        }
        if (s_wav_file) {
            fclose(s_wav_file);
            s_wav_file = NULL;
        }
        heap_caps_free(s_record_buffers);
        s_record_buffers = NULL;
        s_recording_task = NULL;
        s_state = REC_IDLE;
    }
    if (s_state != REC_IDLE) {
        ESP_LOGW(TAG, "Recording already in progress");
        return ESP_ERR_INVALID_STATE;
    }

    if (!s_sd_mounted) {
        ESP_LOGE(TAG, "SD card not mounted");
        return ESP_ERR_INVALID_STATE;
    }

    if (!ensure_recordings_dir()) {
        return ESP_FAIL;
    }

    find_next_rec_number();

    snprintf(s_current_filename, sizeof(s_current_filename), "%s/REC%03d.WAV", RECORDINGS_DIR, s_next_rec_number);
    s_next_rec_number++;

    const char *fname = strrchr(s_current_filename, '/');
    display_set_recording_track(fname ? fname + 1 : s_current_filename);

    s_wav_file = fopen(s_current_filename, "wb");
    if (!s_wav_file) {
        ESP_LOGE(TAG, "Failed to create WAV file: %s", s_current_filename);
        return ESP_FAIL;
    }

    const input_preset_t *preset = inp_preset();
    s_rec_ch = preset->channels;
    s_rec_rate = preset->rate;
    if (!write_wav_header(s_wav_file, s_rec_rate, s_rec_ch, 24, 0)) {
        fclose(s_wav_file);
        s_wav_file = NULL;
        return ESP_FAIL;
    }

    s_commands = xQueueCreate(4, sizeof(rec_command_t));
    if (!s_commands) {
        fclose(s_wav_file);
        s_wav_file = NULL;
        return ESP_ERR_NO_MEM;
    }

    s_recording_stopped = xSemaphoreCreateBinary();
    if (!s_recording_stopped) {
        vQueueDelete(s_commands);
        s_commands = NULL;
        fclose(s_wav_file);
        s_wav_file = NULL;
        return ESP_ERR_NO_MEM;
    }

    int qlen = recording_write_queue_len();

    s_record_buffers=heap_caps_calloc(qlen,RECORD_BLOCK_MAX_BYTES,MALLOC_CAP_SPIRAM);
    if(!s_record_buffers) {
        ESP_LOGE(TAG,"No memory for %d recording buffers (%u bytes each)",
                 qlen,(unsigned)RECORD_BLOCK_MAX_BYTES);
        vQueueDelete(s_commands);
        s_commands=NULL;
        vSemaphoreDelete(s_recording_stopped);
        s_recording_stopped=NULL;
        fclose(s_wav_file);
        s_wav_file=NULL;
        return ESP_ERR_NO_MEM;
    }
    s_write_queue=xQueueCreate(qlen,sizeof(recording_write_block_t));
    s_free_buffer_queue=xQueueCreate(qlen,sizeof(uint8_t *));
    s_writer_stopped=xSemaphoreCreateBinary();
    if(!s_write_queue || !s_free_buffer_queue || !s_writer_stopped) {
        if(s_write_queue) {
            vQueueDelete(s_write_queue);
            s_write_queue=NULL;
        }
        if(s_free_buffer_queue) {
            vQueueDelete(s_free_buffer_queue);
            s_free_buffer_queue=NULL;
        }
        if(s_writer_stopped) {
            vSemaphoreDelete(s_writer_stopped);
            s_writer_stopped=NULL;
        }
        heap_caps_free(s_record_buffers);
        s_record_buffers=NULL;
        vQueueDelete(s_commands);
        s_commands=NULL;
        vSemaphoreDelete(s_recording_stopped);
        s_recording_stopped=NULL;
        fclose(s_wav_file);
        s_wav_file=NULL;
        return ESP_ERR_NO_MEM;
    }

    s_writer_max_write_us=0;
    for(int i=0;i<qlen;i++) {
        uint8_t *buffer=s_record_buffers[i];
        if(xQueueSend(s_free_buffer_queue,&buffer,0)!=pdTRUE) {
            vQueueDelete(s_write_queue);
            vQueueDelete(s_free_buffer_queue);
            vSemaphoreDelete(s_writer_stopped);
            s_write_queue=NULL;
            s_free_buffer_queue=NULL;
            s_writer_stopped=NULL;
            heap_caps_free(s_record_buffers);
            s_record_buffers=NULL;
            vQueueDelete(s_commands);
            s_commands=NULL;
            vSemaphoreDelete(s_recording_stopped);
            s_recording_stopped=NULL;
            fclose(s_wav_file);
            s_wav_file=NULL;
            return ESP_FAIL;
        }
    }

    if(xTaskCreatePinnedToCore(recording_writer_task,"rec_writer",4096,NULL,7,NULL,1)!=pdPASS) {
        vQueueDelete(s_write_queue);
        s_write_queue=NULL;
        vQueueDelete(s_free_buffer_queue);
        s_free_buffer_queue=NULL;
        vSemaphoreDelete(s_writer_stopped);
        s_writer_stopped=NULL;
        heap_caps_free(s_record_buffers);
        s_record_buffers=NULL;
        vQueueDelete(s_commands);
        s_commands=NULL;
        vSemaphoreDelete(s_recording_stopped);
        s_recording_stopped=NULL;
        fclose(s_wav_file);
        s_wav_file=NULL;
        return ESP_ERR_NO_MEM;
    }
    s_writer_task_running=true;

    s_state = REC_STARTING;

    BaseType_t created = xTaskCreate(recording_task, "recording", 8192, NULL, 5, &s_recording_task);
    if (created != pdPASS) {
        recording_writer_finish();
        vQueueDelete(s_commands);
        s_commands = NULL;
        vSemaphoreDelete(s_recording_stopped);
        s_recording_stopped = NULL;
        fclose(s_wav_file);
        s_wav_file = NULL;
        s_state = REC_IDLE;
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

// API to stop the running recording
esp_err_t recording_stop(void)
{
    if (s_state != REC_RECORDING && s_state != REC_STARTING) {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_commands) {
        rec_command_t cmd = CMD_REC_STOP;
        xQueueSend(s_commands, &cmd, pdMS_TO_TICKS(100));
    }

    if (s_recording_stopped) {
        xSemaphoreTake(s_recording_stopped, portMAX_DELAY);
        vSemaphoreDelete(s_recording_stopped);
        s_recording_stopped = NULL;
    }

    if (s_commands) {
        vQueueDelete(s_commands);
        s_commands = NULL;
    }

    s_recording_task = NULL;
    return ESP_OK;
}

// REC state queries other modules
bool recording_is_active(void)
{
    return s_state == REC_RECORDING || s_state == REC_STARTING;
}

bool recording_is_ready(void)
{
    return s_state==REC_RECORDING;
}

void recording_set_sd_mounted(bool mounted)
{
    s_sd_mounted = mounted;
}
