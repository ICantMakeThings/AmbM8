/*
 * SPDX-FileCopyrightText: 2023-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_private/usb_phy.h"
#include "esp_timer.h"
#include "tusb.h"
#include "uac_config.h"
#include "usb_device_uac.h"
#include "uac_descriptors.h"

static const char *TAG = "usbd_uac";

// 128 entries: ~250 ms of buffering headroom at 24 kHz packet rate. A deeper
// queue only absorbs usb_spk_task stalls; the steady-state latency target is
// unchanged because draining runs as fast as the stream buffer accepts.
#define UAC_SPK_QUEUE_DEPTH 256
#define UAC_SPK_PACKET_BYTES CFG_TUD_AUDIO_FUNC_1_FORMAT_1_EP_SZ_OUT

/*
 * Microphone chunk pipeline.
 *
 * The mic task generates one MIC_INTERVAL_MS chunk per tick; TinyUSB's ISO IN
 * flow controller drains the EP-IN FIFO at the same average rate but needs a
 * full chunk of free space before a chunk can be handed over.  With a single
 * double buffer, any timing skew between the two made the task overwrite (and
 * therefore lose) the chunk that had not been drained yet, which skipped
 * content and made recordings play fast/pitched-up.  Queue a few chunks so a
 * late hand-over only adds latency, never loses audio.
 */
#define UAC_MIC_SLOTS 3
#define UAC_MIC_SLOT_BYTES \
    ((CFG_TUD_AUDIO_FUNC_1_MAX_SAMPLE_RATE / 1000 * \
      CFG_TUD_AUDIO_FUNC_1_FORMAT_1_N_BYTES_PER_SAMPLE_TX * \
      CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX) * MIC_INTERVAL_MS + \
     CFG_TUD_AUDIO_FUNC_1_FORMAT_1_EP_SZ_IN)

typedef struct {
    uint16_t len;
    uint8_t data[UAC_SPK_PACKET_BYTES];
} uac_spk_packet_t;

const uint32_t sample_rates[] = {DEFAULT_SAMPLE_RATE};

#define N_SAMPLE_RATES  TU_ARRAY_SIZE(sample_rates)

enum {
    VOLUME_CTRL_0_DB = 0,
    VOLUME_CTRL_10_DB = 2560,
    VOLUME_CTRL_20_DB = 5120,
    VOLUME_CTRL_30_DB = 7680,
    VOLUME_CTRL_40_DB = 10240,
    VOLUME_CTRL_50_DB = 12800,
    VOLUME_CTRL_60_DB = 15360,
    VOLUME_CTRL_70_DB = 17920,
    VOLUME_CTRL_80_DB = 20480,
    VOLUME_CTRL_90_DB = 23040,
    VOLUME_CTRL_100_DB = 25600,
    VOLUME_CTRL_SILENCE = 0x8000,
};

// Resolution per format
const uint8_t spk_resolutions_per_format[CFG_TUD_AUDIO_FUNC_1_N_FORMATS] = {CFG_TUD_AUDIO_FUNC_1_FORMAT_1_RESOLUTION_RX};
const uint8_t mic_resolutions_per_format[CFG_TUD_AUDIO_FUNC_1_N_FORMATS] = {CFG_TUD_AUDIO_FUNC_1_FORMAT_1_RESOLUTION_TX};

typedef struct {
    usb_phy_handle_t phy_hdl;
    uac_device_config_t user_cfg;
    int8_t mute[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1];         // SPK feature unit (RX = OUT = SPK), +1 for master channel 0
    int16_t volume[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1];      // SPK feature unit (RX = OUT = SPK), +1 for master channel 0
    uint8_t mic_chunk[UAC_MIC_SLOTS][UAC_MIC_SLOT_BYTES];       // Microphone chunk ring
    uint16_t mic_slot_len[UAC_MIC_SLOTS];                        // Filled length of each chunk
    volatile uint8_t mic_slot_ready[UAC_MIC_SLOTS];              // 1 = filled, waiting to be sent
    volatile uint8_t mic_wr_slot;                                // Slot the mic task fills next
    volatile uint8_t mic_rd_slot;                                // Slot currently being sent
    volatile uint16_t mic_send_offset;                           // Bytes already sent from mic_rd_slot
    QueueHandle_t spk_queue;                                    // Queued USB speaker packets
    int spk_data_size;                                           // Speaker data size received in the last frame
    int mic_data_size;                                           // Bytes left to send from mic_rd_slot
    int spk_itf_num;
    int mic_itf_num;
    uint8_t spk_resolution;
    uint8_t mic_resolution;
    uint32_t current_sample_rate;                                // Current resolution, update on format change
    TaskHandle_t mic_task_handle;
    TaskHandle_t spk_task_handle;
    size_t spk_bytes_per_ms;
    size_t mic_bytes_per_ms;
    bool spk_active;
    bool mic_active;
} uac_device_t;

static uac_device_t *s_uac_device = NULL;
static volatile uint32_t s_spk_usb_packets;
static volatile uint32_t s_spk_usb_bytes;
static volatile uint32_t s_spk_read_bytes;
static volatile uint32_t s_spk_task_packets;
static volatile uint32_t s_spk_queue_highwater;
static volatile uint32_t s_spk_queue_drops;
static volatile uint32_t s_mic_tx_write_calls;
static volatile uint32_t s_mic_tx_write_bytes;
static volatile uint32_t s_mic_tx_partial_writes;
static volatile uint32_t s_mic_generated_bytes;
static volatile uint32_t s_mic_tx_callbacks;
static volatile uint32_t s_mic_tx_fifo_waits;
static volatile uint32_t s_mic_task_skips;
static volatile uint32_t s_mic_wire_bytes;
static volatile uint32_t s_mic_wire_packets;
static volatile uint32_t s_mic_wire_shorts;
static volatile uac_speaker_stats_t s_spk_stats_snapshot;
static volatile bool s_usb_suspended;

bool uac_device_get_speaker_stats(uac_speaker_stats_t *stats)
{
    if (stats == NULL) return false;
    stats->usb_packets_per_sec = s_spk_stats_snapshot.usb_packets_per_sec;
    stats->usb_bytes_per_sec = s_spk_stats_snapshot.usb_bytes_per_sec;
    stats->fifo_bytes_per_sec = s_spk_stats_snapshot.fifo_bytes_per_sec;
    stats->task_packets_per_sec = s_spk_stats_snapshot.task_packets_per_sec;
    stats->queue_high_water = s_spk_stats_snapshot.queue_high_water;
    stats->queue_depth = s_spk_stats_snapshot.queue_depth;
    stats->drops = s_spk_stats_snapshot.drops;
    return true;
}

bool uac_device_is_suspended(void)
{
    return s_usb_suspended;
}
static volatile uint16_t s_mic_tx_fifo_depth;
static volatile uint16_t s_mic_tx_fifo_free;
static volatile uint16_t s_mic_tx_bytes_required;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
#define UAC_ENTER_CRITICAL()    portENTER_CRITICAL(&s_mux)
#define UAC_EXIT_CRITICAL()     portEXIT_CRITICAL(&s_mux)

/* Flush device-side and application-side audio at stream boundaries so a
 * pause, seek, suspend, or reconnect cannot replay old queued samples. */
static void speaker_stream_reset(void)
{
    if (s_uac_device == NULL) {
        return;
    }
    if (s_uac_device->spk_queue != NULL) {
        xQueueReset(s_uac_device->spk_queue);
    }
    if (s_uac_device->user_cfg.output_cb != NULL) {
        s_uac_device->user_cfg.output_cb(NULL, 0, s_uac_device->user_cfg.cb_ctx);
    }
}

static void usb_phy_init(void)
{
    // Configure USB PHY
    usb_phy_config_t phy_conf = {
        .controller = USB_PHY_CTRL_OTG,
        .otg_mode = USB_OTG_MODE_DEVICE,
        .target = USB_PHY_TARGET_INT,
#if CONFIG_TINYUSB_RHPORT_HS
        .otg_speed = USB_PHY_SPEED_HIGH,
#endif
    };
    usb_new_phy(&phy_conf, &s_uac_device->phy_hdl);
}

static void tusb_device_task(void *arg)
{
    while (1) {
        tud_task();
    }
}

#if !CONFIG_USB_DEVICE_UAC_AS_PART
// Invoked when device is mounted
void tud_mount_cb(void)
{
    if (s_uac_device == NULL) {
        ESP_LOGI(TAG, "USB mounted without UAC device (mass-storage mode)");
        return;
    }
    s_uac_device->spk_active = false;
    s_uac_device->mic_active = false;
    ESP_LOGI(TAG, "USB mounted");
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
    if (s_uac_device != NULL) {
        s_uac_device->spk_active = false;
        s_uac_device->mic_active = false;
    }
    s_usb_suspended = true;
    speaker_stream_reset();
    ESP_LOGI(TAG, "USB unmounted");
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
    (void)remote_wakeup_en;
    if (s_uac_device != NULL) {
        s_uac_device->spk_active = false;
        s_uac_device->mic_active = false;
    }
    s_usb_suspended = true;
    speaker_stream_reset();
    ESP_LOGI(TAG, "USB suspended");
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
    s_usb_suspended = false;
    ESP_LOGI(TAG, "USB resumed");
}
#endif

// Helper for clock get requests
static bool tud_audio_clock_get_request(uint8_t rhport, audio_control_request_t const *request)
{
    TU_ASSERT(request->bEntityID == UAC2_ENTITY_CLOCK);

    if (request->bControlSelector == AUDIO_CS_CTRL_SAM_FREQ) {
        if (request->bRequest == AUDIO_CS_REQ_CUR) {
            TU_LOG1("Clock get current freq %lu\r\n", s_uac_device->current_sample_rate);
            audio_control_cur_4_t curf = { (int32_t) tu_htole32(s_uac_device->current_sample_rate) };
            return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &curf, sizeof(curf));
        } else if (request->bRequest == AUDIO_CS_REQ_RANGE) {
            audio_control_range_4_n_t(N_SAMPLE_RATES) rangef = {
                .wNumSubRanges = tu_htole16(N_SAMPLE_RATES)
            };
            TU_LOG1("Clock get %d freq ranges\r\n", N_SAMPLE_RATES);
            for (uint8_t i = 0; i < N_SAMPLE_RATES; i++) {
                rangef.subrange[i].bMin = (int32_t) sample_rates[i];
                rangef.subrange[i].bMax = (int32_t) sample_rates[i];
                rangef.subrange[i].bRes = 0;
                TU_LOG1("Range %d (%d, %d, %d)\r\n", i, (int)rangef.subrange[i].bMin, (int)rangef.subrange[i].bMax, (int)rangef.subrange[i].bRes);
            }

            return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &rangef, sizeof(rangef));
        }
    } else if (request->bControlSelector == AUDIO_CS_CTRL_CLK_VALID && request->bRequest == AUDIO_CS_REQ_CUR) {
        audio_control_cur_1_t cur_valid = {
            .bCur = 1
        };
        TU_LOG1("Clock get is valid %u\r\n", cur_valid.bCur);
        return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &cur_valid, sizeof(cur_valid));
    }

    TU_LOG1("Clock get request not supported, entity = %u, selector = %u, request = %u\r\n", request->bEntityID, request->bControlSelector, request->bRequest);
    return false;
}

// Helper for clock set requests
static bool tud_audio_clock_set_request(uint8_t rhport, audio_control_request_t const *request, uint8_t const *buf)
{
    (void)rhport;

    TU_ASSERT(request->bEntityID == UAC2_ENTITY_CLOCK);
    TU_VERIFY(request->bRequest == AUDIO_CS_REQ_CUR);

    if (request->bControlSelector == AUDIO_CS_CTRL_SAM_FREQ) {
        TU_VERIFY(request->wLength == sizeof(audio_control_cur_4_t));

        uint32_t target_sample_rate = (uint32_t)((audio_control_cur_4_t const *)buf)->bCur;
        TU_LOG1("Clock set current freq: %ld\r\n", target_sample_rate);

        if (target_sample_rate != s_uac_device->current_sample_rate) {
            // For now, we only support one sample rate
            return false;
        }

        return true;
    } else {
        TU_LOG1("Clock set request not supported, entity = %u, selector = %u, request = %u\r\n",
                request->bEntityID, request->bControlSelector, request->bRequest);
        return false;
    }
}

void tud_audio_feedback_params_cb(uint8_t func_id, uint8_t alt_itf, audio_feedback_params_t* feedback_param)
{
    (void)func_id;
    (void)alt_itf;
    /*
     * The microphone is a crystal-locked (APLL) source: the stream rate does
     * not follow the USB SOF. Report the nominal rate with a live feedback
     * endpoint so ALSA clocks the capture from the device-rated value. With
     * AUDIO_FEEDBACK_METHOD_DISABLED the FB EP exists in the descriptor but
     * is never serviced, ALSA has no rate reference, and every microframe
     * the endpoint cannot fill becomes frames missing from the recording
     * (recordings play fast / pitched up). The small residual drift between
     * the APLL and USB SOF stays within the FB interval tolerance; the
     * application's upsampler servo absorbs the rest.
     */
    feedback_param->method = AUDIO_FEEDBACK_METHOD_FREQUENCY_FIXED;
    feedback_param->sample_freq = s_uac_device->current_sample_rate;

    ESP_LOGD(TAG, "Feedback method: %d, sample freq: %"PRIu32"", feedback_param->method, feedback_param->sample_freq);
}

// Helper for feature unit get requests
static bool tud_audio_feature_unit_get_request(uint8_t rhport, audio_control_request_t const *request)
{
    TU_ASSERT(request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT);

    if (request->bControlSelector == AUDIO_FU_CTRL_MUTE && request->bRequest == AUDIO_CS_REQ_CUR) {
        audio_control_cur_1_t mute1 = {
            .bCur = s_uac_device->mute[request->bChannelNumber]
        };
        TU_LOG1("Get channel %u mute %d\r\n", request->bChannelNumber, mute1.bCur);
        return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &mute1, sizeof(mute1));
    } else if (UAC2_ENTITY_SPK_FEATURE_UNIT && request->bControlSelector == AUDIO_FU_CTRL_VOLUME) {
        if (request->bRequest == AUDIO_CS_REQ_RANGE) {
            audio_control_range_2_n_t(1) range_vol = {
                .wNumSubRanges = tu_htole16(1),
                .subrange[0] = { .bMin = tu_htole16(-VOLUME_CTRL_50_DB), tu_htole16(VOLUME_CTRL_0_DB), tu_htole16(256) }
            };
            TU_LOG1("Get channel %u volume range (%d, %d, %u) dB\r\n", request->bChannelNumber,
                    range_vol.subrange[0].bMin / 256, range_vol.subrange[0].bMax / 256, range_vol.subrange[0].bRes / 256);
            return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &range_vol, sizeof(range_vol));
        } else if (request->bRequest == AUDIO_CS_REQ_CUR) {
            audio_control_cur_2_t cur_vol = {
                .bCur = tu_htole16(s_uac_device->volume[request->bChannelNumber])
            };
            TU_LOG1("Get channel %u volume %d dB\r\n", request->bChannelNumber, cur_vol.bCur / 256);
            return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &cur_vol, sizeof(cur_vol));
        }
    }
    TU_LOG1("Feature unit get request not supported, entity = %u, selector = %u, request = %u\r\n",
            request->bEntityID, request->bControlSelector, request->bRequest);

    return false;
}

static bool tud_audio_feature_unit_set_request(uint8_t rhport, audio_control_request_t const *request, uint8_t const *buf)
{
    (void)rhport;

    TU_ASSERT(request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT);
    TU_VERIFY(request->bRequest == AUDIO_CS_REQ_CUR);

    if (request->bControlSelector == AUDIO_FU_CTRL_MUTE) {
        TU_VERIFY(request->wLength == sizeof(audio_control_cur_1_t));
        s_uac_device->mute[request->bChannelNumber] = ((audio_control_cur_1_t const *)buf)->bCur;
        TU_LOG1("Set speaker channel %d Mute: %d\r\n", request->bChannelNumber, s_uac_device->mute[request->bChannelNumber]);
        if (s_uac_device->user_cfg.set_mute_cb) {
            s_uac_device->user_cfg.set_mute_cb(s_uac_device->mute[request->bChannelNumber], s_uac_device->user_cfg.cb_ctx);
        }

        return true;
    } else if (request->bControlSelector == AUDIO_FU_CTRL_VOLUME) {
        TU_VERIFY(request->wLength == sizeof(audio_control_cur_2_t));
        s_uac_device->volume[request->bChannelNumber] = ((audio_control_cur_2_t const *)buf)->bCur;
        int volume_db = s_uac_device->volume[request->bChannelNumber] / 256; // Convert to dB
        int volume = (volume_db + 50) * 2; // Map to range 0 to 100
        TU_LOG1("Set speaker channel %d volume: %d dB (%d)\r\n", request->bChannelNumber, volume_db, volume);
        if (s_uac_device->user_cfg.set_volume_cb) {
            s_uac_device->user_cfg.set_volume_cb(volume, s_uac_device->user_cfg.cb_ctx);
        }
        return true;
    } else {
        TU_LOG1("Feature unit set request not supported, entity = %u, selector = %u, request = %u\r\n",
                request->bEntityID, request->bControlSelector, request->bRequest);
        return false;
    }
}

//--------------------------------------------------------------------+
// Application Callback API Implementations
//--------------------------------------------------------------------+

// Invoked when audio class specific get request received for an entity
bool tud_audio_get_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request)
{
    audio_control_request_t const *request = (audio_control_request_t const *)p_request;

    if (request->bEntityID == UAC2_ENTITY_CLOCK) {
        return tud_audio_clock_get_request(rhport, request);
    }
    if (request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT) {
        return tud_audio_feature_unit_get_request(rhport, request);
    } else {
        TU_LOG1("Get request not handled, entity = %d, selector = %d, request = %d\r\n",
                request->bEntityID, request->bControlSelector, request->bRequest);
    }
    return false;
}

// Invoked when audio class specific set request received for an entity
bool tud_audio_set_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *buf)
{
    audio_control_request_t const *request = (audio_control_request_t const *)p_request;

    if (request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT) {
        return tud_audio_feature_unit_set_request(rhport, request, buf);
    }
    if (request->bEntityID == UAC2_ENTITY_CLOCK) {
        return tud_audio_clock_set_request(rhport, request, buf);
    }
    TU_LOG1("Set request not handled, entity = %d, selector = %d, request = %d\r\n",
            request->bEntityID, request->bControlSelector, request->bRequest);

    return false;
}

bool tud_audio_set_itf_close_EP_cb(uint8_t rhport, tusb_control_request_t const *p_request)
{
    (void)rhport;

    uint8_t const itf = tu_u16_low(tu_le16toh(p_request->wIndex));
    uint8_t const alt = tu_u16_low(tu_le16toh(p_request->wValue));

#if SPEAK_CHANNEL_NUM
    if (s_uac_device->spk_itf_num == itf && alt == 0) {
        TU_LOG2("Speaker interface closed");
        s_uac_device->spk_data_size = 0;
        s_uac_device->spk_active = false;
        speaker_stream_reset();
    }
#endif

#if MIC_CHANNEL_NUM
    if (s_uac_device->mic_itf_num == itf && alt == 0) {
        TU_LOG2("Microphone interface closed");
        UAC_ENTER_CRITICAL();
        s_uac_device->mic_data_size = 0;
        s_uac_device->mic_send_offset = 0;
        s_uac_device->mic_rd_slot = s_uac_device->mic_wr_slot;
        for (int i = 0; i < UAC_MIC_SLOTS; i++) {
            s_uac_device->mic_slot_ready[i] = 0;
        }
        UAC_EXIT_CRITICAL();
        s_uac_device->mic_active = false;
    }
#endif

    return true;
}

bool tud_audio_set_itf_cb(uint8_t rhport, tusb_control_request_t const *p_request)
{
    (void)rhport;
    uint8_t const itf = tu_u16_low(tu_le16toh(p_request->wIndex));
    uint8_t const alt = tu_u16_low(tu_le16toh(p_request->wValue));

    TU_LOG2("Set interface %d alt %d\r\n", itf, alt);

#if SPEAK_CHANNEL_NUM
    if (s_uac_device->spk_itf_num == itf && alt != 0) {
        speaker_stream_reset();
        s_uac_device->spk_data_size = 0;
        s_uac_device->spk_resolution = spk_resolutions_per_format[alt - 1];
        s_uac_device->spk_active = true;
        s_uac_device->spk_bytes_per_ms = s_uac_device->current_sample_rate / 1000 * SPEAK_CHANNEL_NUM * CFG_TUD_AUDIO_FUNC_1_FORMAT_1_N_BYTES_PER_SAMPLE_RX;
        /* HS feedback is samples per 125 us microframe, in 16.16 format. */
        uint32_t feedback = (uint32_t)(((uint64_t)s_uac_device->current_sample_rate << 16) / 8000);
        tud_audio_n_fb_set(0, feedback);
        xTaskNotifyGive(s_uac_device->spk_task_handle);
        TU_LOG1("Speaker interface %d-%d opened", itf, alt);
        printf("Speaker interface %d-%d opened\n", itf, alt);
    }
#endif

#if MIC_CHANNEL_NUM
    if (s_uac_device->mic_itf_num == itf && alt != 0) {
        UAC_ENTER_CRITICAL();
        s_uac_device->mic_data_size = 0;
        s_uac_device->mic_send_offset = 0;
        s_uac_device->mic_wr_slot = 0;
        s_uac_device->mic_rd_slot = 0;
        for (int i = 0; i < UAC_MIC_SLOTS; i++) {
            s_uac_device->mic_slot_ready[i] = 0;
        }
        UAC_EXIT_CRITICAL();
        s_uac_device->mic_resolution = mic_resolutions_per_format[alt - 1];
        s_uac_device->mic_active = true;
        s_uac_device->mic_bytes_per_ms = s_uac_device->current_sample_rate / 1000 * MIC_CHANNEL_NUM * CFG_TUD_AUDIO_FUNC_1_FORMAT_1_N_BYTES_PER_SAMPLE_TX;
        // Prefill to the flow-control setpoint (depth/2): the controller only
        // corrects ~0.4% per packet, so from an empty FIFO it climbs for
        // seconds, sending audible zero-length packets on the way. Mirrors
        // the speaker path's buffered start.
        tu_fifo_t *in_ff = tud_audio_get_ep_in_ff();
        if (in_ff != NULL) {
            static const uint8_t zeros[64] = {0};
            uint16_t want = tu_fifo_depth(in_ff) / 2;
            while (want > 0) {
                uint16_t chunk = (want > sizeof(zeros)) ? (uint16_t)sizeof(zeros) : want;
                if (tud_audio_write(zeros, chunk) != chunk) {
                    break;
                }
                want -= chunk;
            }
        }
        xTaskNotifyGive(s_uac_device->mic_task_handle);
        TU_LOG1("Microphone interface %d-%d opened", itf, alt);
        printf("Microphone interface %d-%d opened\n", itf, alt);
    }
#endif

    return true;
}

bool tud_audio_rx_done_isr(uint8_t rhport, uint16_t n_bytes_received, uint8_t func_id, uint8_t ep_out, uint8_t cur_alt_setting)
{
    (void)rhport;
    (void)ep_out;
    (void)cur_alt_setting;

    /* Preserve each USB packet: the component's former single shared buffer
     * could be overwritten by the next packet before usb_spk_task copied it. */
    if (s_uac_device == NULL || s_uac_device->spk_queue == NULL ||
        n_bytes_received == 0 || n_bytes_received > UAC_SPK_PACKET_BYTES) {
        return true;
    }
    s_spk_usb_packets++;
    s_spk_usb_bytes += n_bytes_received;

    uac_spk_packet_t packet;
    uint16_t bytes_read = tud_audio_n_read(func_id, packet.data, n_bytes_received);
    if (bytes_read == 0 || bytes_read > UAC_SPK_PACKET_BYTES) {
        return true;
    }
    s_spk_read_bytes += bytes_read;

    packet.len = bytes_read;
    BaseType_t task_woken = pdFALSE;
    if (xQueueSendFromISR(s_uac_device->spk_queue, &packet, &task_woken) != pdTRUE) {
        s_spk_queue_drops++;
    } else {
        uint32_t waiting = (uint32_t)uxQueueMessagesWaitingFromISR(s_uac_device->spk_queue);
        if (waiting > s_spk_queue_highwater) s_spk_queue_highwater = waiting;
    }
    if (task_woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
    return true;
}

bool tud_audio_tx_done_isr(uint8_t rhport, uint16_t n_bytes_sent, uint8_t func_id, uint8_t ep_in, uint8_t cur_alt_setting)
{
    (void)rhport;
    (void)ep_in;
    (void)cur_alt_setting;
    s_mic_tx_callbacks++;
    /* Wire-level accounting: n_bytes_sent is what the ISO IN endpoint actually
     * transmitted this microframe. ALSA writes exactly these bytes into the
     * capture file at the declared sample rate, so any shortfall here is a
     * permanent pitch-up in the recording. Measure it explicitly. */
    if (s_uac_device->mic_active) {
        s_mic_wire_bytes += n_bytes_sent;
        s_mic_wire_packets++;
        uint16_t nominal_packet = (uint16_t)(s_uac_device->mic_bytes_per_ms / 8);
        if (n_bytes_sent > 0 && n_bytes_sent < nominal_packet) {
            s_mic_wire_shorts++;
        }
    } else {
        s_mic_wire_packets++;
    }
    size_t bytes_require = MIC_INTERVAL_MS * s_uac_device->mic_bytes_per_ms;

    tu_fifo_t *sw_in_fifo = tud_audio_n_get_ep_in_ff(func_id);
    uint16_t fifo_remained = tu_fifo_remaining(sw_in_fifo);
    s_mic_tx_fifo_depth = tu_fifo_depth(sw_in_fifo);
    s_mic_tx_fifo_free = fifo_remained;
    s_mic_tx_bytes_required = (uint16_t)bytes_require;

    UAC_ENTER_CRITICAL();

    /* Pick up the next queued chunk when the previous one is finished. */
    if (s_uac_device->mic_data_size == 0) {
        for (int i = 1; i <= UAC_MIC_SLOTS; i++) {
            uint8_t slot = (uint8_t)((s_uac_device->mic_rd_slot + i) % UAC_MIC_SLOTS);
            if (s_uac_device->mic_slot_ready[slot]) {
                s_uac_device->mic_rd_slot = slot;
                s_uac_device->mic_send_offset = 0;
                s_uac_device->mic_data_size = s_uac_device->mic_slot_len[slot];
                break;
            }
        }
    }

    /* Push as much as the FIFO accepts. Full chunks fit in the steady state;
     * a short write just continues on the next transfer. */
    if (s_uac_device->mic_data_size > 0 && fifo_remained > 0) {
        uint16_t to_send = (uint16_t)s_uac_device->mic_data_size;
        if (to_send > fifo_remained) {
            to_send = fifo_remained;
        }
        uint16_t written = tud_audio_n_write(
            func_id,
            (void *)&s_uac_device->mic_chunk[s_uac_device->mic_rd_slot][s_uac_device->mic_send_offset],
            to_send);
        if (written > 0) {
            s_mic_tx_write_calls++;
            s_mic_tx_write_bytes += written;
            if (written < to_send) {
                s_mic_tx_partial_writes++;
            }
            s_uac_device->mic_send_offset += written;
            s_uac_device->mic_data_size -= written;
            if (s_uac_device->mic_data_size <= 0) {
                s_uac_device->mic_data_size = 0;
                s_uac_device->mic_send_offset = 0;
                s_uac_device->mic_slot_ready[s_uac_device->mic_rd_slot] = 0;
            }
        } else {
            /* No progress this transfer (FIFO not ready); retry next tick. */
            s_mic_tx_fifo_waits++;
        }
    } else if (s_uac_device->mic_data_size > 0) {
        s_mic_tx_fifo_waits++;
    }

    UAC_EXIT_CRITICAL();

    return true;
}

#if SPEAK_CHANNEL_NUM
static void usb_spk_task(void *pvParam)
{
    (void)pvParam;
    uac_spk_packet_t packet;
    int64_t report_at = esp_timer_get_time();
    uint32_t prev_usb_packets = 0;
    uint32_t prev_usb_bytes = 0;
    uint32_t prev_read_bytes = 0;
    uint32_t prev_task_packets = 0;

    while (1) {
        if (xQueueReceive(s_uac_device->spk_queue, &packet, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        if (!s_uac_device->spk_active || packet.len == 0) {
            continue;
        }
        if (s_uac_device->user_cfg.output_cb) {
            s_uac_device->user_cfg.output_cb(packet.data, packet.len, s_uac_device->user_cfg.cb_ctx);
            s_spk_task_packets++;
        }

        int64_t now = esp_timer_get_time();
        if (now - report_at >= 2000000) {
            uint32_t usb_packets = s_spk_usb_packets;
            uint32_t usb_bytes = s_spk_usb_bytes;
            uint32_t read_bytes = s_spk_read_bytes;
            uint32_t task_packets = s_spk_task_packets;
            uint32_t queue_drops = s_spk_queue_drops;
            uint32_t dt_ms = (uint32_t)((now - report_at) / 1000);
            ESP_LOGI(TAG,
                     "speaker path: USB=%" PRIu32 " packets/s, %" PRIu32
                     " B/s; FIFO read=%" PRIu32 " B/s; task=%" PRIu32
                     " packets/s; queue high-water=%" PRIu32 "/%u, drops=%" PRIu32,
                     (uint32_t)((uint64_t)(usb_packets - prev_usb_packets) * 1000 / dt_ms),
                     (uint32_t)((uint64_t)(usb_bytes - prev_usb_bytes) * 1000 / dt_ms),
                     (uint32_t)((uint64_t)(read_bytes - prev_read_bytes) * 1000 / dt_ms),
                     (uint32_t)((uint64_t)(task_packets - prev_task_packets) * 1000 / dt_ms),
                     s_spk_queue_highwater, UAC_SPK_QUEUE_DEPTH, queue_drops);
            s_spk_stats_snapshot.usb_packets_per_sec =
                (uint32_t)((uint64_t)(usb_packets - prev_usb_packets) * 1000 / dt_ms);
            s_spk_stats_snapshot.usb_bytes_per_sec =
                (uint32_t)((uint64_t)(usb_bytes - prev_usb_bytes) * 1000 / dt_ms);
            s_spk_stats_snapshot.fifo_bytes_per_sec =
                (uint32_t)((uint64_t)(read_bytes - prev_read_bytes) * 1000 / dt_ms);
            s_spk_stats_snapshot.task_packets_per_sec =
                (uint32_t)((uint64_t)(task_packets - prev_task_packets) * 1000 / dt_ms);
            s_spk_stats_snapshot.queue_high_water = s_spk_queue_highwater;
            s_spk_stats_snapshot.queue_depth = UAC_SPK_QUEUE_DEPTH;
            s_spk_stats_snapshot.drops = queue_drops;
            report_at = now;
            prev_usb_packets = usb_packets;
            prev_usb_bytes = usb_bytes;
            prev_read_bytes = read_bytes;
            prev_task_packets = task_packets;
            s_spk_queue_highwater = 0;
        }
    }
}
#endif

#if MIC_CHANNEL_NUM
static void usb_mic_task(void *pvParam)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    int64_t report_at = esp_timer_get_time();
    uint32_t prev_generated_bytes = 0;
    uint32_t prev_write_calls = 0;
    uint32_t prev_write_bytes = 0;
    uint32_t prev_partial_writes = 0;
    uint32_t prev_tx_callbacks = 0;
    uint32_t prev_fifo_waits = 0;
    uint32_t prev_skips = 0;
    uint32_t prev_wire_bytes = 0;
    uint32_t prev_wire_packets = 0;
    uint32_t prev_wire_shorts = 0;
    while (1) {
        if (s_uac_device->mic_active == false) {
            // clear the notification
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            xLastWakeTime = xTaskGetTickCount();
            continue;
        }
        // clear the notification
        // read data from the microphone chunk by chunk
        size_t bytes_require = MIC_INTERVAL_MS * s_uac_device->mic_bytes_per_ms;
        /* Only generate into a chunk slot that the USB hand-over has already
         * released. The ring has UAC_MIC_SLOTS entries, so a hand-over that is
         * late by a tick or two only adds latency; the ADC ring buffers the
         * extra input and no audio is dropped (which used to skip content and
         * make recordings play fast/pitched up). */
        bool slot_free;
        UAC_ENTER_CRITICAL();
        slot_free = (s_uac_device->mic_slot_ready[s_uac_device->mic_wr_slot] == 0);
        UAC_EXIT_CRITICAL();
        if (!slot_free) {
            s_mic_task_skips++;
            vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(MIC_INTERVAL_MS));
            continue;
        }
        if (s_uac_device->user_cfg.input_cb) {
            size_t bytes_read = 0;
            esp_err_t ret = s_uac_device->user_cfg.input_cb(
                (uint8_t *)s_uac_device->mic_chunk[s_uac_device->mic_wr_slot],
                bytes_require, &bytes_read, s_uac_device->user_cfg.cb_ctx);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to read data from mic");
                continue;
            }
            if (bytes_read == 0) {
                vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(MIC_INTERVAL_MS));
                continue;
            }
            s_mic_generated_bytes += bytes_read;
            UAC_ENTER_CRITICAL();
            s_uac_device->mic_slot_len[s_uac_device->mic_wr_slot] = (uint16_t)bytes_read;
            s_uac_device->mic_slot_ready[s_uac_device->mic_wr_slot] = 1;
            s_uac_device->mic_wr_slot = (uint8_t)((s_uac_device->mic_wr_slot + 1) % UAC_MIC_SLOTS);
            UAC_EXIT_CRITICAL();
        }

        int64_t now = esp_timer_get_time();
        if (now - report_at >= 2000000) {
            uint32_t generated_bytes = s_mic_generated_bytes;
            uint32_t write_calls = s_mic_tx_write_calls;
            uint32_t write_bytes = s_mic_tx_write_bytes;
            uint32_t partial_writes = s_mic_tx_partial_writes;
            uint32_t tx_callbacks = s_mic_tx_callbacks;
            uint32_t fifo_waits = s_mic_tx_fifo_waits;
            uint32_t skips = s_mic_task_skips;
            uint32_t wire_bytes = s_mic_wire_bytes;
            uint32_t wire_packets = s_mic_wire_packets;
            uint32_t wire_shorts = s_mic_wire_shorts;
            uint16_t fifo_depth = s_mic_tx_fifo_depth;
            uint16_t fifo_free = s_mic_tx_fifo_free;
            uint16_t bytes_required = s_mic_tx_bytes_required;
            uint32_t dt_ms = (uint32_t)((now - report_at) / 1000);
            /*
             * At high speed every ISO IN endpoint must produce a packet in
             * each 125 us microframe (8000/s). When the device misses one
             * (USB task late past the deadline), the host records a
             * zero-length packet: current_sample_rate/8000 frames are gone
             * from the capture timeline for good, so recordings play fast
             * and end up pitched up. Measure the deficit explicitly so the
             * fix can be verified from the log.
             */
            int64_t cb_delta = (int64_t)tx_callbacks - (int64_t)prev_tx_callbacks;
            int64_t microframes_expected = (int64_t)dt_ms * 8000 / 1000;
            int64_t microframes_missed = microframes_expected - cb_delta;
            int64_t frames_lost_per_s = (microframes_missed > 0)
                ? (microframes_missed * (int64_t)s_uac_device->current_sample_rate / 8000 *
                   1000 / (int64_t)dt_ms)
                : 0;
            int64_t ppm_lost = (int64_t)s_uac_device->current_sample_rate
                ? (frames_lost_per_s * 1000000 / (int64_t)s_uac_device->current_sample_rate)
                : 0;
            /* Pitch truth from the wire itself: ALSA records exactly these
             * bytes at the declared rate, so wire B/s vs nominal B/s is the
             * recorded pitch error. Nominal = bytes_per_ms x 1000 (e.g.
             * 1536 x 1000 = 1,536,000 B/s at 96 kHz, 4 ch, 4 B/sample). */
            uint64_t wire_nominal = (uint64_t)s_uac_device->mic_bytes_per_ms * 1000u;
            int64_t wire_bps = (int64_t)(wire_bytes - prev_wire_bytes) * 1000
                / (int64_t)dt_ms;
            int64_t wire_deficit_ppm = wire_nominal
                ? (wire_bps * 1000000) / (int64_t)wire_nominal - 1000000
                : 0;
            ESP_LOGI(TAG,
                     "mic path: generated=%" PRIu32 " B/s; USB FIFO writes=%" PRIu32
                     " chunks/s, %" PRIu32 " B/s; partial=%" PRIu32
                     "; TX callbacks=%" PRIu32 "/s, FIFO waits=%" PRIu32
                     "/s, skips=%" PRIu32 "/s, FIFO free/depth=%u/%u, need=%u"
                     ", wire=%" PRIu32 " B/s (%" PRIu32 " pkts/s, shorts=%" PRIu32
                     ", %+ld ppm)"
                     "; missed microframes=%" PRId64 "/s (%" PRId64 " frames/s, %" PRId64 " ppm loss)",
                     (uint32_t)((uint64_t)(generated_bytes - prev_generated_bytes) * 1000 / dt_ms),
                     (uint32_t)((uint64_t)(write_calls - prev_write_calls) * 1000 / dt_ms),
                     (uint32_t)((uint64_t)(write_bytes - prev_write_bytes) * 1000 / dt_ms),
                     partial_writes - prev_partial_writes,
                     (uint32_t)((uint64_t)(tx_callbacks - prev_tx_callbacks) * 1000 / dt_ms),
                     (uint32_t)((uint64_t)(fifo_waits - prev_fifo_waits) * 1000 / dt_ms),
                     (uint32_t)((uint64_t)(skips - prev_skips) * 1000 / dt_ms),
                     fifo_free, fifo_depth, bytes_required,
                     (uint32_t)(((uint64_t)(wire_bytes - prev_wire_bytes) * 1000) / dt_ms),
                     (uint32_t)(((uint64_t)(wire_packets - prev_wire_packets) * 1000) / dt_ms),
                     wire_shorts - prev_wire_shorts,
                     (long)wire_deficit_ppm,
                     microframes_missed > 0 ? microframes_missed : 0,
                     frames_lost_per_s, ppm_lost);
            report_at = now;
            prev_generated_bytes = generated_bytes;
            prev_write_calls = write_calls;
            prev_write_bytes = write_bytes;
            prev_partial_writes = partial_writes;
            prev_tx_callbacks = tx_callbacks;
            prev_fifo_waits = fifo_waits;
            prev_skips = skips;
            prev_wire_bytes = wire_bytes;
            prev_wire_packets = wire_packets;
            prev_wire_shorts = wire_shorts;
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(MIC_INTERVAL_MS));
    }
}
#endif

esp_err_t uac_device_init(uac_device_config_t *config)
{
    ESP_RETURN_ON_FALSE(config != NULL, ESP_ERR_INVALID_ARG, TAG, "config is NULL");
    if (s_uac_device != NULL) {
        ESP_LOGW(TAG, "uac device already initialized");
        return ESP_OK;
    }
    s_uac_device = calloc(1, sizeof(uac_device_t));
    ESP_RETURN_ON_FALSE(s_uac_device != NULL, ESP_ERR_NO_MEM, TAG, "Failed to allocate memory for uac device");
    s_uac_device->user_cfg.output_cb = config->output_cb;
    s_uac_device->user_cfg.input_cb = config->input_cb;
    s_uac_device->user_cfg.cb_ctx = config->cb_ctx;
    s_uac_device->user_cfg.set_mute_cb = config->set_mute_cb;
    s_uac_device->user_cfg.set_volume_cb = config->set_volume_cb;
    s_uac_device->current_sample_rate = DEFAULT_SAMPLE_RATE;
    s_uac_device->mic_wr_slot = 0;
    s_uac_device->mic_rd_slot = 0;
    s_uac_device->mic_send_offset = 0;
    s_uac_device->mic_data_size = 0;

#if SPEAK_CHANNEL_NUM
    s_uac_device->spk_queue = xQueueCreate(UAC_SPK_QUEUE_DEPTH, sizeof(uac_spk_packet_t));
    ESP_RETURN_ON_FALSE(s_uac_device->spk_queue != NULL, ESP_ERR_NO_MEM, TAG,
                        "Failed to create speaker packet queue");
#endif

#if CONFIG_USB_DEVICE_UAC_AS_PART
    s_uac_device->spk_itf_num = config->spk_itf_num;
    s_uac_device->mic_itf_num = config->mic_itf_num;
#else
#if SPEAK_CHANNEL_NUM
    s_uac_device->spk_itf_num = ITF_NUM_AUDIO_STREAMING_SPK;
#endif
#if MIC_CHANNEL_NUM
    s_uac_device->mic_itf_num = ITF_NUM_AUDIO_STREAMING_MIC;
#endif
#endif

    BaseType_t ret_val;
    if (!config->skip_tinyusb_init) {
        usb_phy_init();
        bool usb_init = tusb_init();
        if (!usb_init) {
            ESP_LOGE(TAG, "USB Device Stack Init Fail");
            return ESP_FAIL;
        }
        ret_val = xTaskCreatePinnedToCore(tusb_device_task, "TinyUSB", 4096, NULL, CONFIG_UAC_TINYUSB_TASK_PRIORITY,
                                          NULL, CONFIG_UAC_TINYUSB_TASK_CORE == -1 ? tskNO_AFFINITY : CONFIG_UAC_TINYUSB_TASK_CORE);
        ESP_RETURN_ON_FALSE(ret_val == pdPASS, ESP_FAIL, TAG, "Failed to create TinyUSB task");
    }

#if MIC_CHANNEL_NUM
    ret_val = xTaskCreatePinnedToCore(usb_mic_task, "usb_mic_task", 4096, NULL, CONFIG_UAC_MIC_TASK_PRIORITY,
                                      &s_uac_device->mic_task_handle, CONFIG_UAC_MIC_TASK_CORE == -1 ? tskNO_AFFINITY : CONFIG_UAC_MIC_TASK_CORE);
    ESP_RETURN_ON_FALSE(ret_val == pdPASS, ESP_FAIL, TAG, "Failed to create usb_mic task");
#endif

#if SPEAK_CHANNEL_NUM
    ret_val = xTaskCreatePinnedToCore(usb_spk_task, "usb_spk_task", 4096, NULL, CONFIG_UAC_SPK_TASK_PRIORITY,
                                      &s_uac_device->spk_task_handle, CONFIG_UAC_SPK_TASK_CORE == -1 ? tskNO_AFFINITY : CONFIG_UAC_SPK_TASK_CORE);
    ESP_RETURN_ON_FALSE(ret_val == pdPASS, ESP_FAIL, TAG, "Failed to create usb_spk task");
#endif

    ESP_LOGI(TAG, "UAC Device Start, Version: %d.%d.%d", USB_DEVICE_UAC_VER_MAJOR, USB_DEVICE_UAC_VER_MINOR, USB_DEVICE_UAC_VER_PATCH);
    return ESP_OK;
}
