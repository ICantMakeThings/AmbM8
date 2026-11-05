/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "uac_descriptors.h"

//------------- CLASS -------------//
// The number of audio interfaces
#define CFG_TUD_AUDIO             1
// The application selects either UAC2 or MSC descriptors at boot.
#define CFG_TUD_MSC               1
#define CFG_TUD_MSC_EP_BUFSIZE    4096

//--------------------------------------------------------------------
// AUDIO CLASS DRIVER CONFIGURATION
//--------------------------------------------------------------------

// Enable feedback EP
#define CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP                             1

// Enable/disable conversion from 16.16 to 10.14 format on full-speed devices
#if CONFIG_UAC_SUPPORT_MACOS
#define CFG_TUD_AUDIO_ENABLE_FEEDBACK_FORMAT_CORRECTION              1
#endif

#define CFG_TUD_AUDIO_FUNC_1_DESC_LEN                                TUD_AUDIO_DEVICE_DESC_LEN

// How many formats are used, need to adjust USB descriptor if changed
#define CFG_TUD_AUDIO_FUNC_1_N_FORMATS                               1

#define CFG_TUD_AUDIO_FUNC_1_MAX_SAMPLE_RATE                         DEFAULT_SAMPLE_RATE
#define CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX                           MIC_CHANNEL_NUM
#define CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX                           SPEAK_CHANNEL_NUM

// Bit resolution and slot size selected via Kconfig (UAC_BIT_RESOLUTION).
// 16-bit -> 2 bytes/sample, 24-bit and 32-bit -> 4 bytes/sample (24-in-32 / 32-in-32).
#define CFG_TUD_AUDIO_FUNC_1_FORMAT_1_N_BYTES_PER_SAMPLE_TX          CONFIG_UAC_BYTES_PER_SAMPLE
#define CFG_TUD_AUDIO_FUNC_1_FORMAT_1_RESOLUTION_TX                  CONFIG_UAC_BIT_RESOLUTION
#define CFG_TUD_AUDIO_FUNC_1_FORMAT_1_N_BYTES_PER_SAMPLE_RX          CONFIG_UAC_BYTES_PER_SAMPLE
#define CFG_TUD_AUDIO_FUNC_1_FORMAT_1_RESOLUTION_RX                  CONFIG_UAC_BIT_RESOLUTION
#define CFG_TUD_AUDIO_FUNC_1_FORMAT_1_FRAME_SZ_TX                    (CFG_TUD_AUDIO_FUNC_1_FORMAT_1_N_BYTES_PER_SAMPLE_TX * CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX)
#define CFG_TUD_AUDIO_FUNC_1_FORMAT_1_FRAME_SZ_RX                    (CFG_TUD_AUDIO_FUNC_1_FORMAT_1_N_BYTES_PER_SAMPLE_RX * CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX)

// EP and buffer size - for isochronous EP´s, the buffer and EP size are equal (different sizes would not make sense)
#define CFG_TUD_AUDIO_ENABLE_EP_IN                1

// Isochronous endpoint interval is 125 us at high speed and 1 ms at full speed.
// Size for the maximum samples in one service interval, plus one audio frame
// of clock-drift headroom. A per-millisecond size is invalid for HS endpoints
// at high rates and can exceed USB's 1024-byte transaction limit.
#if defined(CONFIG_TINYUSB_RHPORT_HS) && CONFIG_TINYUSB_RHPORT_HS
#define UAC_USB_INTERVALS_PER_MS 8
#define UAC_USB_RATE_DIVISOR     8000
#define UAC_USB_RATE_ROUND       7999
#else
#define UAC_USB_INTERVALS_PER_MS 1
#define UAC_USB_RATE_DIVISOR     1000
#define UAC_USB_RATE_ROUND       999
#endif

#define CFG_TUD_AUDIO_FUNC_1_FORMAT_1_EP_SZ_IN \
    ((((CFG_TUD_AUDIO_FUNC_1_MAX_SAMPLE_RATE + UAC_USB_RATE_ROUND) / UAC_USB_RATE_DIVISOR) + 1) * \
     CFG_TUD_AUDIO_FUNC_1_FORMAT_1_FRAME_SZ_TX)

// EP-IN FIFO: the refill gate waits for MIC_INTERVAL_MS of free space, so the
// FIFO must hold at least two such chunks. Scale packet count by 8 for HS,
// where each millisecond contains eight isochronous service intervals. TinyUSB
// tu_fifo_config() rejects depths above 0x8000 bytes; cap the FIFO on whole
// endpoint packets so that configuration cannot silently leave it at depth 0.
#if (2 * MIC_INTERVAL_MS + 2) > 4
#define UAC_MIC_FIFO_PACKETS_RAW  ((2 * MIC_INTERVAL_MS + 2) * UAC_USB_INTERVALS_PER_MS)
#else
#define UAC_MIC_FIFO_PACKETS_RAW  (4 * UAC_USB_INTERVALS_PER_MS)
#endif
#define UAC_MIC_FIFO_MAX_PACKETS  (0x8000 / CFG_TUD_AUDIO_FUNC_1_FORMAT_1_EP_SZ_IN)
#if UAC_MIC_FIFO_PACKETS_RAW > UAC_MIC_FIFO_MAX_PACKETS
#define UAC_MIC_FIFO_PACKETS      UAC_MIC_FIFO_MAX_PACKETS
#else
#define UAC_MIC_FIFO_PACKETS      UAC_MIC_FIFO_PACKETS_RAW
#endif
#define CFG_TUD_AUDIO_FUNC_1_EP_IN_SW_BUF_SZ      CFG_TUD_AUDIO_FUNC_1_FORMAT_1_EP_SZ_IN * UAC_MIC_FIFO_PACKETS
#define CFG_TUD_AUDIO_FUNC_1_EP_IN_SZ_MAX         CFG_TUD_AUDIO_FUNC_1_FORMAT_1_EP_SZ_IN  // Maximum EP IN size for all AS alternate settings used

#if CFG_TUD_AUDIO_FUNC_1_EP_IN_SW_BUF_SZ < (2 * MIC_INTERVAL_MS * CFG_TUD_AUDIO_FUNC_1_FORMAT_1_N_BYTES_PER_SAMPLE_TX * CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX)
#error "Mic EP-IN FIFO is too small for the configured microphone refill interval"
#endif

// EP and buffer size - for isochronous EP´s, the buffer and EP size are equal (different sizes would not make sense)
#define CFG_TUD_AUDIO_ENABLE_EP_OUT               1

// SPK OUT EP (host-to-device = RX): keep EP size aligned to one complete audio frame.
#define CFG_TUD_AUDIO_FUNC_1_FORMAT_1_EP_SZ_OUT \
    ((((CFG_TUD_AUDIO_FUNC_1_MAX_SAMPLE_RATE + UAC_USB_RATE_ROUND) / UAC_USB_RATE_DIVISOR) + 1) * \
     CFG_TUD_AUDIO_FUNC_1_FORMAT_1_FRAME_SZ_RX)

#define CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ     CFG_TUD_AUDIO_FUNC_1_FORMAT_1_EP_SZ_OUT * (SPK_INTERVAL_MS + 1)
#define CFG_TUD_AUDIO_FUNC_1_EP_OUT_SZ_MAX        CFG_TUD_AUDIO_FUNC_1_FORMAT_1_EP_SZ_OUT // Maximum EP OUT size for all AS alternate settings used

// Number of Standard AS Interface Descriptors (4.9.1) defined per audio function - this is required to be able to remember the current alternate settings of these interfaces - We restrict us here to have a constant number for all audio functions (which means this has to be the maximum number of AS interfaces an audio function has and a second audio function with less AS interfaces just wastes a few bytes)
#define CFG_TUD_AUDIO_FUNC_1_N_AS_INT             1

// Size of control request buffer
#define CFG_TUD_AUDIO_FUNC_1_CTRL_BUF_SZ    64

#ifdef __cplusplus
}
#endif
