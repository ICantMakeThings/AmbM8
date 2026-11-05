/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 * Copyright (c) 2020 Jerzy Kasenbreg
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include "tusb.h"
#include "class/msc/msc.h"
#include "class/hid/hid_device.h"
#include "uac_descriptors.h"

// Set by the application before TinyUSB is initialized. The device enumerates
// as one USB personality per boot, so hosts see a clean disconnect/reconnect.
extern bool app_is_sd_mode(void);

//--------------------------------------------------------------------+
// Device Descriptors
//--------------------------------------------------------------------+
tusb_desc_device_t const desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,

    // Use Interface Association Descriptor (IAD) for CDC
    // As required by USB Specs IAD's subclass must be common class (2) and protocol must be IAD (1)
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor           = CONFIG_UAC_TUSB_VID,
    .idProduct          = CONFIG_UAC_TUSB_PID,
    .bcdDevice          = 0x0100,

    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,

    .bNumConfigurations = 0x01
};

// Invoked when received GET DEVICE DESCRIPTOR
// Application return pointer to descriptor
uint8_t const *tud_descriptor_device_cb(void)
{
    if (app_is_sd_mode()) {
        static const tusb_desc_device_t msc_device = {
            .bLength = sizeof(tusb_desc_device_t),
            .bDescriptorType = TUSB_DESC_DEVICE,
            .bcdUSB = 0x0200,
            .bDeviceClass = 0,
            .bDeviceSubClass = 0,
            .bDeviceProtocol = 0,
            .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
            .idVendor = CONFIG_UAC_TUSB_VID,
            // A distinct PID keeps host driver/interface caches separate from UAC.
            .idProduct = CONFIG_UAC_TUSB_PID ^ 0x0001,
            .bcdDevice = 0x0100,
            .iManufacturer = 0x01,
            .iProduct = 0x02,
            .iSerialNumber = 0x03,
            .bNumConfigurations = 1,
        };
        return (const uint8_t *)&msc_device;
    }
    return (uint8_t const *)&desc_device;
}

//--------------------------------------------------------------------+
// Configuration Descriptor
//--------------------------------------------------------------------+
#define CONFIG_TOTAL_LEN        (TUD_CONFIG_DESC_LEN + CFG_TUD_AUDIO * TUD_AUDIO_DEVICE_DESC_LEN + CFG_TUD_HID * TUD_HID_DESC_LEN)
#define EPNUM_AUDIO_OUT   0x01
#define EPNUM_AUDIO_FB    0x81
#define EPNUM_AUDIO_IN    0x82
#define EPNUM_HID_IN      0x83

// Consumer Control report: single 16-bit usage-code field. Hosts map this to
// global media keys (Play/Pause, Next Track, Previous Track, ...).
static const uint8_t desc_hid_report[] = {
    TUD_HID_REPORT_DESC_CONSUMER()
};

uint8_t const desc_configuration[] = {
    // Config number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),
    // Interface number, string index, EP Out & EP In address, EP size
    TUD_AUDIO_DESCRIPTOR(ITF_NUM_AUDIO_CONTROL, 4, EPNUM_AUDIO_OUT, EPNUM_AUDIO_IN, EPNUM_AUDIO_FB),
#if CFG_TUD_HID
    // Interface number, string index, protocol, report descriptor len, EP In address, EP size, polling interval
    TUD_HID_DESCRIPTOR(ITF_NUM_HID, 7, HID_ITF_PROTOCOL_NONE, sizeof(desc_hid_report), EPNUM_HID_IN, CFG_TUD_HID_EP_BUFSIZE, 1),
#endif
};

#define MSC_CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_MSC_DESC_LEN)
static const uint8_t desc_msc_configuration[] = {
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, MSC_CONFIG_TOTAL_LEN, 0x00, 100),
    TUD_MSC_DESCRIPTOR(0, 4, 0x01, 0x81, TUD_OPT_HIGH_SPEED ? 512 : 64),
};

// Invoked when received GET CONFIGURATION DESCRIPTOR
// Application return pointer to descriptor
// Descriptor contents must exist long enough for transfer to complete
uint8_t const *tud_descriptor_configuration_cb(uint8_t index)
{
    (void)index; // for multiple configurations
    if (app_is_sd_mode()) return desc_msc_configuration;
    return desc_configuration;
}

//--------------------------------------------------------------------+
// String Descriptors
//--------------------------------------------------------------------+

// array of pointer to string descriptors
char const *string_desc_arr [] = {
    (const char[]) { 0x09, 0x04 },  // 0: is supported language is English (0x0409)
    CONFIG_UAC_TUSB_MANUFACTURER,       // 1: Manufacturer
    CONFIG_UAC_TUSB_PRODUCT,            // 2: Product
    CONFIG_UAC_TUSB_SERIAL_NUM,         // 3: Serials, should use chip ID
    "usb uac",                      // 4: UAC control Interface
#if SPEAK_CHANNEL_NUM
    "speaker",                     // 5: Speak Interface
#endif
#if MIC_CHANNEL_NUM
    "microphone",                   // 6: Mic Interface
#endif
#if CFG_TUD_HID
    "media keys",                   // 7: Consumer Control Interface
#endif
};

static uint16_t _desc_str[32];

// Invoked when received GET STRING DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    (void)langid;

    uint8_t chr_count;

    if (index == 0) {
        memcpy(&_desc_str[1], string_desc_arr[0], 2);
        chr_count = 1;
    } else {

        if (!(index < sizeof(string_desc_arr) / sizeof(string_desc_arr[0]))) {
            return NULL;
        }

        const char *str = string_desc_arr[index];
        if (index == 2 && app_is_sd_mode()) str = "AMB4 FS";
        if (index == 4 && app_is_sd_mode()) str = "AMB4 FS";

        // Cap at max char
        chr_count = (uint8_t) strlen(str);
        if (chr_count > 31) {
            chr_count = 31;
        }

        for (uint8_t i = 0; i < chr_count; i++) {
            _desc_str[1 + i] = str[i];
        }
    }

    // first byte is length (including header), second byte is string type
    _desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));

    return _desc_str;
}

//--------------------------------------------------------------------+
// HID Consumer Control (media keys)
//--------------------------------------------------------------------+

// Last usage code sent. 0 = no key pressed. Reports are 2-byte usage-index
// values matching TUD_HID_REPORT_DESC_CONSUMER().
static uint16_t s_hid_last_usage;

// Send a consumer-control usage to the host (usage codes are the standard
// HID Consumer Page values, e.g. 0xCD = Play/Pause, 0xB5 = Next, 0xB6 = Prev).
bool usb_desc_hid_send_usage(uint16_t usage)
{
    if (!tud_ready() || !tud_hid_ready()) {
        return false;
    }
    s_hid_last_usage = usage;
    return tud_hid_report(0, &s_hid_last_usage, sizeof(s_hid_last_usage));
}

// Invoked when received GET_REPORT control request (application must fill buffer)
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type,
                               uint8_t *buffer, uint16_t reqlen)
{
    (void)instance; (void)report_id; (void)report_type; (void)buffer; (void)reqlen;
    return 0;
}

// Invoked when the host issues GET_DESCRIPTOR (HID Report): return the
// Consumer Control report map above.
uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance)
{
    (void)instance;
    return desc_hid_report;
}

// Invoked when received SET_REPORT control request
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type,
                           uint8_t const *buffer, uint16_t bufsize)
{
    (void)instance; (void)report_id; (void)report_type; (void)buffer; (void)bufsize;
}
