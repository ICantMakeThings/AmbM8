// SD storage

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "driver/sdmmc_host.h"
#include "driver/sdmmc_default_configs.h"
#include "driver/gpio.h"
#include "led_strip.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_ldo_regulator.h"
#include "esp_private/usb_phy.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdmmc_cmd.h"
#include "tusb.h"

#include "SD.h"
#include "Display.h"

#define SD_SECTOR_SIZE 512
#define SD_BATCH_SECTORS 8

static const char *TAG = "sd_storage";
static sdmmc_card_t s_card;
static esp_ldo_channel_handle_t s_sd_ldo;
static SemaphoreHandle_t s_card_mutex;
static bool s_card_ready;
static volatile bool s_host_ejected;
static bool s_storage_usb_active;
static usb_phy_handle_t s_usb_phy;
static led_strip_handle_t s_status_led;
static volatile uint32_t s_read_count;
static volatile uint32_t s_write_count;
static volatile bool s_eject_warning;
static uint32_t s_prev_display_reads, s_prev_display_writes;
static DMA_ATTR uint8_t s_sector_buffer[SD_SECTOR_SIZE];
static DMA_ATTR uint8_t s_multi_sector_buffer[SD_SECTOR_SIZE * SD_BATCH_SECTORS];

// LED task
static void sd_status_led_task(void *arg)
{
    (void)arg;
    uint32_t prev_reads = s_read_count;
    uint32_t prev_writes = s_write_count;
    int activity_ticks = 0;
    bool activity_write = false;
    bool blink_on = false;
    bool warning_blink_on = false;

    while (true) {
        if (s_eject_warning) {
            warning_blink_on = !warning_blink_on;
            led_strip_set_pixel(s_status_led, 0,
                                warning_blink_on ? 100 : 0,
                                warning_blink_on ? 35 : 0, 0);
            led_strip_refresh(s_status_led);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        uint32_t reads = s_read_count;
        uint32_t writes = s_write_count;
        if (reads != s_prev_display_reads || writes != s_prev_display_writes) {
            display_set_io(writes != s_prev_display_writes ? DISPLAY_IO_WRITING : DISPLAY_IO_READING);
            s_prev_display_reads = reads; s_prev_display_writes = writes;
        } else if (activity_ticks == 0) {
            display_set_io(s_host_ejected ? DISPLAY_IO_UNMOUNTED : DISPLAY_IO_NONE);
        }
        if (reads != prev_reads || writes != prev_writes) {
            activity_write = writes != prev_writes;
            activity_ticks = 8;
            prev_reads = reads;
            prev_writes = writes;
        }

        if (activity_ticks > 0) {
            blink_on = !blink_on;
            if (activity_write) {
                led_strip_set_pixel(s_status_led, 0, blink_on ? 80 : 0, 0, 0);
            } else {
                led_strip_set_pixel(s_status_led, 0, 0, 0, blink_on ? 100 : 0);
            }
            activity_ticks--;
        } else {

            led_strip_set_pixel(s_status_led, 0, 0x00, 0x49, 0x9C);
        }
        led_strip_refresh(s_status_led);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static esp_err_t sd_status_led_init(void)
{
    const led_strip_config_t strip_config = {
        .strip_gpio_num = GPIO_NUM_51,
        .max_leds = 1,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags.invert_out = false,
    };
    const led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
        .flags.with_dma = false,
    };
    return led_strip_new_rmt_device(&strip_config, &rmt_config, &s_status_led);
}

// Sector R+W thru SDMMC driver
static esp_err_t card_read(uint32_t lba, uint32_t offset, void *buffer, uint32_t length)
{
    uint8_t *dst = buffer;
    if (!s_card_ready || (uint64_t)lba * SD_SECTOR_SIZE + offset + length >
                         (uint64_t)s_card.csd.capacity * s_card.csd.sector_size) return ESP_ERR_INVALID_STATE;
    xSemaphoreTake(s_card_mutex, portMAX_DELAY);
    while (length) {
        uint32_t sector_lba = lba + offset / SD_SECTOR_SIZE;
        uint32_t sector_offset = offset % SD_SECTOR_SIZE;
        if (sector_offset == 0 && length >= SD_SECTOR_SIZE) {
            uint32_t sectors = length / SD_SECTOR_SIZE;
            if (sectors > SD_BATCH_SECTORS) sectors = SD_BATCH_SECTORS;
            uint32_t chunk = sectors * SD_SECTOR_SIZE;
            esp_err_t err = sdmmc_read_sectors(&s_card, s_multi_sector_buffer,
                                                sector_lba, sectors);
            if (err != ESP_OK) { xSemaphoreGive(s_card_mutex); return err; }
            memcpy(dst, s_multi_sector_buffer, chunk);
            dst += chunk; length -= chunk; offset += chunk;
            continue;
        }
        uint32_t chunk = SD_SECTOR_SIZE - sector_offset;
        if (chunk > length) chunk = length;
        esp_err_t err = sdmmc_read_sectors(&s_card, s_sector_buffer, sector_lba, 1);
        if (err != ESP_OK) { xSemaphoreGive(s_card_mutex); return err; }
        memcpy(dst, s_sector_buffer + sector_offset, chunk);
        dst += chunk; length -= chunk; offset += chunk;
    }
    xSemaphoreGive(s_card_mutex);
    return ESP_OK;
}

static esp_err_t card_write(uint32_t lba, uint32_t offset, const void *buffer, uint32_t length)
{
    const uint8_t *src = buffer;
    if (!s_card_ready || (uint64_t)lba * SD_SECTOR_SIZE + offset + length >
                         (uint64_t)s_card.csd.capacity * s_card.csd.sector_size) return ESP_ERR_INVALID_STATE;
    xSemaphoreTake(s_card_mutex, portMAX_DELAY);
    while (length) {
        uint32_t sector_lba = lba + offset / SD_SECTOR_SIZE;
        uint32_t sector_offset = offset % SD_SECTOR_SIZE;
        if (sector_offset == 0 && length >= SD_SECTOR_SIZE) {
            uint32_t sectors = length / SD_SECTOR_SIZE;
            if (sectors > SD_BATCH_SECTORS) sectors = SD_BATCH_SECTORS;
            uint32_t chunk = sectors * SD_SECTOR_SIZE;
            memcpy(s_multi_sector_buffer, src, chunk);
            esp_err_t err = sdmmc_write_sectors(&s_card, s_multi_sector_buffer,
                                                 sector_lba, sectors);
            if (err != ESP_OK) { xSemaphoreGive(s_card_mutex); return err; }
            src += chunk; length -= chunk; offset += chunk;
            continue;
        }
        uint32_t chunk = SD_SECTOR_SIZE - sector_offset;
        if (chunk > length) chunk = length;
        esp_err_t err;
        if (sector_offset || chunk != SD_SECTOR_SIZE) {
            err = sdmmc_read_sectors(&s_card, s_sector_buffer, sector_lba, 1);
            if (err != ESP_OK) { xSemaphoreGive(s_card_mutex); return err; }
        }
        memcpy(s_sector_buffer + sector_offset, src, chunk);
        err = sdmmc_write_sectors(&s_card, s_sector_buffer, sector_lba, 1);
        if (err != ESP_OK) { xSemaphoreGive(s_card_mutex); return err; }
        src += chunk; length -= chunk; offset += chunk;
    }
    xSemaphoreGive(s_card_mutex);
    return ESP_OK;
}

// Host state queries
bool sd_storage_can_switch_mode(void)
{
    return !s_storage_usb_active || s_host_ejected || !tud_connected();
}

bool sd_storage_usb_attached(void)
{
    return s_storage_usb_active && tud_connected() && !tud_suspended();
}

bool sd_storage_usb_mounted(void)
{
    return s_storage_usb_active && tud_mounted();
}

void sd_storage_set_eject_warning(bool active)
{
    s_eject_warning = active;
}

// TinyUSB MSC storage backend
void tud_msc_inquiry_cb(uint8_t lun, uint8_t vendor_id[8], uint8_t product_id[16], uint8_t product_rev[4])
{
    (void)lun;
    memcpy(vendor_id, "AmbM8    ", 8);
    memcpy(product_id, "FS              ", 16);
    memcpy(product_rev, "1.0 ", 4);
}

bool tud_msc_test_unit_ready_cb(uint8_t lun)
{
    if (!s_card_ready || s_host_ejected) tud_msc_set_sense(lun, SCSI_SENSE_NOT_READY, 0x3a, 0x00);
    return s_card_ready && !s_host_ejected;
}

void tud_msc_capacity_cb(uint8_t lun, uint32_t *block_count, uint16_t *block_size)
{
    (void)lun;
    *block_count = s_card_ready ? s_card.csd.capacity : 0;
    *block_size = SD_SECTOR_SIZE;
}

// MSC start/stop callback
bool tud_msc_start_stop_cb(uint8_t lun, uint8_t power_condition, bool start, bool load_eject)
{
    (void)lun; (void)power_condition;
    if (load_eject) { s_host_ejected = !start; display_set_io(start ? DISPLAY_IO_NONE : DISPLAY_IO_UNMOUNTED); }
    return true;
}

bool tud_msc_is_writable_cb(uint8_t lun) { (void)lun; return true; }

int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset, void *buffer, uint32_t bufsize)
{
    if (card_read(lba, offset, buffer, bufsize) != ESP_OK) {
        tud_msc_set_sense(lun, SCSI_SENSE_MEDIUM_ERROR, 0x11, 0x00);
        return -1;
    }
    s_read_count++;
    return (int32_t)bufsize;
}

int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset, uint8_t *buffer, uint32_t bufsize)
{
    if (card_write(lba, offset, buffer, bufsize) != ESP_OK) {
        tud_msc_set_sense(lun, SCSI_SENSE_MEDIUM_ERROR, 0x0c, 0x00);
        return -1;
    }
    s_write_count++;
    return (int32_t)bufsize;
}

int32_t tud_msc_scsi_cb(uint8_t lun, const uint8_t scsi_cmd[16], void *buffer, uint16_t bufsize)
{
    (void)scsi_cmd;
    (void)buffer;
    (void)bufsize;
    tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x20, 0x00);
    return -1;
}

// Hosts the TinyUSB device stack MSC
static void tinyusb_task(void *arg)
{
    (void)arg;
    while (true) tud_task();
}

// Boots SD mode
esp_err_t sd_storage_start(void)
{
    const esp_ldo_channel_config_t ldo_config = {
        .chan_id = 4, .voltage_mv = 3300, .voltage_stable_delay_us = 200,
    };
    esp_err_t err = esp_ldo_acquire_channel(&ldo_config, &s_sd_ldo);
    if (err != ESP_OK) return err;

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;
    sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
    slot.clk = GPIO_NUM_43; slot.cmd = GPIO_NUM_44;
    slot.d0 = GPIO_NUM_39; slot.d1 = GPIO_NUM_40;
    slot.d2 = GPIO_NUM_41; slot.d3 = GPIO_NUM_42;
    slot.width = 4;
    slot.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    ESP_RETURN_ON_ERROR(sdmmc_host_init(), TAG, "SDMMC host init failed");
    ESP_RETURN_ON_ERROR(sdmmc_host_init_slot(host.slot, &slot), TAG, "SDMMC slot init failed");
    memset(&s_card, 0, sizeof(s_card));
    ESP_RETURN_ON_ERROR(sdmmc_card_init(&host, &s_card), TAG, "SD card init failed");
    s_card_ready = true;
    uint64_t card_bytes=(uint64_t)s_card.csd.capacity*s_card.csd.sector_size;
    display_set_storage(card_bytes, card_bytes, true);
    s_card_mutex = xSemaphoreCreateMutex();
    if (!s_card_mutex) return ESP_ERR_NO_MEM;
    sdmmc_card_print_info(stdout, &s_card);

    const usb_phy_config_t phy_config = {
        .controller = USB_PHY_CTRL_OTG,
        .target = USB_PHY_TARGET_INT,
        .otg_mode = USB_OTG_MODE_DEVICE,
#if CONFIG_TINYUSB_RHPORT_HS
        .otg_speed = USB_PHY_SPEED_HIGH,
#endif
    };
    ESP_RETURN_ON_ERROR(usb_new_phy(&phy_config, &s_usb_phy), TAG, "USB PHY init failed");
    if (!tusb_init()) return ESP_FAIL;
    if (xTaskCreate(tinyusb_task, "TinyUSB", 4096, NULL, 5, NULL) != pdPASS) return ESP_ERR_NO_MEM;
    ESP_RETURN_ON_ERROR(sd_status_led_init(), TAG, "status LED init failed");
    if (xTaskCreate(sd_status_led_task, "sd_status_led", 2048, NULL, 3, NULL) != pdPASS) return ESP_ERR_NO_MEM;
    s_storage_usb_active = true;
    ESP_LOGI(TAG, "SD card is available as USB mass storage");
    return ESP_OK;
}

// Card bring-up for REC engine
esp_err_t sd_card_init_for_recording(void)
{
    if (s_card_ready) {
        ESP_LOGI(TAG, "SD card already initialized");
        return ESP_OK;
    }

    const esp_ldo_channel_config_t ldo_config = {
        .chan_id = 4, .voltage_mv = 3300, .voltage_stable_delay_us = 200,
    };
    esp_err_t err = esp_ldo_acquire_channel(&ldo_config, &s_sd_ldo);
    if (err != ESP_OK) return err;

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.max_freq_khz = SDMMC_FREQ_DEFAULT;
    sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
    slot.clk = GPIO_NUM_43; slot.cmd = GPIO_NUM_44;
    slot.d0 = GPIO_NUM_39; slot.d1 = GPIO_NUM_40;
    slot.d2 = GPIO_NUM_41; slot.d3 = GPIO_NUM_42;
    slot.width = 4;
    slot.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    ESP_RETURN_ON_ERROR(sdmmc_host_init(), TAG, "SDMMC host init failed");
    ESP_RETURN_ON_ERROR(sdmmc_host_init_slot(host.slot, &slot), TAG, "SDMMC slot init failed");
    memset(&s_card, 0, sizeof(s_card));
    ESP_RETURN_ON_ERROR(sdmmc_card_init(&host, &s_card), TAG, "SD card init failed");
    s_card_ready = true;
    uint64_t card_bytes = (uint64_t)s_card.csd.capacity * s_card.csd.sector_size;
    uint64_t free_bytes = (uint64_t)(s_card.csd.capacity - 1000) * s_card.csd.sector_size;
    display_set_storage(card_bytes, free_bytes, true);
    s_card_mutex = xSemaphoreCreateMutex();
    if (!s_card_mutex) return ESP_ERR_NO_MEM;
    sdmmc_card_print_info(stdout, &s_card);

    ESP_LOGI(TAG, "SD card initialized for recording");
    return ESP_OK;
}
