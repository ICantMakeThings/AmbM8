// AmbM8 boot manager, initilisation and switching

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "SD.h"
#include "UAC2.h"
#include "Display.h"
#include "Standalone.h"
#include "Recording.h"
#include "tusb.h"

#define SIDE_KEY_GPIO GPIO_NUM_30
#define MODE_HOLD_MS 1000
#define USB_DETECT_WINDOW_MS 2500

static const char *TAG = "app_mode";
static uint8_t s_boot_mode;
static volatile uint8_t s_active_mode;

// Routes UI events for controling active modes
static void control_event(display_control_event_t event, int value)
{
    if (s_active_mode == 2) {
        standalone_control(event, value);
        return;
    }
    if (s_active_mode != 0) return;
    switch (event) {
    case DISPLAY_CONTROL_ENCODER_STEP: uac2_encoder_step(value); break;
    case DISPLAY_CONTROL_ENCODER_CLICK:
        if(display_advanced_page()) uac2_host_media(UAC2_MEDIA_PLAY_PAUSE);
        else uac2_encoder_click();
        break;
    case DISPLAY_CONTROL_ENCODER_HOLD: uac2_encoder_hold(); break;
    case DISPLAY_CONTROL_ENCODER_DOUBLE: uac2_toggle_encoder_mode(); break;
    case DISPLAY_CONTROL_NEXT:
        if(value) uac2_host_media(UAC2_MEDIA_NEXT_TRACK);
        break;
    case DISPLAY_CONTROL_PREVIOUS:
        if(value) uac2_host_media(UAC2_MEDIA_PREV_TRACK);
        break;
    case DISPLAY_CONTROL_REC:
        break;
    case DISPLAY_CONTROL_PLAY_PAUSE:
        break;
    default:
        break;
    }
}

// boot in USB storage
bool app_is_sd_mode(void)
{
    return s_boot_mode == 1;
}

// keeps latest mode in NVS
static void mode_save_and_restart(uint8_t mode)
{
    nvs_handle_t nvs;
    ESP_ERROR_CHECK(nvs_open("app", NVS_READWRITE, &nvs));
    ESP_ERROR_CHECK(nvs_set_u8(nvs, "usb_mode", mode));
    ESP_ERROR_CHECK(nvs_commit(nvs));
    nvs_close(nvs);
    const char *mode_str;
    switch (mode) {
        case 1: mode_str = "SD storage"; break;
        case 2: mode_str = "standalone"; break;
        default: mode_str = "USB audio"; break;
    }
    ESP_LOGI(TAG, "Switching to %s mode", mode_str);
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_restart();
}

// Switcher to switch modes from UAC2, Mass Storage or Standalone,
static void mode_button_task(void *arg)
{
    (void)arg;
    while (true) {
        if (gpio_get_level(SIDE_KEY_GPIO) == 0) {
            int64_t held_ms = 0;
            while (gpio_get_level(SIDE_KEY_GPIO) == 0 && held_ms < MODE_HOLD_MS) {
                vTaskDelay(pdMS_TO_TICKS(20));
                held_ms += 20;
            }
            if (held_ms >= MODE_HOLD_MS) {
                if (s_boot_mode == 1 && !sd_storage_can_switch_mode()) {
                    ESP_LOGW(TAG, "Safely eject the SD drive on the host before switching modes");
                    sd_storage_set_eject_warning(true);
                    while (gpio_get_level(SIDE_KEY_GPIO) == 0) vTaskDelay(pdMS_TO_TICKS(20));
                    sd_storage_set_eject_warning(false);
                    continue;
                }
                if (s_active_mode == 2 && recording_is_active()) {
                    ESP_LOGW(TAG, "Stop recording before switching modes");
                    while (gpio_get_level(SIDE_KEY_GPIO) == 0) vTaskDelay(pdMS_TO_TICKS(20));
                    continue;
                }
                mode_save_and_restart((s_boot_mode + 1) % 3);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// helper for mode change
static void late_usb_audio_task(void *arg)
{
    (void)arg;
    while (!uac2_high_speed_host_connected()) vTaskDelay(pdMS_TO_TICKS(20));

    if (recording_is_active()) {
        ESP_LOGI(TAG, "USB host connected while recording; waiting for recording to finish");
        while (recording_is_active()) vTaskDelay(pdMS_TO_TICKS(100));
        if (!uac2_high_speed_host_connected()) {
            vTaskDelete(NULL);
            return;
        }
    }

    ESP_LOGI(TAG, "HS USB host connected... switching to UAC2");
    esp_err_t err=standalone_stop_for_usb();
    if(err!=ESP_OK) {
        ESP_LOGE(TAG,"Could not stop standalone playback for USB switch: %s",esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }
    display_set_mode(DISPLAY_MODE_UAC2);
    s_active_mode=0;
    uac2_start();
    vTaskDelete(NULL);
}

// Watches UAC2
static void usb_disconnect_task(void *arg)
{
    (void)arg;

    for (;;) {

        while (!uac2_high_speed_host_connected()) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        ESP_LOGI(TAG, "UAC2 host detected; monitoring for USB disconnect");

        while (uac2_high_speed_host_connected()) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        vTaskDelay(pdMS_TO_TICKS(1500));
        if (!uac2_high_speed_host_connected()) {
            ESP_LOGI(TAG, "UAC2 host disconnected; restarting into auto/standalone mode");
            mode_save_and_restart(0);
        }
    }
}

// MSC watcher
static void sd_usb_disconnect_task(void *arg)
{
    (void)arg;
    for(;;) {
        while(!sd_storage_usb_mounted()) vTaskDelay(pdMS_TO_TICKS(100));
        ESP_LOGI(TAG,"SD mass-storage host mounted");
        while(sd_storage_usb_attached()) vTaskDelay(pdMS_TO_TICKS(100));
        vTaskDelay(pdMS_TO_TICKS(1500));
        if(!sd_storage_usb_attached()) {
            if (sd_storage_can_switch_mode()) {
                ESP_LOGI(TAG,"SD MSC USB disconnected, returning to standalone mode");
                mode_save_and_restart(0);
            }
            ESP_LOGW(TAG,"USB bus suspended while SD is still mounted; safely eject it on the host before switching modes");
            while(!sd_storage_usb_attached()) vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

// NVS restore boot
void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    } else {
        ESP_ERROR_CHECK(err);
    }

    nvs_handle_t nvs;
    ESP_ERROR_CHECK(nvs_open("app", NVS_READWRITE, &nvs));
    uint8_t saved_mode = 0;
    (void)nvs_get_u8(nvs, "usb_mode", &saved_mode);
    nvs_close(nvs);
    s_boot_mode = saved_mode <= 2 ? saved_mode : saved_mode == 3 ? 2 : 0;
    ESP_LOGI(TAG, "Saved USB mode: %u (0=auto USB, 1=SD, 2=standalone)",
             (unsigned)s_boot_mode);

    ESP_ERROR_CHECK(gpio_config(&(gpio_config_t){
        .pin_bit_mask = 1ULL << SIDE_KEY_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    }));
    BaseType_t created = xTaskCreate(mode_button_task, "mode_button", 2048, NULL, 4, NULL);
    ESP_ERROR_CHECK(created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);

    uint8_t active_mode=s_boot_mode;
    if(s_boot_mode==0) {
        ESP_LOGI(TAG,"Probing USB for a high-speed host (%d ms window)",USB_DETECT_WINDOW_MS);
        uac2_probe_start();
        for(int elapsed=0;elapsed<USB_DETECT_WINDOW_MS;elapsed+=10) {
            if(uac2_high_speed_host_connected()) break;
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        ESP_LOGI(TAG, "USB probe result: mounted=%d, speed=%d (2=high-speed)",
                 (int)tud_mounted(), (int)tud_speed_get());
        active_mode=uac2_high_speed_host_connected()?0:2;
        if(active_mode==0) ESP_LOGI(TAG,"High-speed USB host enumerated");
        else ESP_LOGI(TAG,"No HS USB host; loading standalone mode");
    }
    s_active_mode = active_mode;

    const char *mode_names[3] = {"USB audio", "SD storage", "standalone"};
    ESP_LOGI(TAG, "Starting in %s mode", mode_names[active_mode]);

    display_mode_t display_mode;
    if (active_mode == 1) display_mode = DISPLAY_MODE_STORAGE;
    else if (active_mode == 2) display_mode = DISPLAY_MODE_STANDALONE;
    else display_mode = DISPLAY_MODE_UAC2;

    display_set_control_callback(control_event);
    display_start(display_mode);

    if (active_mode == 1) {
        err = sd_storage_start();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "SD storage did not start (%s); check card power and SD wiring",
                     esp_err_to_name(err));
        }
        BaseType_t watcher = xTaskCreate(sd_usb_disconnect_task, "sd_usb_detach", 2048, NULL, 4, NULL);
        if (watcher != pdPASS) ESP_LOGE(TAG, "Could not start SD USB disconnect detector");
    } else if (active_mode == 2) {
        err = standalone_start();
        if (err != ESP_OK) ESP_LOGE(TAG, "Standalone player did not start (%s)", esp_err_to_name(err));
        if (s_boot_mode == 0) {
            BaseType_t watcher = xTaskCreate(late_usb_audio_task, "late_usb_audio", 3072, NULL, 6, NULL);
            if (watcher != pdPASS) ESP_LOGE(TAG, "Could not start late USB audio detector");
        }
    } else {
        uac2_start();
    }

    if (s_boot_mode == 0) {
        BaseType_t watcher = xTaskCreate(usb_disconnect_task, "usb_disconnect",
                                         2048, NULL, 5, NULL);
        if (watcher != pdPASS) {
            ESP_LOGE(TAG, "Could not start USB disconnect detector");
        }
    }
}
