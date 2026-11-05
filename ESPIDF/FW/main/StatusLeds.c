// status LEDs

#include "StatusLeds.h"

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"

#define LED_GPIO    GPIO_NUM_31
#define LED_COUNT   2
#define TIMER_PERIOD_US 250000

static const char *TAG = "status_leds";
static led_strip_handle_t s_strip;
static esp_timer_handle_t s_timer;
static volatile bool s_recording;
static volatile int s_clip_flashes;
static uint32_t s_tick;

// Blink pattern timer
static void status_leds_timer_callback(void *arg)
{
    (void)arg;
    s_tick++;
    bool on = s_tick & 1;

    uint8_t r0 = 0, g0 = 0, b0 = 0;
    uint8_t r1 = 0, g1 = 0, b1 = 0;

    if (s_recording && on) {
        r0 = 90;
    }
    if (s_clip_flashes > 0) {
        if (on) {
            r1 = 100;
            g1 = 50;
        } else {
            s_clip_flashes--;
        }
    }

    if (!s_strip) return;
    led_strip_set_pixel(s_strip, 0, r0, g0, b0);
    led_strip_set_pixel(s_strip, 1, r1, g1, b1);
    led_strip_refresh(s_strip);
}

// Starts LED driver
esp_err_t status_leds_start(void)
{
    if (s_timer) return ESP_OK;

    const led_strip_config_t strip = {
        .strip_gpio_num = LED_GPIO,
        .max_leds = LED_COUNT,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags.invert_out = false,
    };
    const led_strip_rmt_config_t rmt = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
        .flags.with_dma = false,
    };
    esp_err_t err = led_strip_new_rmt_device(&strip, &rmt, &s_strip);
    if (err != ESP_OK) {
        s_strip = NULL;
        return err;
    }
    led_strip_clear(s_strip);

    const esp_timer_create_args_t timer_args = {
        .callback = status_leds_timer_callback,
        .name = "status_leds",
        .dispatch_method = ESP_TIMER_TASK,
    };
    err = esp_timer_create(&timer_args, &s_timer);
    if (err != ESP_OK) {
        led_strip_del(s_strip);
        s_strip = NULL;
        return err;
    }
    err = esp_timer_start_periodic(s_timer, TIMER_PERIOD_US);
    if (err != ESP_OK) {
        esp_timer_delete(s_timer);
        s_timer = NULL;
        led_strip_del(s_strip);
        s_strip = NULL;
        return err;
    }
    ESP_LOGI(TAG, "status LEDs started on GPIO %d", LED_GPIO);
    return ESP_OK;
}

// Stops the timer
void status_leds_stop(void)
{
    if (s_timer) {
        esp_timer_stop(s_timer);
        esp_timer_delete(s_timer);
        s_timer = NULL;
    }
    if (s_strip) {
        led_strip_clear(s_strip);
        led_strip_del(s_strip);
        s_strip = NULL;
    }
    s_recording = false;
    s_clip_flashes = 0;
    s_tick = 0;
}

// recording indicator & clip flash req
void status_leds_set_recording(bool on)
{
    s_recording = on;
}

void status_leds_flash_clip(void)
{
    if (s_clip_flashes < 8) s_clip_flashes++;
}
