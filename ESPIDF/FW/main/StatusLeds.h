// API for StatusLeds.c

#pragma once

#include <stdbool.h>
#include "esp_err.h"

esp_err_t status_leds_start(void);
void status_leds_stop(void);
void status_leds_set_recording(bool on);
void status_leds_flash_clip(void);
