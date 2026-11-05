// API for Recording.c

#pragma once

#include <stdbool.h>
#include "esp_err.h"
#include "Display.h"

esp_err_t recording_start(void);
esp_err_t recording_stop(void);

bool recording_is_active(void);
bool recording_is_ready(void);
void recording_set_sd_mounted(bool mounted);

void recording_set_auto_gain(bool enabled);
bool recording_auto_gain_enabled(void);
