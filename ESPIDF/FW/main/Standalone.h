// API for Standalone.c

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"
#include "Display.h"

esp_err_t standalone_start(void);

esp_err_t standalone_stop_for_usb(void);

void standalone_control(display_control_event_t event, int value);

esp_err_t standalone_recording_monitor_enable(void);
void standalone_recording_monitor_stop(void);

bool standalone_recording_monitor_is_enabled(void);
bool standalone_recording_monitor_process(const int32_t *samples, size_t frames);
