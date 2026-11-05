// API for SD.c

#pragma once

#include <stdbool.h>
#include "esp_err.h"


esp_err_t sd_storage_start(void);

esp_err_t sd_card_init_for_recording(void);

bool sd_storage_can_switch_mode(void);

bool sd_storage_usb_attached(void);
bool sd_storage_usb_mounted(void);
void sd_storage_set_eject_warning(bool active);
bool app_is_sd_mode(void);
