// API for PCM1865.c

#pragma once

#include <stdbool.h>
#include "esp_err.h"

esp_err_t pcm1865_init(void);

void pcm1865_report_status(void);

// VINLx/VINRx mux SEL codes
esp_err_t pcm1865_set_input_mux(const uint8_t sel[4]);
// 0 = stereo, 1 = 4ch TDM on DOUT
esp_err_t pcm1865_set_tdm_output(uint8_t osel);

bool pcm1865_present(void);
