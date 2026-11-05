// API for InputsPreset.c

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#define INPUT_PRESET_COUNT 6

typedef struct {
    const char *name;
    const char *slot_label[4];
    uint8_t channels;
    uint8_t ambisonic_slots;
    uint32_t rate;
    uint8_t tdm_osel;
    uint8_t mux_sel[4];
    uint8_t mask;
    uint8_t total_slots;
    uint16_t mclk_mult;
} input_preset_t;

// active preset
uint8_t inp_preset_count(void);
uint8_t inp_preset_index(void);
const input_preset_t *inp_preset(void);
const input_preset_t *inp_preset_at(uint8_t index);
esp_err_t inp_preset_save(uint8_t index);
esp_err_t inp_preset_apply(void);

// Tetra mic map
const uint8_t *inp_mic_map(void);
esp_err_t inp_mic_map_save(const uint8_t map[4]);

// Encoding W/X/Y/Z on file if used
bool inp_rec_wxyz(void);
esp_err_t inp_rec_wxyz_save(bool wxyz);
