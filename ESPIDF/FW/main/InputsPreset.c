// presets for the PCM1865 via mux/TDM and capture-rate configs (changing sample, rate, channel ammount, ect..)

#include "InputsPreset.h"

#include <stddef.h>

#include "PCM1865.h"
#include "esp_check.h"
#include "esp_log.h"
#include "nvs.h"

#define NVS_NAMESPACE "audio"

static const char *TAG = "inputs";

static const input_preset_t s_presets[INPUT_PRESET_COUNT] = {

    { .name = "4CHAUX 1/2", .slot_label = {"1L", "1R", "2L", "2R"},
      .channels = 4, .ambisonic_slots = 4, .rate = 48000,
      .tdm_osel = 0x01, .mux_sel = {0x01, 0x01, 0x02, 0x02},
      .mask = 0x0F, .total_slots = 8, .mclk_mult = 768 },

    { .name = "4CHAUX 3/4", .slot_label = {"3L", "3R", "4L", "4R"},
      .channels = 4, .ambisonic_slots = 4, .rate = 48000,
      .tdm_osel = 0x01, .mux_sel = {0x04, 0x04, 0x08, 0x08},
      .mask = 0x0F, .total_slots = 8, .mclk_mult = 768 },

    { .name = "2CHAUX 1 192K", .slot_label = {"1L", "1R"},
      .channels = 2, .ambisonic_slots = 0, .rate = 192000,
      .tdm_osel = 0x00, .mux_sel = {0x01, 0x01, 0x00, 0x00},
      .mask = 0x03, .total_slots = 2, .mclk_mult = 256 },

    { .name = "2CHAUX 2 192K", .slot_label = {"2L", "2R"},
      .channels = 2, .ambisonic_slots = 0, .rate = 192000,
      .tdm_osel = 0x00, .mux_sel = {0x02, 0x02, 0x00, 0x00},
      .mask = 0x03, .total_slots = 2, .mclk_mult = 256 },

    { .name = "2CHAUX 3 192K", .slot_label = {"3L", "3R"},
      .channels = 2, .ambisonic_slots = 0, .rate = 192000,
      .tdm_osel = 0x00, .mux_sel = {0x04, 0x04, 0x00, 0x00},
      .mask = 0x03, .total_slots = 2, .mclk_mult = 256 },

    { .name = "2CHAUX 4 192K", .slot_label = {"4L", "4R"},
      .channels = 2, .ambisonic_slots = 0, .rate = 192000,
      .tdm_osel = 0x00, .mux_sel = {0x08, 0x08, 0x00, 0x00},
      .mask = 0x03, .total_slots = 2, .mclk_mult = 256 },
};

static uint8_t s_index;
static uint8_t s_mic_map[4] = {0, 1, 2, 3};
static bool s_rec_wxyz;

// Loads newest flags from NVS and double checking for corruption or sm
static void inp_load_from_nvs(void)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK) {
        return;
    }
    int8_t saved = 0;
    if (nvs_get_i8(nvs, "mon_inp", &saved) == ESP_OK) {

        if (saved >= 0 && saved < INPUT_PRESET_COUNT) {
            s_index = (uint8_t)saved;
        }
    }
    uint8_t map[4] = {0, 1, 2, 3};
    size_t len = sizeof(map);
    if (nvs_get_blob(nvs, "mon_map", map, &len) == ESP_OK && len == sizeof(map)) {
        for (int i = 0; i < 4; i++) {
            if (map[i] < 4) s_mic_map[i] = map[i];
        }
    }
    saved = 0;
    if (nvs_get_i8(nvs, "rec_wxyz", &saved) == ESP_OK) {
        s_rec_wxyz = saved != 0;
    }
    nvs_close(nvs);
}

static void inp_ensure_loaded(void)
{
    static bool s_loaded;
    if (!s_loaded) {
        s_loaded = true;
        inp_load_from_nvs();
    }
}

// preset table accessors
uint8_t inp_preset_count(void) { return INPUT_PRESET_COUNT; }
const input_preset_t *inp_preset_at(uint8_t index)
{
    return &s_presets[index < INPUT_PRESET_COUNT ? index : 0];
}
uint8_t inp_preset_index(void)
{
    inp_ensure_loaded();
    return s_index;
}
const input_preset_t *inp_preset(void) { return inp_preset_at(inp_preset_index()); }

esp_err_t inp_preset_save(uint8_t index)
{
    if (index >= INPUT_PRESET_COUNT) return ESP_ERR_INVALID_ARG;
    s_index = index;
    nvs_handle_t nvs;
    esp_err_t err;
    if ((err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs)) != ESP_OK) {
        ESP_LOGW(TAG, "preset save failed: %s", esp_err_to_name(err));
        return err;
    }
    err = nvs_set_i8(nvs, "mon_inp", (int8_t)index);
    if (err == ESP_OK) err = nvs_commit(nvs);
    nvs_close(nvs);
    if (err != ESP_OK) ESP_LOGW(TAG, "preset commit failed: %s", esp_err_to_name(err));
    return err;
}

// makes active preset into codec
esp_err_t inp_preset_apply(void)
{
    const input_preset_t *p = inp_preset();
    ESP_RETURN_ON_FALSE(p != NULL, ESP_ERR_INVALID_STATE, TAG, "no active preset");
    ESP_RETURN_ON_ERROR(pcm1865_set_input_mux(p->mux_sel), TAG, "input mux");
    ESP_RETURN_ON_ERROR(pcm1865_set_tdm_output(p->tdm_osel), TAG, "tdm osel");
    ESP_LOGI(TAG, "preset %s: %u ch, %lu Hz, TDM_OSEL 0x%02X, mux %d/%d/%d/%d",
             p->name, p->channels, (unsigned long)p->rate, p->tdm_osel,
             p->mux_sel[0], p->mux_sel[1], p->mux_sel[2], p->mux_sel[3]);
    return ESP_OK;
}

// mapping mic (FLU/FRD/BLB/BRU)
const uint8_t *inp_mic_map(void)
{
    inp_ensure_loaded();
    return s_mic_map;
}

esp_err_t inp_mic_map_save(const uint8_t map[4])
{
    if (!map) return ESP_ERR_INVALID_ARG;
    for (int i = 0; i < 4; i++) {
        if (map[i] >= 4) return ESP_ERR_INVALID_ARG;
    }
    for (int i = 0; i < 4; i++) s_mic_map[i] = map[i];
    nvs_handle_t nvs;
    esp_err_t err;
    if ((err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs)) != ESP_OK) return err;
    err = nvs_set_blob(nvs, "mon_map", s_mic_map, sizeof(s_mic_map));
    if (err == ESP_OK) err = nvs_commit(nvs);
    nvs_close(nvs);
    return err;
}

// setting to record WXYZ to file
bool inp_rec_wxyz(void)
{
    inp_ensure_loaded();
    return s_rec_wxyz;
}

esp_err_t inp_rec_wxyz_save(bool wxyz)
{
    s_rec_wxyz = wxyz;
    nvs_handle_t nvs;
    esp_err_t err;
    if ((err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs)) != ESP_OK) return err;
    err = nvs_set_i8(nvs, "rec_wxyz", wxyz ? 1 : 0);
    if (err == ESP_OK) err = nvs_commit(nvs);
    nvs_close(nvs);
    return err;
}
