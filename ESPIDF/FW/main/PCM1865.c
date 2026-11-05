// PCM1865 I2C driver 

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

#include "PCM1865.h"

#define PCM1865_I2C_SDA_GPIO GPIO_NUM_22
#define PCM1865_I2C_SCL_GPIO GPIO_NUM_23
#define PCM1865_I2C_ADDR     0x4A
#define PCM1865_I2C_HZ       100000
#define PCM1865_I2C_TIMEOUT  100

#define PCM1865_PGA_DB 40
#define PCM1865_PGA_REG ((uint8_t)((PCM1865_PGA_DB) * 2))

#define PCM1865_REG_PAGE      0x00
#define PCM1865_REG_PGA_CH1L  0x01
#define PCM1865_REG_PGA_CH1R  0x02
#define PCM1865_REG_PGA_CH2L  0x03
#define PCM1865_REG_PGA_CH2R  0x04
#define PCM1865_REG_ADC1L_SEL 0x06
#define PCM1865_REG_ADC1R_SEL 0x07
#define PCM1865_REG_ADC2L_SEL 0x08
#define PCM1865_REG_ADC2R_SEL 0x09
#define PCM1865_REG_I2S_FMT   0x0B
#define PCM1865_REG_TDM_OSEL  0x0C
#define PCM1865_REG_TX_OFFSET 0x0D
#define PCM1865_REG_CLK_CTRL  0x20
#define PCM1865_REG_PLL_CTRL  0x28
#define PCM1865_REG_PWR_CTRL  0x70
#define PCM1865_REG_DSP_CTRL  0x71

static const char *TAG = "pcm1865";

static i2c_master_bus_handle_t s_i2c_bus;
static i2c_master_dev_handle_t s_i2c_dev;
static bool s_initialized;
static bool s_present;

typedef struct {
    uint8_t reg;
    uint8_t val;
} pcm1865_regval_t;

static const pcm1865_regval_t s_config[] = {
    {PCM1865_REG_PAGE, 0x00},

    {PCM1865_REG_ADC1L_SEL, 0x41},
    {PCM1865_REG_ADC1R_SEL, 0x41},
    {PCM1865_REG_ADC2L_SEL, 0x42},
    {PCM1865_REG_ADC2R_SEL, 0x42},

    {PCM1865_REG_PGA_CH1L, PCM1865_PGA_REG},
    {PCM1865_REG_PGA_CH1R, PCM1865_PGA_REG},
    {PCM1865_REG_PGA_CH2L, PCM1865_PGA_REG},
    {PCM1865_REG_PGA_CH2R, PCM1865_PGA_REG},

    {PCM1865_REG_I2S_FMT, 0x03},

    {PCM1865_REG_TDM_OSEL, 0x01},

    {PCM1865_REG_TX_OFFSET, 0x00},

    {PCM1865_REG_CLK_CTRL, 0x01},

    {PCM1865_REG_PLL_CTRL, 0x03},

    {PCM1865_REG_PWR_CTRL, 0x70},
    {PCM1865_REG_DSP_CTRL, 0x10},
};

// R+W registers
static esp_err_t pcm1865_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t data[2] = {reg, val};
    return i2c_master_transmit(s_i2c_dev, data, sizeof(data),
                               PCM1865_I2C_TIMEOUT);
}

static esp_err_t pcm1865_read_reg(uint8_t reg, uint8_t *val)
{
    return i2c_master_transmit_receive(s_i2c_dev, &reg, 1, val, 1,
                                       PCM1865_I2C_TIMEOUT);
}

// Detects codec on bus
static esp_err_t pcm1865_probe(void)
{
    uint8_t state = 0;
    esp_err_t err = pcm1865_read_reg(0x72, &state);

    if (err != ESP_OK) {
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "found at I2C 0x%02X (state 0x%02X)", PCM1865_I2C_ADDR, state);
    return ESP_OK;
}

// S/W rst register handshake
static esp_err_t pcm1865_reset(void)
{
    esp_err_t err = pcm1865_write_reg(PCM1865_REG_PAGE, 0xFE);
    if (err != ESP_OK) {
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    return pcm1865_write_reg(PCM1865_REG_PAGE, 0x00);
}

// Clock-status decode, fs band & maps BCK/LRCK : codes
static const char *pcm1865_state_name(uint8_t state)
{
    switch (state & 0x0F) {
    case 0x0: return "power down";
    case 0x1: return "wait clock stable";
    case 0x2: return "release reset";
    case 0x3: return "standby";
    case 0x4: return "fade in";
    case 0x5: return "fade out";
    case 0x9: return "sleep";
    case 0xF: return "run";
    default:  return "reserved";
    }
}

static uint32_t pcm1865_ratio_from_code(uint8_t code)
{
    static const uint32_t table[8] = {0, 32, 48, 64, 256, 0, 0, 0};
    return table[code & 0x07];
}

static void pcm1865_log_status(void)
{
    uint8_t state = 0, fs = 0, ratio = 0, clk_err = 0;

    for (int i = 0; i < 20; i++) {
        if (pcm1865_read_reg(0x72, &state) == ESP_OK && (state & 0x0F) == 0x0F) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    esp_err_t e_state = pcm1865_read_reg(0x72, &state);
    esp_err_t e_fs    = pcm1865_read_reg(0x73, &fs);
    esp_err_t e_ratio = pcm1865_read_reg(0x74, &ratio);
    esp_err_t e_err   = pcm1865_read_reg(0x75, &clk_err);

    if (e_state != ESP_OK || e_fs != ESP_OK || e_ratio != ESP_OK ||
        e_err != ESP_OK) {
        ESP_LOGW(TAG, "status readback failed");
        return;
    }

    ESP_LOGI(TAG, "status: %s (0x%02X), fS info %u, BCK/LRCK %u, SCK/LRCK %u, errors 0x%02X",
             pcm1865_state_name(state), state,
             (unsigned)(fs & 0x07),
             (unsigned)pcm1865_ratio_from_code((ratio >> 4) & 0x07),
             (unsigned)pcm1865_ratio_from_code(ratio & 0x07),
             (unsigned)clk_err);

    if ((state & 0x0F) != 0x0F) {
        ESP_LOGW(TAG, "codec is not in run state; check BCK/LRCK at 256x, 48 kHz");
    }
    if (pcm1865_ratio_from_code((ratio >> 4) & 0x07) != 256) {
        ESP_LOGW(TAG, "unexpected BCK/LRCK ratio; PCM1865 TDM requires 256 BCK per frame");
    }
}

// debugging tingy
void pcm1865_report_status(void)
{
    uint8_t state = 0, fs = 0, ratio = 0;

    if (!s_present) {
        return;
    }
    if (pcm1865_read_reg(0x73, &fs) != ESP_OK ||
        pcm1865_read_reg(0x74, &ratio) != ESP_OK) {
        ESP_LOGW(TAG, "runtime status readback failed");
        return;
    }
    (void)pcm1865_read_reg(0x72, &state);
    ESP_LOGI(TAG, "runtime status: %s, fS info %u, BCK/LRCK %u",
             pcm1865_state_name(state),
             (unsigned)(fs & 0x07),
             (unsigned)pcm1865_ratio_from_code((ratio >> 4) & 0x07));
}

// probe, reset, defaults and architecture configs
esp_err_t pcm1865_init(void)
{
    if (s_initialized) {
        return s_present ? ESP_OK : ESP_ERR_NOT_FOUND;
    }
    s_initialized = true;
    s_present = false;

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = PCM1865_I2C_SDA_GPIO,
        .scl_io_num = PCM1865_I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t err = i2c_new_master_bus(&bus_cfg, &s_i2c_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus init failed: %s", esp_err_to_name(err));
        return err;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = PCM1865_I2C_ADDR,
        .scl_speed_hz = PCM1865_I2C_HZ,
    };

    err = i2c_master_bus_add_device(s_i2c_bus, &dev_cfg, &s_i2c_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C device add failed: %s", esp_err_to_name(err));
        i2c_del_master_bus(s_i2c_bus);
        s_i2c_bus = NULL;
        return err;
    }

    err = pcm1865_probe();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "PCM1865 not found at 0x%02X; microphone input disabled",
                 PCM1865_I2C_ADDR);
        return err;
    }

    err = pcm1865_reset();
    if (err == ESP_OK) {
        for (size_t i = 0; i < sizeof(s_config) / sizeof(s_config[0]); i++) {
            err = pcm1865_write_reg(s_config[i].reg, s_config[i].val);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "register 0x%02X write failed: %s",
                         s_config[i].reg, esp_err_to_name(err));
                break;
            }
        }
    }

    if (err != ESP_OK) {
        return err;
    }

    s_present = true;
    ESP_LOGI(TAG, "configured: 4ch TDM, 32-bit slots, %+d dB PGA, 48 kHz / 256 BCK",
             PCM1865_PGA_DB);
    pcm1865_log_status();
    return ESP_OK;
}

// true if successful config
bool pcm1865_present(void)
{
    return s_present;
}

// reconfig per-ADC VINLx/VINRx SEL codes
esp_err_t pcm1865_set_input_mux(const uint8_t sel[4])
{
    static const uint8_t regs[4] = {
        PCM1865_REG_ADC1L_SEL, PCM1865_REG_ADC1R_SEL,
        PCM1865_REG_ADC2L_SEL, PCM1865_REG_ADC2R_SEL,
    };
    if (!s_present) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t err = ESP_OK;
    for (int i = 0; i < 4; i++) {

        esp_err_t e = pcm1865_write_reg(regs[i], (uint8_t)((sel[i] & 0x3F) | 0x40));
        if (e != ESP_OK) {
            ESP_LOGE(TAG, "input mux reg 0x%02X write failed: %s", regs[i],
                     esp_err_to_name(e));
            err = e;
        }
    }
    return err;
}

// reconfig 4ch TDM / stereo select on DOUT
esp_err_t pcm1865_set_tdm_output(uint8_t osel)
{
    if (!s_present) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t fmt = (osel & 0x03) != 0 ? 0x03 : 0x00;
    esp_err_t err = pcm1865_write_reg(PCM1865_REG_I2S_FMT, fmt);
    if (err != ESP_OK) {
        return err;
    }
    return pcm1865_write_reg(PCM1865_REG_TDM_OSEL, (uint8_t)(osel & 0x03));
}
