idf_component_register(
    SRCS "SD.c" "main.c" "UAC2.c" "Display.c" "Standalone.c" "PCM1865.c" "Recording.c" "StatusLeds.c" "EqApo.c" "InputsPreset.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES
        esp_driver_i2s
        esp_driver_i2c
        esp_driver_gpio
        esp_driver_spi
        esp_driver_sdmmc
        fatfs
        esp_hw_support
        esp_timer
        esp_ringbuf
        esp_audio_codec
        nvs_flash
        espressif__tinyusb
)
