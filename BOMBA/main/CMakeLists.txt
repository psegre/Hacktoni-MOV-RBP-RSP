idf_component_register(
    SRCS "main.c" "display.c" "audio.c" "buttons.c keyboard.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_lcd esp_driver_spi esp_driver_i2s esp_driver_gpio nvs_flash esp_event esp_netif esp_wifi
)
