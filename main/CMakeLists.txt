idf_component_register(SRCS "main.c" "oled_ssd1306.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp32-camera espressif__quirc espressif__esp_jpeg nvs_flash esp_timer esp_psram spi_flash driver esp_driver_gpio esp_driver_ledc esp_driver_uart esp_driver_i2c esp_wifi esp_event esp_netif esp_http_client esp-tls esp_http_server mbedtls lwip)
