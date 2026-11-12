#pragma once

#include "esp_camera.h"

/**
 * ============================================================================
 * ESP32-S3 Integrated 24-Pin FPC Camera Socket Pinout
 * Direct on-board PCB connection (ESP32-S3-WROOM-1 N16R8) - NO external jumper wires
 * ============================================================================
 */
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     15
#define SIOD_GPIO_NUM     4
#define SIOC_GPIO_NUM     5

#define Y9_GPIO_NUM       16
#define Y8_GPIO_NUM       17
#define Y7_GPIO_NUM       18
#define Y6_GPIO_NUM       12
#define Y5_GPIO_NUM       10
#define Y4_GPIO_NUM       8
#define Y3_GPIO_NUM       9
#define Y2_GPIO_NUM       11
#define VSYNC_GPIO_NUM    6
#define HREF_GPIO_NUM     7
#define PCLK_GPIO_NUM     13

/* Aliases for ESP-IDF Camera Driver */
#define CAM_PIN_PWDN      PWDN_GPIO_NUM
#define CAM_PIN_RESET     RESET_GPIO_NUM
#define CAM_PIN_XCLK      XCLK_GPIO_NUM
#define CAM_PIN_SIOD      SIOD_GPIO_NUM
#define CAM_PIN_SIOC      SIOC_GPIO_NUM

#define CAM_PIN_D7        Y9_GPIO_NUM
#define CAM_PIN_D6        Y8_GPIO_NUM
#define CAM_PIN_D5        Y7_GPIO_NUM
#define CAM_PIN_D4        Y6_GPIO_NUM
#define CAM_PIN_D3        Y5_GPIO_NUM
#define CAM_PIN_D2        Y4_GPIO_NUM
#define CAM_PIN_D1        Y3_GPIO_NUM
#define CAM_PIN_D0        Y2_GPIO_NUM

#define CAM_PIN_VSYNC     VSYNC_GPIO_NUM
#define CAM_PIN_HREF      HREF_GPIO_NUM
#define CAM_PIN_PCLK      PCLK_GPIO_NUM

/* Auxiliary signals */
#define FLASH_LED_PIN     2   /* Onboard High-power White Flash LED (GPIO 2) */
#define BUZZER_PIN        1   /* Buzzer signal output (GPIO 1) */

/* OLED 0.96" I2C (SSD1306) pins - conflict-free GPIOs on ESP32-S3-WROOM-1 N16R8 CAM */
#define OLED_I2C_SDA_PIN  21  /* OLED I2C SDA (GPIO 21) */
#define OLED_I2C_SCL_PIN  47  /* OLED I2C SCL (GPIO 47) */
