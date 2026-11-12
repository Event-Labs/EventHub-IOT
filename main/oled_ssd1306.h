#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    OLED_TICKET_IDLE = 0,
    OLED_TICKET_CHECKING,
    OLED_TICKET_VERIFIED,
    OLED_TICKET_OK,
    OLED_TICKET_USED,
    OLED_TICKET_NOT_FOUND,
    OLED_TICKET_WRONG_EVENT,
    OLED_TICKET_NET_ERR,
} oled_ticket_status_t;

typedef enum {
    OLED_IOT_NONE = 0,
    OLED_IOT_UNPAIRED,
    OLED_IOT_PENDING,
    OLED_IOT_REPAIR_REQUIRED,
    OLED_IOT_DISABLED,
    OLED_IOT_NO_EVENT,
    OLED_IOT_READY,
} oled_iot_status_t;

/**
 * @brief Initialize I2C master bus (GPIO SDA/SCL) and SSD1306 128x64 OLED display task.
 * Safe no-op if no SSD1306 panel is attached (logs warning, does not fail boot).
 */
void oled_init(void);

/**
 * @brief Return true if an SSD1306 OLED panel was detected and initialized.
 */
bool oled_is_ready(void);

/**
 * @brief Update the top network status bar (Wi-Fi state, SSID, IP, RSSI, auto-confirm, USB/Standalone, token).
 */
void oled_update_network(bool wifi_connected,
                         const char *ssid,
                         const char *ip,
                         int8_t rssi,
                         bool auto_confirm,
                         bool pc_usb_active,
                         bool has_token);

/**
 * @brief Show a ticket verification / check-in result on the OLED.
 */
void oled_show_ticket_status(oled_ticket_status_t status, const char *ticket_code, const char *sub_msg);

/**
 * @brief Update IoT device pairing / event assignment state shown on idle OLED screen.
 */
void oled_update_iot_status(oled_iot_status_t iot_status,
                            const char *pairing_code,
                            const char *event_code,
                            const char *mac_short);

#ifdef __cplusplus
}
#endif
