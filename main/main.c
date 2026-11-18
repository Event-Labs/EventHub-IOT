/**
 * ============================================================================
 * EventHub QR Check-in Station: Dual-Mode Firmware (USB + Wi-Fi Standalone)
 * Hardware: ESP32-S3-WROOM-1 (N16R8) + OV5640 Camera (Integrated 24-Pin FPC)
 * Dual-Mode Operation:
 *  1. Cắm dây USB: Stream ảnh JPEG @ 2Mbaud lên PC & nhận lệnh điều khiển UART.
 *  2. Rút dây USB (cấp nguồn rời): Tự động bắt Wi-Fi, chạy Web Server cổng 80,
 *     đăng ký thiết bị IoT & tự gửi HTTPS request tới EventHub Backend.
 * ============================================================================
 */

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_psram.h"
#include "esp_heap_caps.h"
#include "esp_rom_sys.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/uart.h"

#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "esp_tls.h"
#include "lwip/ip_addr.h"
#include "lwip/err.h"
#include "lwip/sys.h"

#include "esp_camera.h"
#include "img_converters.h"
#include "camera_pins.h"
#include "oled_ssd1306.h"
#include "quirc.h"
#include "jpeg_decoder.h"

#if defined(__has_include)
#if __has_include("local_config.h")
#include "local_config.h"
#else
#include "local_config.example.h"
#endif
#else
#include "local_config.example.h"
#endif

static const char *TAG = "EVENTHUB_DUAL_CAM";

/* ============================================================================
 * 1. CẤU HÌNH GIAO THỨC UART & CHÂN NGOẠI VI
 * ============================================================================ */
#define UART_PORT_NUM           UART_NUM_0
#define UART_BAUD_RATE          2000000              /* 2.0 Mbps USB Serial */
#define UART_RX_BUF_SIZE        8192
#define UART_TX_BUF_SIZE        32768                /* 32KB TX ring buffer: gửi VGA JPEG không chặn CPU */

static const uint8_t SYNC_MAGIC[4] = {0xAA, 0x55, 0xAA, 0x55};
static const uint8_t END_MAGIC[4]  = {0x55, 0xAA, 0x55, 0xAA};

#ifndef BUZZER_PIN
#define BUZZER_PIN              1                    /* GPIO 1: Còi Buzzer */
#endif

#ifndef FLASH_LED_PIN
#define FLASH_LED_PIN           2                    /* GPIO 2: LED trợ sáng PWM */
#endif

#define LEDC_FLASH_TIMER        LEDC_TIMER_1
#define LEDC_FLASH_MODE         LEDC_LOW_SPEED_MODE
#define LEDC_FLASH_CHANNEL      LEDC_CHANNEL_1

#ifndef SERVO_PIN
#define SERVO_PIN               14                   /* GPIO 14: PWM điều khiển Servo mở cổng */
#endif

#define LEDC_SERVO_TIMER        LEDC_TIMER_2
#define LEDC_SERVO_MODE         LEDC_LOW_SPEED_MODE
#define LEDC_SERVO_CHANNEL      LEDC_CHANNEL_2
#define LEDC_SERVO_DUTY_RES     LEDC_TIMER_14_BIT
#define LEDC_SERVO_FREQ_HZ      50

#define SERVO_MIN_PULSEWIDTH_US 500                  /* 0 độ: đóng thanh chắn (500us) */
#define SERVO_MAX_PULSEWIDTH_US 2500                 /* 180 độ: góc tối đa (2500us) */
#define SERVO_GATE_CLOSE_DEGREE 0                    /* Góc đóng cổng: 0 độ */
#define SERVO_GATE_OPEN_DEGREE  90                   /* Góc mở cổng: 90 độ */
#define SERVO_GATE_HOLD_MS      3000                 /* Thời gian giữ tối thiểu: 3000ms (3s) */
#define SERVO_GATE_SAFETY_TIMEOUT_MS 15000           /* Timeout an toàn: Tự đóng sau 15s nếu người không đi qua */

#ifndef IR_SENSOR_PIN
#define IR_SENSOR_PIN           48                   /* GPIO 48: Cảm biến hồng ngoại phát hiện người qua cổng */
#endif

#ifndef LED_GREEN_PIN
#define LED_GREEN_PIN           41                   /* GPIO 41: Đèn LED Xanh lá (Vé hợp lệ) */
#endif

#ifndef LED_RED_PIN
#define LED_RED_PIN             42                   /* GPIO 42: Đèn LED Đỏ (Vé lỗi / Đã check-in) */
#endif

#define STATUS_LED_HOLD_MS      60000                /* Thời gian sáng đèn: 60,000ms (1 phút) */

/* Wi-Fi & Network Global State */
#define MAX_WIFI_PROFILES 4
typedef struct {
    char ssid[33];
    char pass[65];
} wifi_profile_t;

static wifi_profile_t g_wifi_profiles[MAX_WIFI_PROFILES];
static int g_wifi_profile_count = 0;
static char g_wifi_ssid[33] = "";
static char g_wifi_pass[65] = "";
static bool g_wifi_connected = false;
static char g_wifi_ip[16] = "0.0.0.0";
static int8_t g_wifi_rssi = 0;
static httpd_handle_t g_http_server = NULL;
static volatile bool g_wifi_connecting_now = false;
static volatile uint32_t g_wifi_last_attempt_tick = 0;
static volatile uint32_t g_wifi_bg_fails = 0;
static volatile bool g_wifi_just_connected_led = false;

/* Device Config (Persisted in NVS) & Standalone Auto Check-in State */
static bool g_auto_confirm = false;
static volatile uint32_t g_pc_usb_last_tick = 0;
static SemaphoreHandle_t g_scan_state_mutex = NULL;

/* IoT Device Registration, Pairing & Event Assignment State (NVS: "iot_dev") */
typedef enum {
    IOT_STATE_UNPAIRED = 0,
    IOT_STATE_PENDING,
    IOT_STATE_ACTIVE,
    IOT_STATE_REPAIR_REQUIRED,
    IOT_STATE_DISABLED,
} iot_reg_state_t;

static SemaphoreHandle_t g_iot_mutex = NULL;
static char g_device_mac[18] = "00:00:00:00:00:00";
static char g_device_mac_short[8] = "000000";
static char g_iot_base_url[192] = EVENTHUB_IOT_BASE_URL;
static char g_iot_device_id[64] = "";
static char g_iot_device_secret[160] = "";
static char g_iot_device_name[64] = "";
static char g_iot_pairing_code[16] = "";
static char g_iot_claim_token[160] = "";
static char g_iot_assigned_event_id[64] = "";
static char g_iot_event_code[64] = "";
static char g_iot_event_name[96] = "";
static int g_iot_config_version = 0;
static int g_iot_poll_interval_sec = 5;
static iot_reg_state_t g_iot_state = IOT_STATE_UNPAIRED;
static volatile bool g_iot_force_sync = false;
static volatile bool g_iot_pending_auto_sync = false;
static char g_iot_last_err[96] = "";

#define SNAP_JPG_MAX_BYTES 131072
#define SNAP_GRAY_MAX_BYTES (640 * 480)
static uint8_t *s_snap_jpg_buf = NULL;
static size_t s_snap_jpg_len = 0;
static uint8_t *s_snap_gray_buf = NULL;
static uint16_t s_snap_gray_w = 0;
static uint16_t s_snap_gray_h = 0;
static volatile bool s_snap_is_gray = false;
static volatile bool s_snap_requested = false;
static volatile bool s_snap_ready = false;

typedef struct {
    char qr[128];
    char status[24];
    char message[192];
    uint32_t seq;
    bool auto_checked_in;
} standalone_scan_state_t;

static standalone_scan_state_t s_last_scan = {
    .qr = "",
    .status = "IDLE",
    .message = "San sang quet ve doc lap",
    .seq = 0,
    .auto_checked_in = false
};

#define DEVICE_HISTORY_MAX_ITEMS OLED_HISTORY_MAX_ITEMS

typedef struct {
    char id[48];
    char ticket_code[48];
    char attendee_name[64];
    char event_id[48];
    char event_code[48];
    char event_name[80];
    char action[16];
    char result[24];
    bool success;
    char message[128];
    char scanned_at[36];
} dev_history_item_t;

typedef struct {
    bool initialized;
    bool initial_synced;
    bool is_stale;
    bool offline_cached;
    bool loading;
    int last_http_code;
    char last_error[96];
    int page;
    int limit;
    int total;
    bool has_more;
    char filter[24];
    bool event_only;
    int total_scans;
    int ok_count;
    int verified_count;
    int failed_count;
    int item_count;
    dev_history_item_t items[DEVICE_HISTORY_MAX_ITEMS];
} dev_history_cache_t;

static SemaphoreHandle_t g_history_mutex = NULL;
static dev_history_cache_t s_dev_history = {
    .page = 1,
    .limit = 5,
    .filter = "ALL",
    .is_stale = true,
};

static void sync_oled_network_state(void);
static const char *find_json_key(const char *json, const char *key);
static bool extract_json_str(const char *json, const char *key, char *out, size_t out_max);
static bool extract_json_bool(const char *json, const char *key, bool *out_val);
static bool extract_json_int(const char *json, const char *key, int *out_val);
static const char *extract_json_object_slice(const char *obj_start, char *out_buf, size_t out_max)
{
    if (!obj_start || *obj_start != '{' || !out_buf || out_max < 2) return NULL;
    const char *p = obj_start;
    size_t idx = 0;
    int depth = 0;
    bool in_str = false;
    while (*p) {
        char c = *p++;
        if (idx + 1 < out_max) out_buf[idx++] = c;
        if (c == '\\' && in_str && *p) {
            char esc = *p++;
            if (idx + 1 < out_max) out_buf[idx++] = esc;
            continue;
        }
        if (c == '"') {
            in_str = !in_str;
        } else if (!in_str) {
            if (c == '{') depth++;
            else if (c == '}') {
                depth--;
                if (depth <= 0) break;
            }
        }
    }
    out_buf[idx] = '\0';
    return p;
}

static esp_err_t fetchDeviceHistory(int page, int limit, const char *result_filter, bool event_only);

static const char *iot_state_to_str(iot_reg_state_t st)
{
    switch (st) {
        case IOT_STATE_PENDING: return "PENDING";
        case IOT_STATE_ACTIVE: return "ACTIVE";
        case IOT_STATE_REPAIR_REQUIRED: return "REPAIR_REQUIRED";
        case IOT_STATE_DISABLED: return "DISABLED";
        case IOT_STATE_UNPAIRED:
        default: return "UNPAIRED";
    }
}

static bool is_iot_device_paired_active(void)
{
    return (g_iot_state == IOT_STATE_ACTIVE &&
            g_iot_device_id[0] != '\0' &&
            g_iot_device_secret[0] != '\0');
}

static void init_device_mac(void)
{
    uint8_t mac[6] = {0};
    if (esp_read_mac(mac, ESP_MAC_WIFI_STA) == ESP_OK || esp_efuse_mac_get_default(mac) == ESP_OK) {
        snprintf(g_device_mac, sizeof(g_device_mac), "%02x:%02x:%02x:%02x:%02x:%02x",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        snprintf(g_device_mac_short, sizeof(g_device_mac_short), "%02X%02X%02X",
                 mac[3], mac[4], mac[5]);
    }
}

static void save_iot_nvs_str(const char *key, const char *value)
{
    nvs_handle_t h;
    if (nvs_open("iot_dev", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_str(h, key, value ? value : "");
        nvs_commit(h);
        nvs_close(h);
    }
}

static void save_iot_nvs_i32(const char *key, int32_t value)
{
    nvs_handle_t h;
    if (nvs_open("iot_dev", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_i32(h, key, value);
        nvs_commit(h);
        nvs_close(h);
    }
}

static void clear_iot_credentials_nvs(bool keep_url)
{
    if (g_iot_mutex) xSemaphoreTake(g_iot_mutex, portMAX_DELAY);
    g_iot_device_id[0] = '\0';
    g_iot_device_secret[0] = '\0';
    g_iot_device_name[0] = '\0';
    g_iot_pairing_code[0] = '\0';
    g_iot_claim_token[0] = '\0';
    g_iot_assigned_event_id[0] = '\0';
    g_iot_event_code[0] = '\0';
    g_iot_event_name[0] = '\0';
    g_iot_config_version = 0;
    g_iot_state = IOT_STATE_UNPAIRED;
    g_iot_pending_auto_sync = false;
    g_iot_last_err[0] = '\0';
    if (g_iot_mutex) xSemaphoreGive(g_iot_mutex);

    save_iot_nvs_str("dev_id", "");
    save_iot_nvs_str("dev_secret", "");
    save_iot_nvs_str("dev_name", "");
    save_iot_nvs_str("evt_id", "");
    save_iot_nvs_str("evt_code", "");
    save_iot_nvs_str("evt_name", "");
    save_iot_nvs_i32("cfg_ver", 0);
    save_iot_nvs_i32("pend_auto", 0);
    /* Luôn giữ nguyên g_iot_base_url và NVS iot_url để người dùng cấu hình 1 lần dùng mãi mãi */
    (void)keep_url;
    g_iot_force_sync = true;
}

static void load_iot_config_nvs(void)
{
    nvs_handle_t h;
    if (nvs_open("iot_dev", NVS_READONLY, &h) == ESP_OK) {
        size_t len = sizeof(g_iot_device_id);
        nvs_get_str(h, "dev_id", g_iot_device_id, &len);
        len = sizeof(g_iot_device_secret);
        nvs_get_str(h, "dev_secret", g_iot_device_secret, &len);
        len = sizeof(g_iot_device_name);
        nvs_get_str(h, "dev_name", g_iot_device_name, &len);
        len = sizeof(g_iot_assigned_event_id);
        nvs_get_str(h, "evt_id", g_iot_assigned_event_id, &len);
        len = sizeof(g_iot_event_code);
        nvs_get_str(h, "evt_code", g_iot_event_code, &len);
        len = sizeof(g_iot_event_name);
        nvs_get_str(h, "evt_name", g_iot_event_name, &len);

        char custom_url[192] = "";
        len = sizeof(custom_url);
        if (nvs_get_str(h, "iot_url", custom_url, &len) == ESP_OK &&
            custom_url[0] != '\0' && strstr(custom_url, "example.com") == NULL) {
            snprintf(g_iot_base_url, sizeof(g_iot_base_url), "%s", custom_url);
            ESP_LOGI(TAG, "NVS: Loaded persistent IoT Base URL: %s", g_iot_base_url);
        } else {
            ESP_LOGW(TAG, "NVS: No custom IoT URL in NVS, using default: %s", g_iot_base_url);
        }

        int32_t ver = 0;
        if (nvs_get_i32(h, "cfg_ver", &ver) == ESP_OK) {
            g_iot_config_version = (int)ver;
        }
        int32_t pend = 0;
        if (nvs_get_i32(h, "pend_auto", &pend) == ESP_OK) {
            g_iot_pending_auto_sync = (pend != 0);
        }
        nvs_close(h);
    }

    g_iot_state = (g_iot_device_id[0] != '\0' && g_iot_device_secret[0] != '\0')
                  ? IOT_STATE_ACTIVE
                  : IOT_STATE_UNPAIRED;
}

static bool is_pc_usb_active(void)
{
    uint32_t last = g_pc_usb_last_tick;
    if (last == 0) return false;
    return (xTaskGetTickCount() - last) < pdMS_TO_TICKS(6000);
}

static void sync_oled_network_state(void)
{
    oled_update_network(g_wifi_connected, g_wifi_ssid, g_wifi_ip, g_wifi_rssi,
                        g_auto_confirm, is_pc_usb_active(),
                        is_iot_device_paired_active());

    oled_iot_status_t o_iot = OLED_IOT_NONE;
    if (g_iot_state == IOT_STATE_PENDING) {
        o_iot = OLED_IOT_PENDING;
    } else if (g_iot_state == IOT_STATE_REPAIR_REQUIRED) {
        o_iot = OLED_IOT_REPAIR_REQUIRED;
    } else if (g_iot_state == IOT_STATE_DISABLED) {
        o_iot = OLED_IOT_DISABLED;
    } else if (g_iot_state == IOT_STATE_ACTIVE) {
        o_iot = (g_iot_assigned_event_id[0] == '\0') ? OLED_IOT_NO_EVENT : OLED_IOT_READY;
    } else {
        o_iot = OLED_IOT_UNPAIRED;
    }

    oled_update_iot_status(o_iot, g_iot_pairing_code, g_iot_event_code, g_device_mac_short);
}

static void save_nvs_u8(const char *key, uint8_t value);

static void mark_local_auto_confirm_changed(bool enabled)
{
    g_auto_confirm = enabled;
    save_nvs_u8("auto_confirm", enabled ? 1 : 0);
    if (is_iot_device_paired_active()) {
        if (g_iot_mutex) xSemaphoreTake(g_iot_mutex, portMAX_DELAY);
        g_iot_pending_auto_sync = true;
        if (g_iot_mutex) xSemaphoreGive(g_iot_mutex);
        save_iot_nvs_i32("pend_auto", 1);
        g_iot_force_sync = true;
    }
    sync_oled_network_state();
}

/* Thread-safe UART output & HTTPS Keep-Alive client state */
static SemaphoreHandle_t g_uart_mutex = NULL;
static SemaphoreHandle_t g_https_mutex = NULL;
static esp_http_client_handle_t s_api_client = NULL;
static bool s_api_client_connected = false;
static volatile bool g_https_in_progress = false;
static volatile bool g_https_prewarming = false;
static esp_timer_handle_t s_buzzer_timer = NULL;

static void trigger_https_prewarm(void);

static void safe_uart_write(const void *data, size_t size)
{
    if (g_uart_mutex) {
        xSemaphoreTake(g_uart_mutex, portMAX_DELAY);
    }
    uart_write_bytes(UART_PORT_NUM, data, size);
    if (g_uart_mutex) {
        xSemaphoreGive(g_uart_mutex);
    }
}

static void safe_uart_write_string(const char *str)
{
    if (str && strlen(str) > 0) {
        safe_uart_write(str, strlen(str));
    }
}

/* Chuyển hướng toàn bộ ESP_LOGI/W/E qua g_uart_mutex để tuyệt đối không xen ngang làm đứt JSON CHECKINRESP hoặc frame camera */
static int safe_uart_log_vprintf(const char *fmt, va_list args)
{
    if (xPortInIsrContext()) {
        return 0;
    }
    char log_buf[256];
    int len = vsnprintf(log_buf, sizeof(log_buf), fmt, args);
    if (len <= 0) return 0;
    if (len >= (int)sizeof(log_buf)) {
        len = (int)sizeof(log_buf) - 1;
    }
    if (g_uart_mutex) {
        if (xSemaphoreTake(g_uart_mutex, pdMS_TO_TICKS(30)) == pdTRUE) {
            uart_write_bytes(UART_PORT_NUM, log_buf, len);
            xSemaphoreGive(g_uart_mutex);
        }
    } else {
        uart_write_bytes(UART_PORT_NUM, log_buf, len);
    }
    return len;
}

/* ============================================================================
 * 2. ĐIỀU KHIỂN THIẾT BỊ NGOẠI VI (BUZZER & LED PWM)
 * ============================================================================ */
static void buzzer_off_cb(void *arg)
{
    (void)arg;
    gpio_set_level(BUZZER_PIN, 0);
}

static void init_buzzer(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BUZZER_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(BUZZER_PIN, 0);

    const esp_timer_create_args_t timer_args = {
        .callback = &buzzer_off_cb,
        .name = "buzzer_off",
    };
    if (esp_timer_create(&timer_args, &s_buzzer_timer) != ESP_OK) {
        s_buzzer_timer = NULL;
    }
    ESP_LOGI(TAG, "Buzzer initialized on GPIO %d", BUZZER_PIN);
}

static void buzzer_beep(uint32_t duration_ms)
{
    if (duration_ms == 0) duration_ms = 90;
    if (s_buzzer_timer) {
        esp_timer_stop(s_buzzer_timer);
        gpio_set_level(BUZZER_PIN, 1);
        esp_timer_start_once(s_buzzer_timer, (uint64_t)duration_ms * 1000ULL);
    } else {
        gpio_set_level(BUZZER_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(duration_ms));
        gpio_set_level(BUZZER_PIN, 0);
    }
}

static void init_led_pwm(void)
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_FLASH_MODE,
        .timer_num        = LEDC_FLASH_TIMER,
        .duty_resolution  = LEDC_TIMER_8_BIT,
        .freq_hz          = 5000,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_FLASH_MODE,
        .channel        = LEDC_FLASH_CHANNEL,
        .timer_sel      = LEDC_FLASH_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = FLASH_LED_PIN,
        .duty           = 0,
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel);
    ESP_LOGI(TAG, "Auxiliary LED PWM initialized on GPIO %d", FLASH_LED_PIN);
}

static void set_led_duty(uint32_t duty)
{
    if (duty > 255) duty = 255;
    ledc_set_duty(LEDC_FLASH_MODE, LEDC_FLASH_CHANNEL, duty);
    ledc_update_duty(LEDC_FLASH_MODE, LEDC_FLASH_CHANNEL);
}

/* ============================================================================
 * 2.1. ĐIỀU KHIỂN SERVO CỔNG CHECK-IN (SG90 / MG90S BARRIER GATE)
 * ============================================================================ */
static TaskHandle_t s_servo_task_handle = NULL;
static uint32_t s_servo_current_angle = SERVO_GATE_CLOSE_DEGREE;

static void servo_set_angle(uint32_t degree)
{
    if (degree > 180) degree = 180;
    s_servo_current_angle = degree;
    uint32_t pulse_us = SERVO_MIN_PULSEWIDTH_US + (degree * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / 180);
    /* Độ phân giải 14-bit: 2^14 = 16384 ticks cho chu kỳ 20,000 us (50Hz) */
    uint32_t duty = (pulse_us * ((1 << 14) - 1)) / 20000;
    ledc_set_duty(LEDC_SERVO_MODE, LEDC_SERVO_CHANNEL, duty);
    ledc_update_duty(LEDC_SERVO_MODE, LEDC_SERVO_CHANNEL);
}

void servo_trigger_gate_open(void)
{
    if (s_servo_task_handle) {
        xTaskNotifyGive(s_servo_task_handle);
    }
}

static void init_servo_pwm(void)
{
    ledc_timer_config_t timer_conf = {
        .speed_mode       = LEDC_SERVO_MODE,
        .timer_num        = LEDC_SERVO_TIMER,
        .duty_resolution  = LEDC_SERVO_DUTY_RES,
        .freq_hz          = LEDC_SERVO_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_conf);

    /* Khởi tạo ở 0 độ (hạ cần / đóng cổng) */
    uint32_t init_duty = (SERVO_MIN_PULSEWIDTH_US * ((1 << 14) - 1)) / 20000;
    ledc_channel_config_t ch_conf = {
        .speed_mode     = LEDC_SERVO_MODE,
        .channel        = LEDC_SERVO_CHANNEL,
        .timer_sel      = LEDC_SERVO_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = SERVO_PIN,
        .duty           = init_duty,
        .hpoint         = 0
    };
    ledc_channel_config(&ch_conf);
    ESP_LOGI(TAG, "Servo Gate PWM initialized on GPIO %d (50Hz, 14-bit, Timer %d, Channel %d)",
             SERVO_PIN, LEDC_SERVO_TIMER, LEDC_SERVO_CHANNEL);
}

/* Khởi tạo cảm biến hồng ngoại phát hiện người đi qua cổng (IR Obstacle Sensor) */
static void init_ir_sensor(void)
{
    gpio_reset_pin(IR_SENSOR_PIN);
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << IR_SENSOR_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    ESP_LOGI(TAG, "IR Sensor initialized on GPIO %d (Active-LOW when obstacle detected)", IR_SENSOR_PIN);
}

/* Đọc trạng thái cảm biến IR (true = có người/vật cản chặn tia hồng ngoại) */
static bool is_ir_obstacle_detected(void)
{
    return gpio_get_level(IR_SENSOR_PIN) == 0;
}

static void servo_gate_task(void *pvParameters)
{
    (void)pvParameters;
    servo_set_angle(SERVO_GATE_CLOSE_DEGREE);

    while (1) {
        /* Chờ tín hiệu mở cổng khi vé check-in hợp lệ */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        ESP_LOGI(TAG, "Servo Gate: VALID TICKET -> OPENING GATE (%d deg)...", SERVO_GATE_OPEN_DEGREE);
        servo_set_angle(SERVO_GATE_OPEN_DEGREE);
        /* Chờ 1 giây để cần servo nâng hoàn toàn lên vị trí 90 độ trước khi giám sát cảm biến */
        vTaskDelay(pdMS_TO_TICKS(1000));

        /* Trạng thái thông minh với cảm biến hồng ngoại:
         * 1. Cổng mở góc 90 độ.
         * 2. Chờ người bước vào cổng (cảm biến IR phát hiện có người = LOW).
         * 3. Khi người đã bước qua hẳn khỏi cổng (cảm biến IR quay lại HIGH) -> Đóng cổng!
         * 4. Kèm timeout an toàn (SERVO_GATE_SAFETY_TIMEOUT_MS) nếu người đổi ý không bước vào. */
        TickType_t start_tick = xTaskGetTickCount();
        bool person_entered = false;

        while (1) {
            /* Nếu có vé khác được check-in liên tiếp, reset lại timer an toàn */
            if (ulTaskNotifyTake(pdTRUE, 0) > 0) {
                ESP_LOGI(TAG, "Servo Gate: Additional valid ticket scanned, extending gate open...");
                start_tick = xTaskGetTickCount();
            }

            bool obstacle = is_ir_obstacle_detected();

            if (obstacle) {
                if (!person_entered) {
                    ESP_LOGI(TAG, "Servo Gate: IR Sensor -> Person ENTERED the gate zone.");
                    person_entered = true;
                }
            } else {
                if (person_entered) {
                    /* Người đã đi qua hẳn khỏi cảm biến hồng ngoại! Chờ 400ms an toàn rồi đóng cổng */
                    vTaskDelay(pdMS_TO_TICKS(400));
                    ESP_LOGI(TAG, "Servo Gate: IR Sensor -> Person PASSED completely. CLOSING GATE (%d deg)...", SERVO_GATE_CLOSE_DEGREE);
                    break;
                }
            }

            /* Cơ chế timeout an toàn nếu không ai đi qua trong 15 giây */
            if ((xTaskGetTickCount() - start_tick) > pdMS_TO_TICKS(SERVO_GATE_SAFETY_TIMEOUT_MS)) {
                ESP_LOGW(TAG, "Servo Gate: Safety timeout (15s) reached. CLOSING GATE (%d deg)...", SERVO_GATE_CLOSE_DEGREE);
                break;
            }

            vTaskDelay(pdMS_TO_TICKS(50));
        }

        servo_set_angle(SERVO_GATE_CLOSE_DEGREE);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/* ============================================================================
 * 2.2. ĐÈN LED BÁO TRẠNG THÁI SOÁT VÉ (LED XANH LÁ & ĐỎ - SÁNG 1 PHÚT)
 * ============================================================================ */
static TimerHandle_t s_led_green_timer = NULL;
static TimerHandle_t s_led_red_timer = NULL;

static void led_green_timer_cb(TimerHandle_t xTimer)
{
    (void)xTimer;
    gpio_set_level(LED_GREEN_PIN, 0);
    ESP_LOGI(TAG, "Status LED: Green LED turned OFF after 60s timeout");
}

static void led_red_timer_cb(TimerHandle_t xTimer)
{
    (void)xTimer;
    gpio_set_level(LED_RED_PIN, 0);
    ESP_LOGI(TAG, "Status LED: Red LED turned OFF after 60s timeout");
}

static void led_turn_off_all(void)
{
    if (s_led_green_timer) xTimerStop(s_led_green_timer, 0);
    if (s_led_red_timer) xTimerStop(s_led_red_timer, 0);
    gpio_set_level(LED_GREEN_PIN, 0);
    gpio_set_level(LED_RED_PIN, 0);
}

void led_set_green_direct(int level)
{
    if (s_led_green_timer) xTimerStop(s_led_green_timer, 0);
    gpio_set_level(LED_GREEN_PIN, level ? 1 : 0);
    ESP_LOGI(TAG, "Status LED: Green direct level = %d (GPIO %d)", level ? 1 : 0, LED_GREEN_PIN);
}

void led_set_red_direct(int level)
{
    if (s_led_red_timer) xTimerStop(s_led_red_timer, 0);
    gpio_set_level(LED_RED_PIN, level ? 1 : 0);
    ESP_LOGI(TAG, "Status LED: Red direct level = %d (GPIO %d)", level ? 1 : 0, LED_RED_PIN);
}

void led_trigger_green_ok(void)
{
    /* Tắt đèn đỏ nếu đang sáng */
    if (s_led_red_timer) xTimerStop(s_led_red_timer, 0);
    gpio_set_level(LED_RED_PIN, 0);

    /* Bật đèn xanh lá và kích hoạt/gia hạn bộ đếm 1 phút (60s) */
    gpio_set_level(LED_GREEN_PIN, 1);
    if (s_led_green_timer) {
        xTimerReset(s_led_green_timer, 0);
    }
    ESP_LOGI(TAG, "Status LED: GREEN ON for 60s (Ticket OK)");
}

void led_trigger_red_error(void)
{
    /* Tắt đèn xanh nếu đang sáng */
    if (s_led_green_timer) xTimerStop(s_led_green_timer, 0);
    gpio_set_level(LED_GREEN_PIN, 0);

    /* Bật đèn đỏ và kích hoạt/gia hạn bộ đếm 1 phút (60s) */
    gpio_set_level(LED_RED_PIN, 1);
    if (s_led_red_timer) {
        xTimerReset(s_led_red_timer, 0);
    }
    ESP_LOGI(TAG, "Status LED: RED ON for 60s (Ticket Error/Used)");
}

static void hw_test_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Hardware Self-Test: STARTING...");
    safe_uart_write_string("TESTRESP:STARTING\n");

    /* Step 1: Green LED test (GPIO 41) */
    ESP_LOGI(TAG, "Hardware Self-Test: Green LED ON (GPIO %d)...", LED_GREEN_PIN);
    gpio_set_level(LED_GREEN_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(1500));
    gpio_set_level(LED_GREEN_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(300));

    /* Step 2: Red LED test (GPIO 42) */
    ESP_LOGI(TAG, "Hardware Self-Test: Red LED ON (GPIO %d)...", LED_RED_PIN);
    gpio_set_level(LED_RED_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(1500));
    gpio_set_level(LED_RED_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(300));

    /* Step 3: Buzzer test (GPIO 1) */
    ESP_LOGI(TAG, "Hardware Self-Test: Buzzer Beep 2x...");
    buzzer_beep(100);
    vTaskDelay(pdMS_TO_TICKS(180));
    buzzer_beep(100);
    vTaskDelay(pdMS_TO_TICKS(400));

    /* Step 4: Servo Gate test (GPIO 14 - 90 deg) */
    ESP_LOGI(TAG, "Hardware Self-Test: Servo Opening to 90 deg...");
    servo_trigger_gate_open();
    vTaskDelay(pdMS_TO_TICKS(3200));

    /* Step 5: OLED feedback */
    oled_show_ticket_status(OLED_TICKET_OK, "TEST PASS", "PHAN CUNG OK");
    vTaskDelay(pdMS_TO_TICKS(2500));
    oled_show_ticket_status(OLED_TICKET_IDLE, "", NULL);

    ESP_LOGI(TAG, "Hardware Self-Test: COMPLETED SUCCESSFULLY.");
    safe_uart_write_string("TESTRESP:COMPLETED\n");
    vTaskDelete(NULL);
}

void trigger_hardware_selftest(void)
{
    xTaskCreate(hw_test_task, "hw_test_task", 3072, NULL, 5, NULL);
}

static void init_status_leds(void)
{
    /* BẮT BUỘC: Reset GPIO 41 và 42 để tách hoàn toàn khỏi chân MTDI/MTMS của khối phần cứng JTAG.
     * Mặc định trên ESP32-S3, GPIO 41 và 42 được gán vào JTAG TAP Controller.
     * Nếu không gọi gpio_reset_pin(), chân vẫn nằm ở chế độ JTAG nên gpio_set_level không thể điều khiển mức ra
     * và bất kỳ thay đổi điện áp/tải gắn vào chân sẽ gây kích hoạt ngắt debug phần cứng làm treo/đơ chip! */
    gpio_reset_pin(LED_GREEN_PIN);
    gpio_set_direction(LED_GREEN_PIN, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(LED_GREEN_PIN, GPIO_FLOATING);
    gpio_set_drive_capability(LED_GREEN_PIN, GPIO_DRIVE_CAP_3);
    gpio_set_level(LED_GREEN_PIN, 0);

    gpio_reset_pin(LED_RED_PIN);
    gpio_set_direction(LED_RED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(LED_RED_PIN, GPIO_FLOATING);
    gpio_set_drive_capability(LED_RED_PIN, GPIO_DRIVE_CAP_3);
    gpio_set_level(LED_RED_PIN, 0);

    /* Khởi tạo FreeRTOS One-shot Timers (60 giây) */
    if (!s_led_green_timer) {
        s_led_green_timer = xTimerCreate("led_grn_tmr", pdMS_TO_TICKS(STATUS_LED_HOLD_MS),
                                          pdFALSE, NULL, led_green_timer_cb);
    }
    if (!s_led_red_timer) {
        s_led_red_timer   = xTimerCreate("led_red_tmr", pdMS_TO_TICKS(STATUS_LED_HOLD_MS),
                                          pdFALSE, NULL, led_red_timer_cb);
    }

    /* Nháy nhanh 2 LED 120ms để tự kiểm tra phần cứng (Hardware Power-on Self-Test) */
    gpio_set_level(LED_GREEN_PIN, 1);
    gpio_set_level(LED_RED_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(120));
    gpio_set_level(LED_GREEN_PIN, 0);
    gpio_set_level(LED_RED_PIN, 0);

    ESP_LOGI(TAG, "Status LEDs initialized safely on GPIO %d (Green) & GPIO %d (Red) - JTAG detached, 60s auto-off",
             LED_GREEN_PIN, LED_RED_PIN);
}

/* ============================================================================
 * 3. TỰ ĐỘNG KHÔI PHỤC BUS SCCB & TỐI ƯU THANH GHI ISP OV5640
 * ============================================================================ */
static void recover_sccb_bus(void)
{
    ESP_LOGI(TAG, "Releasing & stabilizing SCCB bus (SDA: %d, SCL: %d)...", CAM_PIN_SIOD, CAM_PIN_SIOC);
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << CAM_PIN_SIOD) | (1ULL << CAM_PIN_SIOC),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    gpio_set_level(CAM_PIN_SIOD, 1);
    for (int i = 0; i < 9; i++) {
        gpio_set_level(CAM_PIN_SIOC, 0);
        esp_rom_delay_us(10);
        gpio_set_level(CAM_PIN_SIOC, 1);
        esp_rom_delay_us(10);
    }

    gpio_set_level(CAM_PIN_SIOD, 0);
    esp_rom_delay_us(10);
    gpio_set_level(CAM_PIN_SIOC, 1);
    esp_rom_delay_us(10);
    gpio_set_level(CAM_PIN_SIOD, 1);
    esp_rom_delay_us(25000);
}

static esp_err_t optimizeOV5640Registers(sensor_t *s)
{
    if (!s) return ESP_ERR_INVALID_ARG;
    ESP_LOGI(TAG, ">> Applying OV5640 Hardware ISP registers (Sharpening & AEC) <<");

    s->set_reg(s, 0x5308, 0xFF, 0x65); /* Manual Sharpness */
    s->set_reg(s, 0x5302, 0xFF, 0x3F); /* Max sharpening strength */
    s->set_reg(s, 0x5300, 0xFF, 0x08);
    s->set_reg(s, 0x5301, 0xFF, 0x30);

    s->set_reg(s, 0x3a0f, 0xFF, 0x48);
    s->set_reg(s, 0x3a10, 0xFF, 0x40);
    s->set_reg(s, 0x3a1b, 0xFF, 0x48);
    s->set_reg(s, 0x3a1e, 0xFF, 0x40);
    s->set_reg(s, 0x3a11, 0xFF, 0x90); /* Fast High Zone */
    s->set_reg(s, 0x3a1f, 0xFF, 0x20); /* Fast Low Zone */

    s->set_exposure_ctrl(s, 1);
    s->set_gain_ctrl(s, 1);
    s->set_whitebal(s, 1);
    s->set_aec2(s, 1);
    s->set_ae_level(s, -2);
    s->set_contrast(s, 0);
    s->set_brightness(s, 0);
    s->set_saturation(s, 0);
    s->set_denoise(s, 1);
    s->set_gainceiling(s, GAINCEILING_4X);
    s->set_vflip(s, 1);
    s->set_hmirror(s, 0);

    return ESP_OK;
}

static void fill_camera_base_config(camera_config_t *config, int xclk_hz, pixformat_t fmt, framesize_t fsize)
{
    memset(config, 0, sizeof(camera_config_t));
    config->ledc_channel  = LEDC_CHANNEL_0;
    config->ledc_timer    = LEDC_TIMER_0;
    config->pin_d0        = CAM_PIN_D0;
    config->pin_d1        = CAM_PIN_D1;
    config->pin_d2        = CAM_PIN_D2;
    config->pin_d3        = CAM_PIN_D3;
    config->pin_d4        = CAM_PIN_D4;
    config->pin_d5        = CAM_PIN_D5;
    config->pin_d6        = CAM_PIN_D6;
    config->pin_d7        = CAM_PIN_D7;
    config->pin_xclk      = CAM_PIN_XCLK;
    config->pin_pclk      = CAM_PIN_PCLK;
    config->pin_vsync     = CAM_PIN_VSYNC;
    config->pin_href      = CAM_PIN_HREF;
    config->pin_sccb_sda  = CAM_PIN_SIOD;
    config->pin_sccb_scl  = CAM_PIN_SIOC;
    config->pin_pwdn      = CAM_PIN_PWDN;
    config->pin_reset     = CAM_PIN_RESET;
    config->xclk_freq_hz  = xclk_hz;
    config->sccb_i2c_port = -1;
    config->pixel_format  = fmt;
    config->frame_size    = fsize;
    config->jpeg_quality  = 12;
    config->fb_count      = 2;
    config->fb_location   = CAMERA_FB_IN_PSRAM;
    config->grab_mode     = CAMERA_GRAB_LATEST;
}

static bool g_camera_sw_jpeg_mode = false;

static void probe_gpio_electrical_state(const char *label, int gpio_num)
{
    if (gpio_num < 0) return;
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << gpio_num),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io);
    esp_rom_delay_us(200);
    int lv_pu = gpio_get_level(gpio_num);

    io.pull_up_en = GPIO_PULLUP_DISABLE;
    io.pull_down_en = GPIO_PULLDOWN_ENABLE;
    gpio_config(&io);
    esp_rom_delay_us(200);
    int lv_pd = gpio_get_level(gpio_num);

    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io);

    const char *verdict = (lv_pu == 0 && lv_pd == 0) ? "SHORTED_TO_GND_OR_DRIVEN_LOW" :
                          (lv_pu == 1 && lv_pd == 1) ? "SHORTED_TO_3V3_OR_DRIVEN_HIGH" :
                                                       "FLOATING_OR_OPEN_TRACE";
    ESP_LOGW(TAG, "CAM_DIAG_ELEC: %s (GPIO %d) -> PU=%d, PD=%d [%s]", label, gpio_num, lv_pu, lv_pd, verdict);
}

static uint8_t run_camera_data_bus_diag(void)
{
    ESP_LOGW(TAG, "CAM_DIAG: Running GRAYSCALE D0-D7 pin integrity probe...");
    const int cam_data_pins[8] = {
        CAM_PIN_D0, CAM_PIN_D1, CAM_PIN_D2, CAM_PIN_D3,
        CAM_PIN_D4, CAM_PIN_D5, CAM_PIN_D6, CAM_PIN_D7
    };
    uint8_t stuck_mask = 0;

    camera_config_t dcfg;
    fill_camera_base_config(&dcfg, 10000000, PIXFORMAT_GRAYSCALE, FRAMESIZE_QQVGA);
    dcfg.fb_count = 1;
    if (esp_camera_init(&dcfg) == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(150));
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb && fb->len >= 16) {
            uint8_t or_mask = 0x00;
            uint8_t and_mask = 0xFF;
            size_t sample_len = fb->len < 4096 ? fb->len : 4096;
            for (size_t i = 0; i < sample_len; i++) {
                or_mask |= fb->buf[i];
                and_mask &= fb->buf[i];
            }
            ESP_LOGW(TAG, "CAM_DIAG: GRAYSCALE len=%u or_mask=0x%02X and_mask=0x%02X first8=%02X %02X %02X %02X %02X %02X %02X %02X",
                     (unsigned)fb->len, or_mask, and_mask,
                     fb->buf[0], fb->buf[1], fb->buf[2], fb->buf[3],
                     fb->buf[4], fb->buf[5], fb->buf[6], fb->buf[7]);
            for (int b = 0; b < 8; b++) {
                if (((or_mask >> b) & 1) == 0) {
                    ESP_LOGE(TAG, "CAM_DIAG: Data line D%d (GPIO %d) appears STUCK LOW (0)!", b, cam_data_pins[b]);
                    stuck_mask |= (1U << b);
                } else if (((and_mask >> b) & 1) == 1) {
                    ESP_LOGE(TAG, "CAM_DIAG: Data line D%d (GPIO %d) appears STUCK HIGH (1)!", b, cam_data_pins[b]);
                    stuck_mask |= (1U << b);
                }
            }
            esp_camera_fb_return(fb);
        } else {
            ESP_LOGE(TAG, "CAM_DIAG: GRAYSCALE frame capture also timed out (VSYNC/HREF/PCLK issue)!");
            if (fb) esp_camera_fb_return(fb);
        }
        esp_camera_deinit();
    } else {
        ESP_LOGE(TAG, "CAM_DIAG: GRAYSCALE init failed!");
    }

    if (stuck_mask != 0) {
        for (int b = 0; b < 8; b++) {
            if ((stuck_mask >> b) & 1) {
                char lbl[8];
                snprintf(lbl, sizeof(lbl), "D%d", b);
                probe_gpio_electrical_state(lbl, cam_data_pins[b]);
            }
        }
    }
    return stuck_mask;
}

static esp_err_t init_camera(void)
{
    if (!esp_psram_is_initialized()) {
        ESP_LOGE(TAG, "PSRAM is REQUIRED for VGA buffers!");
        return ESP_ERR_NOT_SUPPORTED;
    }

    g_camera_sw_jpeg_mode = false;
    esp_err_t last_err = ESP_FAIL;

    /* Bước 1: Thử Hardware JPEG chuẩn (16MHz trước, rồi 10MHz nếu không lỗi chân dữ liệu) */
    recover_sccb_bus();
    camera_config_t config;
    fill_camera_base_config(&config, 16000000, PIXFORMAT_JPEG, FRAMESIZE_VGA);
    ESP_LOGI(TAG, "Probing camera HW-JPEG @ XCLK=16000000 Hz...");
    esp_err_t err = esp_camera_init(&config);
    if (err == ESP_OK) {
        sensor_t *s = esp_camera_sensor_get();
        if (s) optimizeOV5640Registers(s);
        vTaskDelay(pdMS_TO_TICKS(120));
        camera_fb_t *test_fb = esp_camera_fb_get();
        if (test_fb && test_fb->len >= 4 && test_fb->buf[0] == 0xFF && test_fb->buf[1] == 0xD8) {
            ESP_LOGI(TAG, "Camera HW-JPEG verified OK @ 16MHz (test frame=%u bytes)", (unsigned)test_fb->len);
            esp_camera_fb_return(test_fb);
            return ESP_OK;
        }
        if (test_fb) esp_camera_fb_return(test_fb);
        esp_camera_deinit();
        vTaskDelay(pdMS_TO_TICKS(80));
    } else {
        last_err = err;
        esp_camera_deinit();
    }

    /* Bước 2: Chạy chẩn đoán bus D0-D7 */
    uint8_t stuck_mask = run_camera_data_bus_diag();

    if (stuck_mask == 0) {
        /* Không có chân nào bị đứt/kẹt -> thử HW-JPEG ở 10MHz */
        recover_sccb_bus();
        fill_camera_base_config(&config, 10000000, PIXFORMAT_JPEG, FRAMESIZE_VGA);
        ESP_LOGI(TAG, "No stuck pins detected; retrying HW-JPEG @ XCLK=10000000 Hz...");
        err = esp_camera_init(&config);
        if (err == ESP_OK) {
            sensor_t *s = esp_camera_sensor_get();
            if (s) optimizeOV5640Registers(s);
            vTaskDelay(pdMS_TO_TICKS(120));
            camera_fb_t *test_fb = esp_camera_fb_get();
            if (test_fb && test_fb->len >= 4 && test_fb->buf[0] == 0xFF && test_fb->buf[1] == 0xD8) {
                ESP_LOGI(TAG, "Camera HW-JPEG verified OK @ 10MHz (test frame=%u bytes)", (unsigned)test_fb->len);
                esp_camera_fb_return(test_fb);
                return ESP_OK;
            }
            if (test_fb) esp_camera_fb_return(test_fb);
            esp_camera_deinit();
        }
    }

    /* Bước 3: Tự động cứu hộ sang GRAYSCALE + Software JPEG ưu tiên độ nét cao (VGA 640x480 @ 20MHz XCLK, Q=72) */
    const framesize_t sw_sizes[] = { FRAMESIZE_VGA, FRAMESIZE_HVGA, FRAMESIZE_QVGA };
    const char *sw_names[] = { "VGA 640x480", "HVGA 480x320", "QVGA 320x240" };

    for (size_t i = 0; i < sizeof(sw_sizes) / sizeof(sw_sizes[0]); i++) {
        recover_sccb_bus();
        camera_config_t sw_cfg;
        fill_camera_base_config(&sw_cfg, 20000000, PIXFORMAT_GRAYSCALE, sw_sizes[i]);
        sw_cfg.fb_count = 3;
        ESP_LOGW(TAG, "CAM_RECOVERY: Activating GRAYSCALE + SW-JPEG fallback (%s @ 20MHz)...", sw_names[i]);
        err = esp_camera_init(&sw_cfg);
        if (err != ESP_OK) {
            last_err = err;
            esp_camera_deinit();
            continue;
        }
        sensor_t *s = esp_camera_sensor_get();
        if (s) optimizeOV5640Registers(s);
        vTaskDelay(pdMS_TO_TICKS(150));
        camera_fb_t *test_fb = esp_camera_fb_get();
        if (test_fb && test_fb->len > 1000) {
            uint8_t *jpg_buf = NULL;
            size_t jpg_len = 0;
            bool ok = frame2jpg(test_fb, 72, &jpg_buf, &jpg_len);
            esp_camera_fb_return(test_fb);
            if (ok && jpg_buf && jpg_len > 100) {
                ESP_LOGW(TAG, "CAM_RECOVERY: SUCCESS! GRAYSCALE (%s) -> SW-JPEG (%u bytes) active!",
                         sw_names[i], (unsigned)jpg_len);
                free(jpg_buf);
                g_camera_sw_jpeg_mode = true;
                return ESP_OK;
            }
            if (jpg_buf) free(jpg_buf);
        } else {
            if (test_fb) esp_camera_fb_return(test_fb);
        }
        esp_camera_deinit();
    }

    return last_err;
}

/* ============================================================================
 * 4. QUẢN LÝ WI-FI VÀ NVS TRÊN ESP32
 * ============================================================================ */
static void load_wifi_config(void)
{
    memset(g_wifi_profiles, 0, sizeof(g_wifi_profiles));
    g_wifi_profile_count = 0;

    nvs_handle_t my_handle;
    if (nvs_open("wifi_store", NVS_READONLY, &my_handle) == ESP_OK) {
        size_t ssid_len = sizeof(g_wifi_ssid);
        size_t pass_len = sizeof(g_wifi_pass);
        uint8_t auto_val = 0;
        uint8_t w_cnt = 0;

        if (nvs_get_str(my_handle, "ssid", g_wifi_ssid, &ssid_len) != ESP_OK) g_wifi_ssid[0] = '\0';
        if (nvs_get_str(my_handle, "pass", g_wifi_pass, &pass_len) != ESP_OK) g_wifi_pass[0] = '\0';
        if (nvs_get_u8(my_handle, "auto_confirm", &auto_val) == ESP_OK) {
            g_auto_confirm = (auto_val != 0);
        }
        if (nvs_get_u8(my_handle, "w_cnt", &w_cnt) == ESP_OK) {
            if (w_cnt > MAX_WIFI_PROFILES) w_cnt = MAX_WIFI_PROFILES;
            for (int i = 0; i < (int)w_cnt; i++) {
                char ks[8], kp[8];
                snprintf(ks, sizeof(ks), "w_s%d", i);
                snprintf(kp, sizeof(kp), "w_p%d", i);
                size_t sl = sizeof(g_wifi_profiles[g_wifi_profile_count].ssid);
                size_t pl = sizeof(g_wifi_profiles[g_wifi_profile_count].pass);
                if (nvs_get_str(my_handle, ks, g_wifi_profiles[g_wifi_profile_count].ssid, &sl) == ESP_OK &&
                    g_wifi_profiles[g_wifi_profile_count].ssid[0] != '\0') {
                    if (nvs_get_str(my_handle, kp, g_wifi_profiles[g_wifi_profile_count].pass, &pl) != ESP_OK) {
                        g_wifi_profiles[g_wifi_profile_count].pass[0] = '\0';
                    }
                    g_wifi_profile_count++;
                }
            }
        }
        nvs_close(my_handle);
    }

    if (g_wifi_profile_count == 0 && g_wifi_ssid[0] != '\0') {
        snprintf(g_wifi_profiles[0].ssid, sizeof(g_wifi_profiles[0].ssid), "%s", g_wifi_ssid);
        snprintf(g_wifi_profiles[0].pass, sizeof(g_wifi_profiles[0].pass), "%s", g_wifi_pass);
        g_wifi_profile_count = 1;
    } else if (g_wifi_ssid[0] == '\0' && g_wifi_profile_count > 0) {
        snprintf(g_wifi_ssid, sizeof(g_wifi_ssid), "%s", g_wifi_profiles[0].ssid);
        snprintf(g_wifi_pass, sizeof(g_wifi_pass), "%s", g_wifi_profiles[0].pass);
    }

    ESP_LOGI(TAG, "Loaded NVS config: ssid='%s', profiles=%d, auto_confirm=%d",
             g_wifi_ssid, g_wifi_profile_count, g_auto_confirm ? 1 : 0);
}

static void persist_wifi_profiles_nvs(void)
{
    nvs_handle_t my_handle;
    if (nvs_open("wifi_store", NVS_READWRITE, &my_handle) == ESP_OK) {
        nvs_set_str(my_handle, "ssid", g_wifi_ssid);
        nvs_set_str(my_handle, "pass", g_wifi_pass);
        nvs_set_u8(my_handle, "w_cnt", (uint8_t)g_wifi_profile_count);
        for (int i = 0; i < g_wifi_profile_count && i < MAX_WIFI_PROFILES; i++) {
            char ks[8], kp[8];
            snprintf(ks, sizeof(ks), "w_s%d", i);
            snprintf(kp, sizeof(kp), "w_p%d", i);
            nvs_set_str(my_handle, ks, g_wifi_profiles[i].ssid);
            nvs_set_str(my_handle, kp, g_wifi_profiles[i].pass);
        }
        nvs_commit(my_handle);
        nvs_close(my_handle);
    }
}

static void add_or_promote_wifi_profile(const char *ssid, const char *pass, bool promote_front)
{
    if (!ssid || ssid[0] == '\0') return;
    wifi_profile_t new_p = {0};
    snprintf(new_p.ssid, sizeof(new_p.ssid), "%s", ssid);
    snprintf(new_p.pass, sizeof(new_p.pass), "%s", pass ? pass : "");

    int existing = -1;
    for (int i = 0; i < g_wifi_profile_count; i++) {
        if (strcmp(g_wifi_profiles[i].ssid, new_p.ssid) == 0) {
            existing = i;
            break;
        }
    }

    if (promote_front) {
        if (existing > 0) {
            for (int i = existing; i > 0; i--) {
                g_wifi_profiles[i] = g_wifi_profiles[i - 1];
            }
            g_wifi_profiles[0] = new_p;
        } else if (existing == 0) {
            g_wifi_profiles[0] = new_p;
        } else {
            int limit = (g_wifi_profile_count < MAX_WIFI_PROFILES) ? g_wifi_profile_count : (MAX_WIFI_PROFILES - 1);
            for (int i = limit; i > 0; i--) {
                g_wifi_profiles[i] = g_wifi_profiles[i - 1];
            }
            g_wifi_profiles[0] = new_p;
            if (g_wifi_profile_count < MAX_WIFI_PROFILES) g_wifi_profile_count++;
        }
        snprintf(g_wifi_ssid, sizeof(g_wifi_ssid), "%s", new_p.ssid);
        snprintf(g_wifi_pass, sizeof(g_wifi_pass), "%s", new_p.pass);
    } else {
        if (existing >= 0) {
            g_wifi_profiles[existing] = new_p;
        } else if (g_wifi_profile_count < MAX_WIFI_PROFILES) {
            g_wifi_profiles[g_wifi_profile_count++] = new_p;
        } else {
            /* Giữ slot 0 (mạng ưu tiên), thay thế slot cũ nhất ở cuối */
            g_wifi_profiles[MAX_WIFI_PROFILES - 1] = new_p;
        }
        if (g_wifi_ssid[0] == '\0') {
            snprintf(g_wifi_ssid, sizeof(g_wifi_ssid), "%s", new_p.ssid);
            snprintf(g_wifi_pass, sizeof(g_wifi_pass), "%s", new_p.pass);
        }
    }

    persist_wifi_profiles_nvs();
}

static void save_wifi_config(const char *ssid, const char *pass)
{
    add_or_promote_wifi_profile(ssid, pass, true);
}

static void save_nvs_u8(const char *key, uint8_t value)
{
    nvs_handle_t my_handle;
    if (nvs_open("wifi_store", NVS_READWRITE, &my_handle) == ESP_OK) {
        nvs_set_u8(my_handle, key, value);
        nvs_commit(my_handle);
        nvs_close(my_handle);
    }
}

static void start_standalone_webserver(void);
static int g_wifi_retry_count = 0;
static bool g_wifi_user_connecting = false;
static bool g_wifi_is_reconfiguring = false;

static void fill_sta_wifi_config(wifi_config_t *wifi_config, const char *ssid, const char *pass)
{
    memset(wifi_config, 0, sizeof(wifi_config_t));
    if (ssid && ssid[0] != '\0') {
        size_t slen = strlen(ssid);
        if (slen >= sizeof(wifi_config->sta.ssid)) slen = sizeof(wifi_config->sta.ssid) - 1;
        memcpy(wifi_config->sta.ssid, ssid, slen);
        wifi_config->sta.ssid[slen] = '\0';

        if (pass && pass[0] != '\0') {
            size_t plen = strlen(pass);
            if (plen >= sizeof(wifi_config->sta.password)) plen = sizeof(wifi_config->sta.password) - 1;
            memcpy(wifi_config->sta.password, pass, plen);
            wifi_config->sta.password[plen] = '\0';
        }
        wifi_config->sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
        wifi_config->sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
        wifi_config->sta.threshold.rssi = -127;
        wifi_config->sta.threshold.authmode = WIFI_AUTH_OPEN;
        wifi_config->sta.pmf_cfg.capable = true;
        wifi_config->sta.pmf_cfg.required = false;
    }
}

static esp_err_t trigger_sta_connect(void)
{
    if (g_wifi_ssid[0] == '\0') return ESP_ERR_INVALID_STATE;
    g_wifi_connecting_now = true;
    g_wifi_last_attempt_tick = xTaskGetTickCount();
    esp_err_t err = esp_wifi_connect();
    if (err != ESP_OK && err != ESP_ERR_WIFI_CONN) {
        g_wifi_connecting_now = false;
    }
    return err;
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        if (strlen(g_wifi_ssid) > 0) {
            trigger_sta_connect();
        }
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t* disconn = (wifi_event_sta_disconnected_t*) event_data;
        uint8_t reason = disconn ? disconn->reason : 0;
        g_wifi_connected = false;
        g_wifi_connecting_now = false;
        strcpy(g_wifi_ip, "0.0.0.0");
        g_wifi_rssi = 0;
        sync_oled_network_state();

        // Bỏ qua nếu đang trong quá trình chuyển đổi cấu hình Wi-Fi mới hoặc ngắt chủ động
        if (g_wifi_is_reconfiguring || reason == 8 || reason == 3 || reason == 36) {
            ESP_LOGI(TAG, "Wi-Fi disconnected intentionally by app (reconfiguring=%d, reason=%d).", g_wifi_is_reconfiguring, reason);
            return;
        }

        const char *reason_str = "CONNECTION_FAIL";
        if (reason == 15 || reason == 202 || reason == 204 || reason == 14 || reason == 23 || reason == 6) {
            reason_str = "WRONG_PASSWORD";
        } else if (reason == 201 || reason == 210 || reason == 211) {
            reason_str = "NO_AP_FOUND";
        } else if (reason == 203 || reason == 9 || reason == 30) {
            reason_str = "ASSOC_FAIL";
        } else if (reason == 200) {
            reason_str = "BEACON_TIMEOUT";
        } else if (reason == 2) {
            reason_str = "AUTH_EXPIRE";
        }

        ESP_LOGW(TAG, "ESP32 Wi-Fi disconnected (ssid='%s', reason %d: %s)", g_wifi_ssid, reason, reason_str);
        s_api_client_connected = false;

        if (g_wifi_user_connecting) {
            if (strcmp(reason_str, "WRONG_PASSWORD") != 0 && g_wifi_retry_count < 2) {
                g_wifi_retry_count++;
                trigger_sta_connect();
                return;
            }
            char fail_buf[128];
            snprintf(fail_buf, sizeof(fail_buf), "WIFIRESULT:FAILED|%d|%s\n", (int)reason, reason_str);
            safe_uart_write_string(fail_buf);
            g_wifi_user_connecting = false;
            g_wifi_bg_fails++;
        } else {
            if (g_wifi_retry_count < 2 && strlen(g_wifi_ssid) > 0) {
                g_wifi_retry_count++;
                trigger_sta_connect();
            } else {
                /* Nhường cho vòng lặp nền uart_cmd_task tự kết nối lại mỗi 4 giây hoặc chuyển sang Wi-Fi đã lưu khác */
                g_wifi_retry_count = 0;
                g_wifi_bg_fails++;
            }
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        snprintf(g_wifi_ip, sizeof(g_wifi_ip), IPSTR, IP2STR(&event->ip_info.ip));
        g_wifi_connected = true;
        g_wifi_connecting_now = false;
        g_wifi_retry_count = 0;
        g_wifi_bg_fails = 0;
        g_wifi_user_connecting = false;
        g_wifi_just_connected_led = true;

        wifi_ap_record_t ap_info;
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            g_wifi_rssi = ap_info.rssi;
        }
        ESP_LOGI(TAG, "=================================================");
        ESP_LOGI(TAG, ">>> ESP32 Wi-Fi CONNECTED! IP: %s (RSSI: %d) <<<", g_wifi_ip, g_wifi_rssi);
        ESP_LOGI(TAG, "=================================================");

        // Lưu cấu hình vào NVS & đẩy lên đầu danh sách ưu tiên
        save_wifi_config(g_wifi_ssid, g_wifi_pass);

        buzzer_beep(120);
        esp_wifi_set_ps(WIFI_PS_NONE);
        start_standalone_webserver();
        trigger_https_prewarm();
        sync_oled_network_state();

        // Báo kết quả thành công qua UART
        char succ_buf[128];
        snprintf(succ_buf, sizeof(succ_buf), "WIFIRESULT:SUCCESS|%s|%d|%s\n", g_wifi_ip, (int)g_wifi_rssi, g_wifi_ssid);
        safe_uart_write_string(succ_buf);
    }
}

static void init_wifi_sta(void)
{
    load_wifi_config();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config;
    fill_sta_wifi_config(&wifi_config, g_wifi_ssid, g_wifi_pass);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    if (strlen(g_wifi_ssid) > 0) {
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    }
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_LOGI(TAG, "Wi-Fi STA initialized (WIFI_PS_NONE). Stored SSID: '%s' (%d profiles)", g_wifi_ssid, g_wifi_profile_count);
}

static void connect_to_wifi_internal(const char *ssid, const char *pass, bool user_initiated)
{
    g_wifi_is_reconfiguring = true;
    g_wifi_user_connecting = user_initiated;
    g_wifi_retry_count = 0;

    snprintf(g_wifi_ssid, sizeof(g_wifi_ssid), "%s", ssid ? ssid : "");
    snprintf(g_wifi_pass, sizeof(g_wifi_pass), "%s", pass ? pass : "");

    wifi_config_t wifi_config;
    fill_sta_wifi_config(&wifi_config, g_wifi_ssid, g_wifi_pass);

    // Đảm bảo dừng scan nếu đang scan dở
    esp_wifi_scan_stop();
    esp_wifi_disconnect();
    vTaskDelay(pdMS_TO_TICKS(150));

    esp_err_t err = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if (err == ESP_ERR_WIFI_STATE) {
        ESP_LOGW(TAG, "Wi-Fi in busy state, stopping and restarting Wi-Fi driver...");
        esp_wifi_stop();
        vTaskDelay(pdMS_TO_TICKS(100));
        err = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
        esp_wifi_start();
        esp_wifi_set_ps(WIFI_PS_NONE);
        g_wifi_is_reconfiguring = false;
        trigger_sta_connect();
        return;
    }

    g_wifi_is_reconfiguring = false;

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_config failed: %s", esp_err_to_name(err));
        if (user_initiated) {
            char rep[128];
            snprintf(rep, sizeof(rep), "WIFIRESULT:FAILED|%d|CONFIG_ERROR\n", (int)err);
            safe_uart_write_string(rep);
        }
        g_wifi_user_connecting = false;
        return;
    }

    err = trigger_sta_connect();
    if (err != ESP_OK && err != ESP_ERR_WIFI_CONN) {
        ESP_LOGE(TAG, "esp_wifi_connect failed: %s", esp_err_to_name(err));
        if (user_initiated) {
            char rep[128];
            snprintf(rep, sizeof(rep), "WIFIRESULT:FAILED|%d|CONNECT_CALL_FAILED\n", (int)err);
            safe_uart_write_string(rep);
        }
        g_wifi_user_connecting = false;
        return;
    }

    ESP_LOGI(TAG, "Connecting to Wi-Fi: '%s' (user=%d)...", g_wifi_ssid, user_initiated ? 1 : 0);
}

static void connect_to_new_wifi(const char *ssid, const char *pass)
{
    connect_to_wifi_internal(ssid, pass, true);
}

/* ============================================================================
 * 5. GỬI REQUEST HTTPS TRỰC TIẾP TỪ ESP32 TỚI EVENTHUB (VERIFY & CHECK-IN)
 * ============================================================================ */
typedef struct {
    char *buf;
    size_t buf_max;
    size_t len;
    esp_err_t tls_last_err;
    int tls_err_code;
    int tls_flags;
} https_resp_ctx_t;

static esp_err_t esp32_https_event_handler(esp_http_client_event_t *evt)
{
    https_resp_ctx_t *ctx = (https_resp_ctx_t *)evt->user_data;
    if (!ctx) return ESP_OK;

    switch (evt->event_id) {
        case HTTP_EVENT_ERROR:
            if (evt->data) {
                esp_tls_last_error_t *err_info = (esp_tls_last_error_t *)evt->data;
                ctx->tls_last_err = err_info->last_error;
                ctx->tls_err_code = err_info->esp_tls_error_code;
                ctx->tls_flags = err_info->esp_tls_flags;
            }
            break;
        case HTTP_EVENT_ON_DATA:
            if (evt->data && evt->data_len > 0 && ctx->buf && ctx->buf_max > 1) {
                size_t avail = (ctx->buf_max - 1) - ctx->len;
                size_t to_copy = (size_t)evt->data_len < avail ? (size_t)evt->data_len : avail;
                if (to_copy > 0) {
                    memcpy(ctx->buf + ctx->len, evt->data, to_copy);
                    ctx->len += to_copy;
                    ctx->buf[ctx->len] = '\0';
                }
            }
            break;
        default:
            break;
    }
    return ESP_OK;
}

static void json_escape_into(const char *src, char *dst, size_t dst_max)
{
    size_t j = 0;
    if (!dst || dst_max == 0) return;
    if (!src) {
        dst[0] = '\0';
        return;
    }
    for (size_t i = 0; src[i] != '\0' && j + 2 < dst_max; i++) {
        unsigned char c = (unsigned char)src[i];
        if (c == '\"' || c == '\\') {
            dst[j++] = '\\';
            dst[j++] = (char)c;
        } else if (c >= 0x20) {
            dst[j++] = (char)c;
        }
    }
    dst[j] = '\0';
}

static https_resp_ctx_t s_https_ctx = {0};

static void reset_api_client_locked(void)
{
    if (s_api_client) {
        esp_http_client_close(s_api_client);
    }
    s_api_client_connected = false;
}

static char s_api_client_origin[96] = "";

static void extract_url_origin(const char *url, char *out, size_t out_max)
{
    if (!out || out_max == 0) return;
    out[0] = '\0';
    if (!url) return;
    const char *scheme = strstr(url, "://");
    const char *host_start = scheme ? (scheme + 3) : url;
    const char *path_start = strchr(host_start, '/');
    size_t len = path_start ? (size_t)(path_start - url) : strlen(url);
    if (len >= out_max) len = out_max - 1;
    memcpy(out, url, len);
    out[len] = '\0';
}

static esp_err_t ensure_api_client_locked(const char *url)
{
    if (!url || (strncmp(url, "http://", 7) != 0 && strncmp(url, "https://", 8) != 0)) {
        ESP_LOGE(TAG, "Invalid HTTP/HTTPS URL passed to client: '%s'", url ? url : "NULL");
        return ESP_ERR_INVALID_ARG;
    }

    char origin[96];
    extract_url_origin(url, origin, sizeof(origin));

    if (s_api_client && strcmp(s_api_client_origin, origin) != 0) {
        esp_http_client_cleanup(s_api_client);
        s_api_client = NULL;
        s_api_client_connected = false;
        s_api_client_origin[0] = '\0';
    }

    if (!s_api_client) {
        esp_http_client_config_t config = {
            .url = url,
            .crt_bundle_attach = esp_crt_bundle_attach,
            .addr_type = HTTP_ADDR_TYPE_INET,
            .timeout_ms = 8000,
            .method = HTTP_METHOD_POST,
            .keep_alive_enable = true,
            .buffer_size = 4096,
            .buffer_size_tx = 2048,
            .event_handler = esp32_https_event_handler,
            .user_data = &s_https_ctx,
        };
        s_api_client = esp_http_client_init(&config);
        if (!s_api_client) {
            return ESP_FAIL;
        }
        esp_http_client_set_header(s_api_client, "User-Agent", "Mozilla/5.0 EventHubStation-ESP32/1.1");
        esp_http_client_set_header(s_api_client, "Connection", "keep-alive");
        snprintf(s_api_client_origin, sizeof(s_api_client_origin), "%s", origin);
        s_api_client_connected = false;
    } else {
        esp_err_t uerr = esp_http_client_set_url(s_api_client, url);
        if (uerr != ESP_OK) {
            esp_http_client_cleanup(s_api_client);
            s_api_client = NULL;
            s_api_client_connected = false;
            s_api_client_origin[0] = '\0';
            return ensure_api_client_locked(url);
        }
    }
    return ESP_OK;
}

static void https_prewarm_task(void *pvParameters)
{
    (void)pvParameters;
    vTaskDelay(pdMS_TO_TICKS(150));

    if (!g_wifi_connected) {
        g_https_prewarming = false;
        vTaskDelete(NULL);
        return;
    }

    if (g_https_mutex) {
        xSemaphoreTake(g_https_mutex, portMAX_DELAY);
    }

    g_https_in_progress = true;
    int64_t t0 = esp_timer_get_time();

    reset_api_client_locked();
    if (ensure_api_client_locked(EVENTHUB_PREWARM_URL) == ESP_OK) {
        memset(&s_https_ctx, 0, sizeof(s_https_ctx));
        esp_http_client_set_method(s_api_client, HTTP_METHOD_GET);
        esp_http_client_delete_header(s_api_client, "Authorization");
        esp_http_client_delete_header(s_api_client, "X-Claim-Token");
        esp_http_client_delete_header(s_api_client, "X-Device-Id");
        esp_http_client_delete_header(s_api_client, "X-Device-Secret");
        esp_http_client_set_header(s_api_client, "Accept", "application/json");
        esp_http_client_set_header(s_api_client, "Connection", "keep-alive");

        esp_err_t err = esp_http_client_open(s_api_client, 0);
        if (err == ESP_OK) {
            int64_t content_len = esp_http_client_fetch_headers(s_api_client);
            if (content_len >= 0) {
                char discard_buf[256];
                while (!esp_http_client_is_complete_data_received(s_api_client)) {
                    int r = esp_http_client_read(s_api_client, discard_buf, sizeof(discard_buf));
                    if (r <= 0) break;
                }
                if (esp_http_client_is_complete_data_received(s_api_client)) {
                    s_api_client_connected = true;
                    ESP_LOGI(TAG, "HTTPS Keep-Alive pre-warmed in %lld ms",
                             (long long)((esp_timer_get_time() - t0) / 1000));
                } else {
                    reset_api_client_locked();
                }
            } else {
                reset_api_client_locked();
            }
        } else {
            ESP_LOGW(TAG, "HTTPS pre-warm skipped (%s)", esp_err_to_name(err));
            reset_api_client_locked();
        }
    }

    g_https_in_progress = false;
    if (g_https_mutex) {
        xSemaphoreGive(g_https_mutex);
    }
    g_https_prewarming = false;
    vTaskDelete(NULL);
}

static void trigger_https_prewarm(void)
{
    if (g_https_prewarming) return;
    g_https_prewarming = true;
    if (xTaskCreatePinnedToCore(https_prewarm_task, "https_prewarm", 8192, NULL, 3, NULL, 0) != pdPASS) {
        g_https_prewarming = false;
    }
}

static esp_err_t esp32_http_json_request(esp_http_client_method_t method,
                                         const char *url,
                                         const char *post_body,
                                         const char *bearer_token,
                                         const char *claim_token,
                                         const char *device_id,
                                         const char *device_secret,
                                         char *out_response,
                                         size_t out_max,
                                         int *out_status_code)
{
    if (out_status_code) *out_status_code = -1;
    if (!out_response || out_max == 0 || !url || url[0] == '\0') return ESP_ERR_INVALID_ARG;
    out_response[0] = '\0';

    if (!g_wifi_connected) {
        snprintf(out_response, out_max, "{\"success\":false,\"message\":\"ESP32 Wi-Fi chua ket noi!\"}");
        return ESP_FAIL;
    }

    char *auth_header = NULL;
    if (bearer_token && bearer_token[0] != '\0') {
        auth_header = (char *)malloc(2048);
        if (!auth_header) {
            snprintf(out_response, out_max, "{\"success\":false,\"message\":\"ESP32 het bo nho tam!\"}");
            return ESP_ERR_NO_MEM;
        }
        snprintf(auth_header, 2048, "Bearer %s", bearer_token);
    }

    size_t post_len = (post_body && method == HTTP_METHOD_POST) ? strlen(post_body) : 0;
    int64_t t_start_us = esp_timer_get_time();

    if (g_https_mutex) {
        xSemaphoreTake(g_https_mutex, portMAX_DELAY);
    }
    g_https_in_progress = true;

    esp_err_t err = ESP_FAIL;
    int sock_errno = 0;
    int status_code = -1;

    for (int attempt = 0; attempt < 2; attempt++) {
        bool was_reused = s_api_client_connected;
        if (ensure_api_client_locked(url) != ESP_OK) {
            err = ESP_FAIL;
            snprintf(out_response, out_max, "{\"success\":false,\"message\":\"Khong khoi tao duoc HTTP client!\"}");
            break;
        }

        memset(&s_https_ctx, 0, sizeof(s_https_ctx));
        esp_http_client_set_method(s_api_client, method);
        esp_http_client_set_header(s_api_client, "Accept", "application/json");
        esp_http_client_set_header(s_api_client, "Connection", "keep-alive");
        if (method == HTTP_METHOD_POST) {
            esp_http_client_set_header(s_api_client, "Content-Type", "application/json");
        } else {
            esp_http_client_delete_header(s_api_client, "Content-Type");
        }

        if (auth_header) {
            esp_http_client_set_header(s_api_client, "Authorization", auth_header);
        } else {
            esp_http_client_delete_header(s_api_client, "Authorization");
        }
        if (claim_token && claim_token[0] != '\0') {
            esp_http_client_set_header(s_api_client, "X-Claim-Token", claim_token);
        } else {
            esp_http_client_delete_header(s_api_client, "X-Claim-Token");
        }
        if (device_id && device_id[0] != '\0') {
            esp_http_client_set_header(s_api_client, "X-Device-Id", device_id);
        } else {
            esp_http_client_delete_header(s_api_client, "X-Device-Id");
        }
        if (device_secret && device_secret[0] != '\0') {
            esp_http_client_set_header(s_api_client, "X-Device-Secret", device_secret);
        } else {
            esp_http_client_delete_header(s_api_client, "X-Device-Secret");
        }

        err = esp_http_client_open(s_api_client, (int)post_len);
        if (err == ESP_OK) {
            int written = 0;
            if (post_len > 0) {
                written = esp_http_client_write(s_api_client, post_body, (int)post_len);
            }
            if (post_len > 0 && written < 0) {
                err = ESP_ERR_HTTP_WRITE_DATA;
            } else {
                int64_t content_len = esp_http_client_fetch_headers(s_api_client);
                if (content_len < 0) {
                    err = ESP_ERR_HTTP_FETCH_HEADER;
                } else {
                    status_code = esp_http_client_get_status_code(s_api_client);
                    int read_len = esp_http_client_read_response(s_api_client, out_response, (int)(out_max - 1));
                    if (read_len < 0) read_len = 0;
                    out_response[read_len] = '\0';

                    if (status_code <= 0) {
                        err = ESP_ERR_HTTP_CONNECTION_CLOSED;
                    } else {
                        if (esp_http_client_is_complete_data_received(s_api_client)) {
                            s_api_client_connected = true;
                        } else {
                            reset_api_client_locked();
                        }

                        int64_t dt_ms = (esp_timer_get_time() - t_start_us) / 1000;
                        ESP_LOGI(TAG, "ESP32 HTTP (%s) -> %d (%d bytes, reused=%d, dt=%lld ms)",
                                 url, status_code, read_len, was_reused ? 1 : 0, (long long)dt_ms);

                        for (int i = 0; i < read_len; i++) {
                            if (out_response[i] == '\r' || out_response[i] == '\n') {
                                out_response[i] = ' ';
                            }
                        }

                        if (read_len == 0 || out_response[0] != '{') {
                            snprintf(out_response, out_max,
                                     "{\"success\":false,\"statusCode\":%d,\"message\":\"May chu tra ve HTTP %d\"}",
                                     status_code, status_code);
                        }
                        break;
                    }
                }
            }
        }

        sock_errno = s_api_client ? esp_http_client_get_errno(s_api_client) : 0;
        reset_api_client_locked();
        if (was_reused && attempt == 0) {
            ESP_LOGW(TAG, "Keep-Alive socket closed (%s), reconnecting immediately...", esp_err_to_name(err));
            continue;
        }
        break;
    }

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ESP32 HTTP failed (%s): err=%s tls=%s mbed=-0x%x flags=0x%x errno=%d",
                 url, esp_err_to_name(err), esp_err_to_name(s_https_ctx.tls_last_err),
                 (unsigned)(-s_https_ctx.tls_err_code), (unsigned)s_https_ctx.tls_flags, sock_errno);
        snprintf(out_response, out_max,
                 "{\"success\":false,\"message\":\"Loi ket noi HTTPS tu ESP32: %s (tls=%s, mbed=-0x%x, flags=0x%x, errno=%d)\"}",
                 esp_err_to_name(err), esp_err_to_name(s_https_ctx.tls_last_err),
                 (unsigned)(-s_https_ctx.tls_err_code), (unsigned)s_https_ctx.tls_flags, sock_errno);
    }

    if (out_status_code) *out_status_code = status_code;
    g_https_in_progress = false;
    if (g_https_mutex) {
        xSemaphoreGive(g_https_mutex);
    }

    free(auth_header);
    return err;
}

static bool is_valid_iot_url(const char *url)
{
    if (!url || strlen(url) < 10) return false;
    if (strncmp(url, "http://", 7) != 0 && strncmp(url, "https://", 8) != 0) return false;
    if (strstr(url, "example.com") != NULL) return false;
    return true;
}

static void normalize_iot_url(const char *in_url, char *out_url, size_t out_max)
{
    if (!out_url || out_max == 0) return;
    out_url[0] = '\0';
    if (!in_url) return;

    while (*in_url == ' ' || *in_url == '\t') in_url++;
    char clean[192];
    snprintf(clean, sizeof(clean), "%s", in_url);
    size_t len = strlen(clean);
    while (len > 0 && (clean[len - 1] == ' ' || clean[len - 1] == '/' || clean[len - 1] == '\r' || clean[len - 1] == '\n')) {
        clean[--len] = '\0';
    }

    if (len == 0) return;

    if (strstr(clean, "/iot/v1") != NULL || strstr(clean, "/v1") != NULL) {
        snprintf(out_url, out_max, "%s", clean);
    } else if (len >= 4 && strcmp(clean + len - 4, "/api") == 0) {
        snprintf(out_url, out_max, "%s/iot/v1", clean);
    } else {
        snprintf(out_url, out_max, "%s/api/iot/v1", clean);
    }
}

static void build_iot_url(const char *suffix, char *out_url, size_t out_max)
{
    if (!out_url || out_max == 0) return;
    out_url[0] = '\0';
    if (!is_valid_iot_url(g_iot_base_url)) return;
    size_t blen = strlen(g_iot_base_url);
    while (blen > 0 && g_iot_base_url[blen - 1] == '/') blen--;
    snprintf(out_url, out_max, "%.*s%s", (int)blen, g_iot_base_url, suffix);
}

static esp_err_t esp32_iot_scan_ticket(const char *qr_code, const char *action,
                                       char *out_response, size_t out_max, int *out_status_code)
{
    if (!is_valid_iot_url(g_iot_base_url)) {
        if (out_response && out_max > 0) {
            snprintf(out_response, out_max,
                     "{\"success\":false,\"errorCode\":\"API_URL_NOT_CONFIGURED\",\"statusCode\":400,"
                     "\"message\":\"Chua cau hinh dia chi API Server tren ESP32!\"}");
        }
        if (out_status_code) *out_status_code = 400;
        return ESP_ERR_INVALID_STATE;
    }

    char escaped_qr[384];
    json_escape_into(qr_code, escaped_qr, sizeof(escaped_qr));

    char url[256];
    build_iot_url("/tickets/scan", url, sizeof(url));
    if (url[0] == '\0') {
        if (out_response && out_max > 0) {
            snprintf(out_response, out_max, "{\"success\":false,\"message\":\"URL API khong hop le!\"}");
        }
        if (out_status_code) *out_status_code = 400;
        return ESP_ERR_INVALID_STATE;
    }

    char *post_data = (char *)malloc(512);
    if (!post_data) {
        if (out_response && out_max > 0) {
            snprintf(out_response, out_max, "{\"success\":false,\"message\":\"ESP32 het bo nho tam!\"}");
        }
        return ESP_ERR_NO_MEM;
    }
    snprintf(post_data, 512, "{\"qr_code\":\"%s\",\"action\":\"%s\"}",
             escaped_qr, (action && action[0]) ? action : "auto");

    char dev_id[64], dev_secret[160];
    if (g_iot_mutex) xSemaphoreTake(g_iot_mutex, portMAX_DELAY);
    snprintf(dev_id, sizeof(dev_id), "%s", g_iot_device_id);
    snprintf(dev_secret, sizeof(dev_secret), "%s", g_iot_device_secret);
    if (g_iot_mutex) xSemaphoreGive(g_iot_mutex);

    esp_err_t err = esp32_http_json_request(HTTP_METHOD_POST, url, post_data,
                                            NULL, NULL, dev_id, dev_secret,
                                            out_response, out_max, out_status_code);
    free(post_data);
    return err;
}

/* ============================================================================
 * 5.1. QUẢN LÝ LỊCH SỬ CHECK-IN / QUÉT VÉ THIẾT BỊ (DEVICE HISTORY)
 * ============================================================================ */
static void copy_utf8_ascii_oled(char *dst, size_t dst_max, const char *src)
{
    if (!dst || dst_max == 0) return;
    dst[0] = '\0';
    if (!src) return;

    size_t j = 0;
    const uint8_t *p = (const uint8_t *)src;
    while (*p && j + 1 < dst_max) {
        if (*p < 0x80) {
            dst[j++] = (char)*p++;
        } else {
            uint32_t cp = 0;
            if ((*p & 0xE0) == 0xC0 && p[1]) {
                cp = ((uint32_t)(p[0] & 0x1F) << 6) | (p[1] & 0x3F);
                p += 2;
            } else if ((*p & 0xF0) == 0xE0 && p[1] && p[2]) {
                cp = ((uint32_t)(p[0] & 0x0F) << 12) | ((uint32_t)(p[1] & 0x3F) << 6) | (p[2] & 0x3F);
                p += 3;
            } else {
                p++;
                continue;
            }
            char ascii = '?';
            if ((cp >= 0xC0 && cp <= 0xC5) || cp == 0x102 || (cp >= 0x1EA0 && cp <= 0x1EB6 && (cp & 1) == 0)) ascii = 'A';
            else if ((cp >= 0xE0 && cp <= 0xE5) || cp == 0x103 || (cp >= 0x1EA1 && cp <= 0x1EB7 && (cp & 1) == 1)) ascii = 'a';
            else if (cp == 0x110) ascii = 'D';
            else if (cp == 0x111) ascii = 'd';
            else if ((cp >= 0xC8 && cp <= 0xCB) || (cp >= 0x1EB8 && cp <= 0x1EC6 && (cp & 1) == 0)) ascii = 'E';
            else if ((cp >= 0xE8 && cp <= 0xEB) || (cp >= 0x1EB9 && cp <= 0x1EC7 && (cp & 1) == 1)) ascii = 'e';
            else if ((cp >= 0xCC && cp <= 0xCF) || cp == 0x128 || cp == 0x1ECA || cp == 0x1EC8) ascii = 'I';
            else if ((cp >= 0xEC && cp <= 0xEF) || cp == 0x129 || cp == 0x1ECB || cp == 0x1EC9) ascii = 'i';
            else if ((cp >= 0xD2 && cp <= 0xD5) || cp == 0x1A0 || (cp >= 0x1ECC && cp <= 0x1EE2 && (cp & 1) == 0)) ascii = 'O';
            else if ((cp >= 0xF2 && cp <= 0xF5) || cp == 0x1A1 || (cp >= 0x1ECD && cp <= 0x1EE3 && (cp & 1) == 1)) ascii = 'o';
            else if ((cp >= 0xD9 && cp <= 0xDC) || cp == 0x1AF || (cp >= 0x1EE4 && cp <= 0x1EF0 && (cp & 1) == 0)) ascii = 'U';
            else if ((cp >= 0xF9 && cp <= 0xFC) || cp == 0x1B0 || (cp >= 0x1EE5 && cp <= 0x1EF1 && (cp & 1) == 1)) ascii = 'u';
            else if (cp == 0xDD || cp == 0x1EF2 || cp == 0x1EF4 || cp == 0x1EF6 || cp == 0x1EF8) ascii = 'Y';
            else if (cp == 0xFD || cp == 0x1EF3 || cp == 0x1EF5 || cp == 0x1EF7 || cp == 0x1EF9) ascii = 'y';
            dst[j++] = ascii;
        }
    }
    dst[j] = '\0';
}

static void format_history_short_time(const char *iso_time, char *out_time, size_t out_max)
{
    if (!out_time || out_max == 0) return;
    out_time[0] = '\0';
    if (!iso_time || iso_time[0] == '\0') return;

    /* Parse ISO UTC "2026-11-12T16:50:00.000Z" -> local VN (+7h) "11-12 23:50" */
    int y = 0, mo = 0, da = 0, hh = 0, mm = 0, ss = 0;
    if (sscanf(iso_time, "%d-%d-%dT%d:%d:%d", &y, &mo, &da, &hh, &mm, &ss) >= 5) {
        if (strchr(iso_time, 'Z') != NULL) {
            hh += 7;
            if (hh >= 24) {
                hh -= 24;
                da += 1;
                static const int mdays[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
                int maxd = (mo >= 1 && mo <= 12) ? mdays[mo - 1] : 30;
                if (mo == 2 && ((y % 4 == 0 && y % 100 != 0) || (y % 400 == 0))) maxd = 29;
                if (da > maxd) {
                    da = 1;
                    mo += 1;
                    if (mo > 12) mo = 1;
                }
            }
        }
        snprintf(out_time, out_max, "%02d-%02d %02d:%02d", mo, da, hh, mm);
        return;
    }
    snprintf(out_time, out_max, "%.11s", iso_time);
}

static void push_history_cache_to_oled_locked(void)
{
    oled_history_snapshot_t snap;
    memset(&snap, 0, sizeof(snap));

    snap.page = s_dev_history.page;
    snap.limit = s_dev_history.limit;
    snap.total = s_dev_history.total;
    snap.has_more = s_dev_history.has_more;
    snap.total_scans = s_dev_history.total_scans;
    snap.ok_count = s_dev_history.ok_count;
    snap.verified_count = s_dev_history.verified_count;
    snap.failed_count = s_dev_history.failed_count;
    snprintf(snap.filter, sizeof(snap.filter), "%.15s", s_dev_history.filter[0] ? s_dev_history.filter : "ALL");
    snap.event_only = s_dev_history.event_only;
    snap.offline_cached = s_dev_history.offline_cached;
    snap.loading = s_dev_history.loading;
    snprintf(snap.error_msg, sizeof(snap.error_msg), "%.23s", s_dev_history.last_error);

    int count = s_dev_history.item_count;
    if (count > OLED_HISTORY_MAX_ITEMS) count = OLED_HISTORY_MAX_ITEMS;
    snap.item_count = count;

    for (int i = 0; i < count; i++) {
        copy_utf8_ascii_oled(snap.items[i].ticket_code, sizeof(snap.items[i].ticket_code), s_dev_history.items[i].ticket_code);
        copy_utf8_ascii_oled(snap.items[i].attendee_name, sizeof(snap.items[i].attendee_name), s_dev_history.items[i].attendee_name);
        copy_utf8_ascii_oled(snap.items[i].result, sizeof(snap.items[i].result), s_dev_history.items[i].result);
        copy_utf8_ascii_oled(snap.items[i].message, sizeof(snap.items[i].message), s_dev_history.items[i].message);
        snap.items[i].success = s_dev_history.items[i].success;
        format_history_short_time(s_dev_history.items[i].scanned_at, snap.items[i].scanned_at, sizeof(snap.items[i].scanned_at));
    }

    oled_update_history(&snap);
}

static void record_local_history_scan(const char *ticket_code, const char *result, const char *msg, bool success)
{
    if (g_history_mutex) xSemaphoreTake(g_history_mutex, portMAX_DELAY);

    /* Shift cached items down */
    for (int i = DEVICE_HISTORY_MAX_ITEMS - 1; i > 0; i--) {
        s_dev_history.items[i] = s_dev_history.items[i - 1];
    }

    dev_history_item_t *head = &s_dev_history.items[0];
    memset(head, 0, sizeof(*head));
    snprintf(head->ticket_code, sizeof(head->ticket_code), "%s", ticket_code ? ticket_code : "");
    snprintf(head->result, sizeof(head->result), "%s", result ? result : "OK");
    snprintf(head->message, sizeof(head->message), "%s", msg ? msg : "");
    head->success = success;

    uint32_t up_sec = (uint32_t)(esp_timer_get_time() / 1000000ULL);
    uint32_t h = (up_sec / 3600) % 24;
    uint32_t m = (up_sec / 60) % 60;
    snprintf(head->scanned_at, sizeof(head->scanned_at), "%02lu:%02lu", (unsigned long)h, (unsigned long)m);

    if (s_dev_history.item_count < DEVICE_HISTORY_MAX_ITEMS) {
        s_dev_history.item_count++;
    }

    s_dev_history.total_scans++;
    s_dev_history.total++;
    if (result && strcmp(result, "OK") == 0) {
        s_dev_history.ok_count++;
    } else if (result && strcmp(result, "VERIFIED") == 0) {
        s_dev_history.verified_count++;
    } else {
        s_dev_history.failed_count++;
    }

    s_dev_history.is_stale = true;
    push_history_cache_to_oled_locked();
    if (g_history_mutex) xSemaphoreGive(g_history_mutex);

    ESP_LOGI(TAG, "History local record: %s [%s] (Total=%d, OK=%d, ERR=%d)",
             ticket_code ? ticket_code : "--", result ? result : "?",
             s_dev_history.total_scans, s_dev_history.ok_count, s_dev_history.failed_count);
}

static esp_err_t fetchDeviceHistory(int page, int limit, const char *result_filter, bool event_only)
{
    if (page < 1) page = 1;
    if (limit < 1) limit = 5;
    if (limit > DEVICE_HISTORY_MAX_ITEMS) limit = DEVICE_HISTORY_MAX_ITEMS;
    const char *flt = (result_filter && result_filter[0]) ? result_filter : "ALL";

    char dev_id[64] = "";
    char dev_sec[160] = "";
    if (g_iot_mutex) xSemaphoreTake(g_iot_mutex, portMAX_DELAY);
    snprintf(dev_id, sizeof(dev_id), "%s", g_iot_device_id);
    snprintf(dev_sec, sizeof(dev_sec), "%s", g_iot_device_secret);
    bool paired = is_iot_device_paired_active();
    if (g_iot_mutex) xSemaphoreGive(g_iot_mutex);

    if (!paired || dev_id[0] == '\0' || dev_sec[0] == '\0') {
        if (g_history_mutex) xSemaphoreTake(g_history_mutex, portMAX_DELAY);
        s_dev_history.offline_cached = true;
        s_dev_history.loading = false;
        snprintf(s_dev_history.last_error, sizeof(s_dev_history.last_error), "CHUA GHEP IOT");
        push_history_cache_to_oled_locked();
        if (g_history_mutex) xSemaphoreGive(g_history_mutex);
        return ESP_ERR_INVALID_STATE;
    }

    if (!is_valid_iot_url(g_iot_base_url)) {
        if (g_history_mutex) xSemaphoreTake(g_history_mutex, portMAX_DELAY);
        s_dev_history.offline_cached = true;
        s_dev_history.loading = false;
        snprintf(s_dev_history.last_error, sizeof(s_dev_history.last_error), "CHUA CAU HINH URL");
        push_history_cache_to_oled_locked();
        if (g_history_mutex) xSemaphoreGive(g_history_mutex);
        return ESP_ERR_INVALID_STATE;
    }

    if (g_history_mutex) xSemaphoreTake(g_history_mutex, portMAX_DELAY);
    s_dev_history.loading = true;
    s_dev_history.page = page;
    s_dev_history.limit = limit;
    snprintf(s_dev_history.filter, sizeof(s_dev_history.filter), "%s", flt);
    s_dev_history.event_only = event_only;
    push_history_cache_to_oled_locked();
    if (g_history_mutex) xSemaphoreGive(g_history_mutex);

    char path[160];
    snprintf(path, sizeof(path), "/devices/history?page=%d&limit=%d&result=%s&event_only=%s",
             page, limit, flt, event_only ? "true" : "false");
    char url[256];
    build_iot_url(path, url, sizeof(url));

    char *resp_buf = (char *)heap_caps_malloc(8192, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!resp_buf) {
        resp_buf = (char *)malloc(8192);
    }
    if (!resp_buf) {
        if (g_history_mutex) xSemaphoreTake(g_history_mutex, portMAX_DELAY);
        s_dev_history.loading = false;
        snprintf(s_dev_history.last_error, sizeof(s_dev_history.last_error), "HET RAM HEAP");
        push_history_cache_to_oled_locked();
        if (g_history_mutex) xSemaphoreGive(g_history_mutex);
        return ESP_ERR_NO_MEM;
    }
    resp_buf[0] = '\0';

    int http_code = -1;
    ESP_LOGI(TAG, "Fetching Device History: %s...", url);
    esp_err_t err = esp32_http_json_request(HTTP_METHOD_GET, url, NULL,
                                            NULL, NULL, dev_id, dev_sec,
                                            resp_buf, 8192, &http_code);

    if (g_history_mutex) xSemaphoreTake(g_history_mutex, portMAX_DELAY);
    s_dev_history.last_http_code = http_code;
    s_dev_history.loading = false;

    if (err != ESP_OK || http_code != 200) {
        s_dev_history.offline_cached = true;
        if (http_code == 403) {
            snprintf(s_dev_history.last_error, sizeof(s_dev_history.last_error), "MAY BI KHOA (403)");
            if (g_iot_mutex) xSemaphoreTake(g_iot_mutex, portMAX_DELAY);
            g_iot_state = IOT_STATE_DISABLED;
            snprintf(g_iot_last_err, sizeof(g_iot_last_err), "DEVICE_DISABLED");
            if (g_iot_mutex) xSemaphoreGive(g_iot_mutex);
            sync_oled_network_state();
        } else if (http_code == 401) {
            snprintf(s_dev_history.last_error, sizeof(s_dev_history.last_error), "SAI SECRET (401)");
            if (g_iot_mutex) xSemaphoreTake(g_iot_mutex, portMAX_DELAY);
            snprintf(g_iot_last_err, sizeof(g_iot_last_err), "UNAUTHORIZED");
            if (g_iot_mutex) xSemaphoreGive(g_iot_mutex);
        } else {
            snprintf(s_dev_history.last_error, sizeof(s_dev_history.last_error), "HTTP %d", http_code);
        }
        push_history_cache_to_oled_locked();
        if (g_history_mutex) xSemaphoreGive(g_history_mutex);
        free(resp_buf);
        return ESP_FAIL;
    }

    if (resp_buf[0] != '{') {
        s_dev_history.offline_cached = true;
        snprintf(s_dev_history.last_error, sizeof(s_dev_history.last_error), "PARSE JSON ERR");
        push_history_cache_to_oled_locked();
        if (g_history_mutex) xSemaphoreGive(g_history_mutex);
        free(resp_buf);
        return ESP_FAIL;
    }

    /* Parse pagination */
    const char *pag_ptr = find_json_key(resp_buf, "pagination");
    if (pag_ptr && *pag_ptr == '{') {
        char pag_buf[192];
        extract_json_object_slice(pag_ptr, pag_buf, sizeof(pag_buf));
        extract_json_int(pag_buf, "page", &s_dev_history.page);
        extract_json_int(pag_buf, "limit", &s_dev_history.limit);
        extract_json_int(pag_buf, "total", &s_dev_history.total);
        extract_json_bool(pag_buf, "has_more", &s_dev_history.has_more);
    }

    /* Parse stats */
    const char *stats_ptr = find_json_key(resp_buf, "stats");
    if (stats_ptr && *stats_ptr == '{') {
        char stats_buf[192];
        extract_json_object_slice(stats_ptr, stats_buf, sizeof(stats_buf));
        extract_json_int(stats_buf, "total_scans", &s_dev_history.total_scans);
        extract_json_int(stats_buf, "ok_count", &s_dev_history.ok_count);
        extract_json_int(stats_buf, "verified_count", &s_dev_history.verified_count);
        extract_json_int(stats_buf, "failed_count", &s_dev_history.failed_count);
    }

    /* Parse items array (zero AST heap allocation) */
    const char *items_ptr = find_json_key(resp_buf, "items");
    int count = 0;
    if (items_ptr && *items_ptr == '[') {
        const char *p = items_ptr + 1;
        while (*p && *p != ']' && count < DEVICE_HISTORY_MAX_ITEMS) {
            while (*p && *p != '{' && *p != ']') p++;
            if (*p != '{') break;

            char item_buf[640];
            const char *next_p = extract_json_object_slice(p, item_buf, sizeof(item_buf));
            if (!next_p) break;
            p = next_p;

            dev_history_item_t *dst = &s_dev_history.items[count];
            memset(dst, 0, sizeof(*dst));

            extract_json_str(item_buf, "id", dst->id, sizeof(dst->id));
            extract_json_str(item_buf, "ticket_code", dst->ticket_code, sizeof(dst->ticket_code));
            extract_json_str(item_buf, "attendee_name", dst->attendee_name, sizeof(dst->attendee_name));
            extract_json_str(item_buf, "event_id", dst->event_id, sizeof(dst->event_id));
            extract_json_str(item_buf, "event_code", dst->event_code, sizeof(dst->event_code));
            extract_json_str(item_buf, "event_name", dst->event_name, sizeof(dst->event_name));
            extract_json_str(item_buf, "action", dst->action, sizeof(dst->action));
            extract_json_str(item_buf, "result", dst->result, sizeof(dst->result));
            extract_json_str(item_buf, "message", dst->message, sizeof(dst->message));
            extract_json_str(item_buf, "scanned_at", dst->scanned_at, sizeof(dst->scanned_at));
            if (!extract_json_bool(item_buf, "success", &dst->success)) {
                dst->success = (strcmp(dst->result, "OK") == 0);
            }

            count++;
        }
    }

    s_dev_history.item_count = count;
    s_dev_history.offline_cached = false;
    s_dev_history.is_stale = false;
    s_dev_history.last_error[0] = '\0';
    push_history_cache_to_oled_locked();
    if (g_history_mutex) xSemaphoreGive(g_history_mutex);

    free(resp_buf);

    ESP_LOGI(TAG, "Device History synced: %d items (page %d, total %d, has_more=%d, OK=%d, ERR=%d)",
             count, s_dev_history.page, s_dev_history.total, s_dev_history.has_more ? 1 : 0,
             s_dev_history.ok_count, s_dev_history.failed_count);
    return ESP_OK;
}

/* ============================================================================
 * 6. WEB SERVER ĐỘC LẬP TRÊN ESP32 (PORT 80)
 * ============================================================================ */
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static esp_err_t jpg_stream_httpd_handler(httpd_req_t *req)
{
    camera_fb_t *fb = NULL;
    esp_err_t res = ESP_OK;
    char part_buf[64];

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if (res != ESP_OK) return res;
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    while (true) {
        fb = esp_camera_fb_get();
        if (!fb) {
            res = ESP_FAIL;
            break;
        }

        uint8_t *jpg_buf = fb->buf;
        size_t jpg_len = fb->len;
        bool jpg_alloc = false;

        if (fb->format != PIXFORMAT_JPEG) {
            jpg_buf = NULL;
            jpg_len = 0;
            if (!frame2jpg(fb, 75, &jpg_buf, &jpg_len) || !jpg_buf || jpg_len < 4) {
                esp_camera_fb_return(fb);
                fb = NULL;
                if (jpg_buf) free(jpg_buf);
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }
            jpg_alloc = true;
        }

        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        }
        if (res == ESP_OK) {
            size_t hlen = snprintf(part_buf, 64, _STREAM_PART, (uint32_t)jpg_len);
            res = httpd_resp_send_chunk(req, part_buf, hlen);
        }
        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(req, (const char *)jpg_buf, jpg_len);
        }
        if (jpg_alloc && jpg_buf) {
            free(jpg_buf);
        }
        esp_camera_fb_return(fb);
        fb = NULL;

        if (res != ESP_OK) break;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return res;
}

static void send_cors_headers(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type, Authorization");
}

static esp_err_t cors_options_httpd_handler(httpd_req_t *req)
{
    send_cors_headers(req);
    httpd_resp_set_status(req, "204 No Content");
    return httpd_resp_send(req, NULL, 0);
}

static const char *find_json_key(const char *json, const char *key)
{
    if (!json || !key) return NULL;
    char pattern[64];
    snprintf(pattern, sizeof(pattern), "\"%s\"", key);
    const char *p = json;
    size_t plen = strlen(pattern);
    while ((p = strstr(p, pattern)) != NULL) {
        const char *q = p + plen;
        while (*q == ' ' || *q == '\t' || *q == '\r' || *q == '\n') q++;
        if (*q == ':') {
            q++;
            while (*q == ' ' || *q == '\t' || *q == '\r' || *q == '\n') q++;
            return q;
        }
        p += plen;
    }
    return NULL;
}

static bool extract_json_str(const char *json, const char *key, char *out, size_t out_max)
{
    if (!out || out_max == 0) return false;
    out[0] = '\0';
    const char *val = find_json_key(json, key);
    if (!val || *val != '\"') return false;
    val++;
    size_t idx = 0;
    while (*val && *val != '\"' && idx < out_max - 1) {
        if (*val == '\\' && *(val + 1)) {
            val++;
        }
        out[idx++] = *val++;
    }
    out[idx] = '\0';
    return true;
}

static bool extract_json_bool(const char *json, const char *key, bool *out_val)
{
    const char *val = find_json_key(json, key);
    if (!val) return false;
    if (strncmp(val, "true", 4) == 0 || strncmp(val, "\"true\"", 6) == 0 || *val == '1' || strncmp(val, "\"1\"", 3) == 0) {
        *out_val = true;
        return true;
    }
    if (strncmp(val, "false", 5) == 0 || strncmp(val, "\"false\"", 7) == 0 || *val == '0' || strncmp(val, "\"0\"", 3) == 0) {
        *out_val = false;
        return true;
    }
    return false;
}

static bool extract_json_int(const char *json, const char *key, int *out_val)
{
    const char *val = find_json_key(json, key);
    if (!val) return false;
    if (*val == '\"') val++;
    char *endp = NULL;
    long v = strtol(val, &endp, 10);
    if (endp == val) return false;
    if (out_val) *out_val = (int)v;
    return true;
}

static bool is_json_null_val(const char *json, const char *key)
{
    const char *val = find_json_key(json, key);
    if (!val) return false;
    return (strncmp(val, "null", 4) == 0);
}

static void json_escape_copy(const char *src, char *dst, size_t dst_max)
{
    if (!dst || dst_max == 0) return;
    if (!src) {
        dst[0] = '\0';
        return;
    }
    size_t idx = 0;
    for (size_t i = 0; src[i] != '\0' && idx < dst_max - 2; i++) {
        char c = src[i];
        if (c == '\"' || c == '\\') {
            if (idx + 2 >= dst_max) break;
            dst[idx++] = '\\';
            dst[idx++] = c;
        } else if ((unsigned char)c < 0x20) {
            dst[idx++] = ' ';
        } else {
            dst[idx++] = c;
        }
    }
    dst[idx] = '\0';
}

static void extract_ticket_code_c(const char *raw, char *out, size_t out_max)
{
    if (!raw || !out || out_max == 0) return;
    out[0] = '\0';
    while (*raw == ' ' || *raw == '\t' || *raw == '\r' || *raw == '\n') raw++;
    if (raw[0] == '{') {
        if (extract_json_str(raw, "ticket_code", out, out_max) && out[0]) return;
        if (extract_json_str(raw, "ticketCode", out, out_max) && out[0]) return;
        if (extract_json_str(raw, "ticket", out, out_max) && out[0]) return;
        if (extract_json_str(raw, "code", out, out_max) && out[0]) return;
        if (extract_json_str(raw, "qrCode", out, out_max) && out[0]) return;
    }
    size_t len = strlen(raw);
    while (len > 0 && (raw[len - 1] == ' ' || raw[len - 1] == '\t' || raw[len - 1] == '\r' || raw[len - 1] == '\n')) {
        len--;
    }
    if (strncmp(raw, "http://", 7) == 0 || strncmp(raw, "https://", 8) == 0) {
        char url[256];
        size_t ulen = len < sizeof(url) - 1 ? len : sizeof(url) - 1;
        memcpy(url, raw, ulen);
        url[ulen] = '\0';
        const char *q = strchr(url, '?');
        if (q) {
            static const char *const qkeys[] = {
                "ticket_code=", "ticketCode=", "ticket=", "qrCode=", "code=", "qr=", "t="
            };
            for (size_t k = 0; k < sizeof(qkeys) / sizeof(qkeys[0]); k++) {
                size_t klen = strlen(qkeys[k]);
                const char *p = q;
                while ((p = strstr(p, qkeys[k])) != NULL) {
                    if (p == q || *(p - 1) == '?' || *(p - 1) == '&') {
                        const char *val = p + klen;
                        const char *end = val;
                        while (*end && *end != '&' && *end != '#') end++;
                        size_t vlen = (size_t)(end - val);
                        if (vlen > 0) {
                            if (vlen >= out_max) vlen = out_max - 1;
                            memcpy(out, val, vlen);
                            out[vlen] = '\0';
                            return;
                        }
                    }
                    p += klen;
                }
            }
        }
        const char *scheme = strstr(url, "://");
        const char *path = scheme ? strchr(scheme + 3, '/') : NULL;
        if (path) {
            char pbuf[256];
            snprintf(pbuf, sizeof(pbuf), "%s", path);
            char *stop = strpbrk(pbuf, "?#");
            if (stop) *stop = '\0';
            size_t plen = strlen(pbuf);
            while (plen > 0 && pbuf[plen - 1] == '/') pbuf[--plen] = '\0';
            const char *slash = strrchr(pbuf, '/');
            const char *seg = slash ? (slash + 1) : pbuf;
            if (seg[0] != '\0') {
                snprintf(out, out_max, "%s", seg);
                return;
            }
        }
    }
    if (len >= out_max) len = out_max - 1;
    memcpy(out, raw, len);
    out[len] = '\0';
}

static void update_last_scan_state(const char *qr, const char *status, const char *msg, bool auto_ci)
{
    if (g_scan_state_mutex) xSemaphoreTake(g_scan_state_mutex, portMAX_DELAY);
    if (qr) snprintf(s_last_scan.qr, sizeof(s_last_scan.qr), "%s", qr);
    if (status) snprintf(s_last_scan.status, sizeof(s_last_scan.status), "%s", status);
    if (msg) snprintf(s_last_scan.message, sizeof(s_last_scan.message), "%s", msg);
    s_last_scan.auto_checked_in = auto_ci;
    s_last_scan.seq++;
    if (g_scan_state_mutex) xSemaphoreGive(g_scan_state_mutex);
}

static void build_iot_status_json_fragment(char *buf, size_t buf_max)
{
    char esc_name[96], esc_ec[96], esc_en[160], esc_url[256], esc_err[160];
    if (g_iot_mutex) xSemaphoreTake(g_iot_mutex, portMAX_DELAY);
    json_escape_copy(g_iot_device_name, esc_name, sizeof(esc_name));
    json_escape_copy(g_iot_event_code, esc_ec, sizeof(esc_ec));
    json_escape_copy(g_iot_event_name, esc_en, sizeof(esc_en));
    json_escape_copy(g_iot_base_url, esc_url, sizeof(esc_url));
    json_escape_copy(g_iot_last_err, esc_err, sizeof(esc_err));
    snprintf(buf, buf_max,
             "{\"mac\":\"%s\",\"mac_short\":\"%s\",\"state\":\"%s\",\"paired\":%s,"
             "\"device_id\":\"%s\",\"device_name\":\"%s\",\"pairing_code\":\"%s\","
             "\"assigned_event_id\":\"%s\",\"event_code\":\"%s\",\"event_name\":\"%s\","
             "\"config_version\":%d,\"pending_auto_sync\":%s,\"iot_base_url\":\"%s\",\"last_error\":\"%s\"}",
             g_device_mac,
             g_device_mac_short,
             iot_state_to_str(g_iot_state),
             is_iot_device_paired_active() ? "true" : "false",
             g_iot_device_id,
             esc_name,
             g_iot_pairing_code,
             g_iot_assigned_event_id,
             esc_ec,
             esc_en,
             g_iot_config_version,
             g_iot_pending_auto_sync ? "true" : "false",
             esc_url,
             esc_err);
    if (g_iot_mutex) xSemaphoreGive(g_iot_mutex);
}

static esp_err_t status_httpd_handler(httpd_req_t *req)
{
    char *json_response = (char *)malloc(2816);
    char *iot_frag = (char *)malloc(1536);
    if (!json_response || !iot_frag) {
        free(json_response);
        free(iot_frag);
        return httpd_resp_send_500(req);
    }

    wifi_ap_record_t ap_info;
    int8_t rssi = -100;
    if (g_wifi_connected && esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
        rssi = ap_info.rssi;
        g_wifi_rssi = rssi;
    }

    char esc_ssid[96], esc_qr[256], esc_msg[384];
    json_escape_copy(g_wifi_ssid, esc_ssid, sizeof(esc_ssid));

    standalone_scan_state_t scan_copy;
    if (g_scan_state_mutex) xSemaphoreTake(g_scan_state_mutex, portMAX_DELAY);
    scan_copy = s_last_scan;
    if (g_scan_state_mutex) xSemaphoreGive(g_scan_state_mutex);

    json_escape_copy(scan_copy.qr, esc_qr, sizeof(esc_qr));
    json_escape_copy(scan_copy.message, esc_msg, sizeof(esc_msg));
    build_iot_status_json_fragment(iot_frag, 1536);

    snprintf(json_response, 2816,
             "{\"status\":\"ok\",\"device\":\"ESP32-S3 OV5640\",\"wifi_connected\":%s,\"ip\":\"%s\",\"rssi\":%d,\"ssid\":\"%s\","
             "\"wifi_profiles_count\":%d,\"auto_confirm\":%s,\"standalone_active\":%s,\"mode\":\"standalone\","
             "\"iot\":%s,"
             "\"last_scan\":{\"qr\":\"%s\",\"status\":\"%s\",\"message\":\"%s\",\"seq\":%lu,\"auto_checked_in\":%s}}",
             g_wifi_connected ? "true" : "false",
             g_wifi_ip,
             rssi,
             esc_ssid,
             g_wifi_profile_count,
             g_auto_confirm ? "true" : "false",
             !is_pc_usb_active() ? "true" : "false",
             iot_frag,
             esc_qr,
             scan_copy.status,
             esc_msg,
             (unsigned long)scan_copy.seq,
             scan_copy.auto_checked_in ? "true" : "false");

    httpd_resp_set_type(req, "application/json");
    send_cors_headers(req);
    esp_err_t err = httpd_resp_send(req, json_response, strlen(json_response));
    free(iot_frag);
    free(json_response);
    return err;
}

static esp_err_t iot_status_httpd_handler(httpd_req_t *req)
{
    char *iot_frag = (char *)malloc(1536);
    if (!iot_frag) return httpd_resp_send_500(req);
    build_iot_status_json_fragment(iot_frag, 1536);
    httpd_resp_set_type(req, "application/json");
    send_cors_headers(req);
    esp_err_t err = httpd_resp_send(req, iot_frag, strlen(iot_frag));
    free(iot_frag);
    return err;
}

static esp_err_t iot_config_httpd_handler(httpd_req_t *req)
{
    char content[512];
    int ret = httpd_req_recv(req, content, sizeof(content) - 1);
    if (ret <= 0) return httpd_resp_send_500(req);
    content[ret] = '\0';

    char new_url[192] = "";
    bool do_reset = false;
    bool do_sync = false;
    extract_json_str(content, "iot_base_url", new_url, sizeof(new_url));
    extract_json_bool(content, "reset", &do_reset);
    extract_json_bool(content, "sync", &do_sync);

    if (do_reset) {
        clear_iot_credentials_nvs(true);
        sync_oled_network_state();
        ESP_LOGW(TAG, "HTTP: Cleared IoT pairing credentials for re-registration");
    }
    if (new_url[0] != '\0') {
        char norm_url[192];
        normalize_iot_url(new_url, norm_url, sizeof(norm_url));
        if (norm_url[0] != '\0') {
            if (g_iot_mutex) xSemaphoreTake(g_iot_mutex, portMAX_DELAY);
            snprintf(g_iot_base_url, sizeof(g_iot_base_url), "%s", norm_url);
            if (g_iot_mutex) xSemaphoreGive(g_iot_mutex);
            save_iot_nvs_str("iot_url", norm_url);
            g_iot_force_sync = true;
            ESP_LOGI(TAG, "HTTP: Persistent IoT Base URL saved to NVS -> %s", norm_url);
        }
    } else if (do_sync) {
        g_iot_force_sync = true;
    }

    return iot_status_httpd_handler(req);
}

static esp_err_t auto_confirm_httpd_handler(httpd_req_t *req)
{
    char content[256];
    int ret = httpd_req_recv(req, content, sizeof(content) - 1);
    if (ret <= 0) return httpd_resp_send_500(req);
    content[ret] = '\0';

    bool enabled = g_auto_confirm;
    if (!extract_json_bool(content, "enabled", &enabled)) {
        extract_json_bool(content, "auto_confirm", &enabled);
    }
    mark_local_auto_confirm_changed(enabled);

    char ack[48];
    snprintf(ack, sizeof(ack), "AUTOCFG:%d\n", enabled ? 1 : 0);
    safe_uart_write_string(ack);

    char resp[128];
    snprintf(resp, sizeof(resp), "{\"success\":true,\"auto_confirm\":%s}", enabled ? "true" : "false");
    httpd_resp_set_type(req, "application/json");
    send_cors_headers(req);
    return httpd_resp_send(req, resp, strlen(resp));
}

static bool parse_wifi_creds_from_req(httpd_req_t *req, char *ssid, size_t ssid_max, char *pass, size_t pass_max)
{
    char content[256];
    int ret = httpd_req_recv(req, content, sizeof(content) - 1);
    if (ret <= 0) return false;
    content[ret] = '\0';

    ssid[0] = '\0';
    pass[0] = '\0';
    extract_json_str(content, "ssid", ssid, ssid_max);
    if (!extract_json_str(content, "pass", pass, pass_max)) {
        extract_json_str(content, "password", pass, pass_max);
    }
    return true;
}

static esp_err_t wifi_cfg_httpd_handler(httpd_req_t *req)
{
    char ssid[33] = {0};
    char pass[65] = {0};
    if (!parse_wifi_creds_from_req(req, ssid, sizeof(ssid), pass, sizeof(pass))) {
        return httpd_resp_send_500(req);
    }

    if (strlen(ssid) > 0) {
        connect_to_new_wifi(ssid, pass);
        const char *resp = "{\"success\":true,\"message\":\"ESP32 dang ket noi Wi-Fi moi...\"}";
        httpd_resp_set_type(req, "application/json");
        send_cors_headers(req);
        return httpd_resp_send(req, resp, strlen(resp));
    }

    const char *resp = "{\"success\":false,\"message\":\"Thieu SSID hop le\"}";
    httpd_resp_set_type(req, "application/json");
    send_cors_headers(req);
    return httpd_resp_send(req, resp, strlen(resp));
}

static esp_err_t wifi_profiles_httpd_handler(httpd_req_t *req)
{
    char ssid[33] = {0};
    char pass[65] = {0};
    if (!parse_wifi_creds_from_req(req, ssid, sizeof(ssid), pass, sizeof(pass))) {
        return httpd_resp_send_500(req);
    }

    if (ssid[0] != '\0') {
        add_or_promote_wifi_profile(ssid, pass, false);
    }

    char resp[96];
    snprintf(resp, sizeof(resp), "{\"success\":true,\"profiles\":%d}", g_wifi_profile_count);
    httpd_resp_set_type(req, "application/json");
    send_cors_headers(req);
    return httpd_resp_send(req, resp, strlen(resp));
}

static bool strcasestr_ascii(const char *hay, const char *needle)
{
    if (!hay || !needle || !needle[0]) return false;
    size_t nlen = strlen(needle);
    for (; *hay; hay++) {
        size_t i = 0;
        while (i < nlen && hay[i]) {
            char a = hay[i];
            char b = needle[i];
            if (a >= 'A' && a <= 'Z') a = (char)(a + ('a' - 'A'));
            if (b >= 'A' && b <= 'Z') b = (char)(b + ('a' - 'A'));
            if (a != b) break;
            i++;
        }
        if (i == nlen) return true;
    }
    return false;
}

static bool is_json_ticket_used(const char *json)
{
    if (!json) return false;
    if (strstr(json, "\"USED\"") != NULL || strstr(json, "\"CHECKED_IN\"") != NULL) return true;
    bool flag = false;
    if (extract_json_bool(json, "isCheckedIn", &flag) && flag) return true;
    if (extract_json_bool(json, "checked_in", &flag) && flag) return true;
    if (strcasestr_ascii(json, "ALREADY_CHECKED_IN") || strcasestr_ascii(json, "ALREADY_USED") ||
        strcasestr_ascii(json, "already checked") || strcasestr_ascii(json, "already used") ||
        strcasestr_ascii(json, "da duoc su dung") || strcasestr_ascii(json, "da check-in") ||
        strcasestr_ascii(json, "da checkin") ||
        (strcasestr_ascii(json, "check") && (strstr(json, "\xc4\x91\xc3\xa3") != NULL || strstr(json, "\xc4\x90\xc3\xa3") != NULL)) ||
        strstr(json, "s\xe1\xbb\xad d\xe1\xbb\xa5ng") != NULL) {
        return true;
    }
    return false;
}

static bool is_json_network_error(const char *json)
{
    if (!json || json[0] == '\0') return true;
    return (strstr(json, "Loi ket noi") != NULL || strstr(json, "ESP_ERR_") != NULL);
}

static void apply_iot_server_config_from_json(const char *json, bool from_scan);

static void sync_device_config_from_scan_response(const char *resp_json)
{
    const char *obj = find_json_key(resp_json, "device_config");
    if (!obj || *obj != '{') return;

    char cfg_buf[512];
    size_t idx = 0;
    int depth = 0;
    bool in_str = false;
    const char *p = obj;
    while (*p && idx < sizeof(cfg_buf) - 1) {
        char c = *p++;
        cfg_buf[idx++] = c;
        if (c == '\\' && in_str && *p) {
            if (idx < sizeof(cfg_buf) - 1) cfg_buf[idx++] = *p++;
            continue;
        }
        if (c == '"') {
            in_str = !in_str;
        } else if (!in_str) {
            if (c == '{') depth++;
            else if (c == '}') {
                depth--;
                if (depth <= 0) break;
            }
        }
    }
    cfg_buf[idx] = '\0';
    apply_iot_server_config_from_json(cfg_buf, true);
}

static oled_ticket_status_t process_iot_scan_response(const char *raw_qr, const char *clean_qr,
                                                      const char *resp_json, int http_code,
                                                      bool *out_net_err)
{
    if (out_net_err) *out_net_err = false;
    if (resp_json && strstr(resp_json, "API_URL_NOT_CONFIGURED") != NULL) {
        if (out_net_err) *out_net_err = true;
        oled_show_ticket_status(OLED_TICKET_NET_ERR, clean_qr, "CHUA CAU HINH URL");
        update_last_scan_state(raw_qr, "ERROR", "Chua cau hinh URL API!", false);
        buzzer_beep(240);
        led_trigger_red_error();
        return OLED_TICKET_NET_ERR;
    }
    if (http_code <= 0 || is_json_network_error(resp_json)) {
        if (out_net_err) *out_net_err = true;
        oled_show_ticket_status(OLED_TICKET_NET_ERR, clean_qr, "KIEM TRA MANG");
        update_last_scan_state(raw_qr, "ERROR", "Loi ket noi server IoT", false);
        buzzer_beep(240);
        led_trigger_red_error();
        return OLED_TICKET_NET_ERR;
    }

    sync_device_config_from_scan_response(resp_json);

    char result_str[32] = "";
    char error_code[32] = "";
    char msg[192] = "";
    extract_json_str(resp_json, "result", result_str, sizeof(result_str));
    extract_json_str(resp_json, "code", error_code, sizeof(error_code));
    extract_json_str(resp_json, "message", msg, sizeof(msg));

    if (http_code == 401) {
        clear_iot_credentials_nvs(true);
        sync_oled_network_state();
        oled_show_ticket_status(OLED_TICKET_NET_ERR, clean_qr, "CAN RESET GHEP");
        update_last_scan_state(raw_qr, "ERROR", msg[0] ? msg : "Thiet bi can ghep noi lai", false);
        buzzer_beep(240);
        led_trigger_red_error();
        return OLED_TICKET_NET_ERR;
    }

    if (http_code == 403 && (strcmp(result_str, "DEVICE_DISABLED") == 0 || strcmp(error_code, "DEVICE_DISABLED") == 0)) {
        if (g_iot_mutex) xSemaphoreTake(g_iot_mutex, portMAX_DELAY);
        g_iot_state = IOT_STATE_DISABLED;
        if (g_iot_mutex) xSemaphoreGive(g_iot_mutex);
        sync_oled_network_state();
        oled_show_ticket_status(OLED_TICKET_NET_ERR, clean_qr, "MAY BI KHOA!");
        update_last_scan_state(raw_qr, "DISABLED", msg[0] ? msg : "Thiet bi da bi khoa boi Admin", false);
        buzzer_beep(240);
        led_trigger_red_error();
        return OLED_TICKET_NET_ERR;
    }

    if (http_code == 403 || strcmp(result_str, "WRONG_EVENT") == 0 || strcmp(error_code, "WRONG_EVENT") == 0) {
        record_local_history_scan(clean_qr, "WRONG_EVENT", msg[0] ? msg : "Ve khong thuoc su kien", false);
        oled_show_ticket_status(OLED_TICKET_WRONG_EVENT, clean_qr, "SAI SU KIEN!");
        update_last_scan_state(raw_qr, "WRONG_EVENT", msg[0] ? msg : "Ve khong thuoc su kien cua thiet bi nay", false);
        buzzer_beep(240);
        led_trigger_red_error();
        return OLED_TICKET_WRONG_EVENT;
    }

    if (http_code == 400 && (strcmp(result_str, "NO_EVENT_ASSIGNED") == 0 || strcmp(error_code, "NO_EVENT_ASSIGNED") == 0)) {
        record_local_history_scan(clean_qr, "NO_EVENT", msg[0] ? msg : "Chua duoc gan su kien", false);
        oled_show_ticket_status(OLED_TICKET_WRONG_EVENT, clean_qr, "CHUA GAN SK!");
        update_last_scan_state(raw_qr, "NO_EVENT", msg[0] ? msg : "Admin chua gan su kien cho thiet bi", false);
        buzzer_beep(240);
        led_trigger_red_error();
        return OLED_TICKET_WRONG_EVENT;
    }

    if (strcmp(result_str, "OK") == 0) {
        record_local_history_scan(clean_qr, "OK", msg[0] ? msg : "Check-in thanh cong", true);
        oled_show_ticket_status(OLED_TICKET_OK, clean_qr, "VE HOP LE");
        update_last_scan_state(raw_qr, "CONFIRMED", msg[0] ? msg : "Check-in IoT thanh cong!", true);
        /* Kêu bíp bíp 2 tiếng vui tai khi vé hợp lệ */
        buzzer_beep(70);
        vTaskDelay(pdMS_TO_TICKS(80));
        buzzer_beep(90);
        servo_trigger_gate_open();
        led_trigger_green_ok();
        return OLED_TICKET_OK;
    }

    if (strcmp(result_str, "VERIFIED") == 0) {
        record_local_history_scan(clean_qr, "VERIFIED", msg[0] ? msg : "Ve hop le (cho xac nhan)", false);
        oled_show_ticket_status(OLED_TICKET_VERIFIED, clean_qr, "BAM XAC NHAN");
        update_last_scan_state(raw_qr, "VERIFIED", msg[0] ? msg : "Ve hop le (cho xac nhan)", false);
        /* Kêu bíp bíp 2 tiếng cho vé hợp lệ chờ xác nhận */
        buzzer_beep(70);
        vTaskDelay(pdMS_TO_TICKS(80));
        buzzer_beep(90);
        led_trigger_green_ok();
        return OLED_TICKET_VERIFIED;
    }

    if (http_code == 409 || strcmp(result_str, "USED") == 0 || is_json_ticket_used(resp_json)) {
        record_local_history_scan(clean_qr, "USED", msg[0] ? msg : "Ve da duoc su dung", false);
        oled_show_ticket_status(OLED_TICKET_USED, clean_qr, "DA CHECKIN ROI");
        update_last_scan_state(raw_qr, "USED", msg[0] ? msg : "Ve nay da duoc su dung truoc do", false);
        buzzer_beep(240);
        led_trigger_red_error();
        return OLED_TICKET_USED;
    }

    record_local_history_scan(clean_qr, "NOT_FOUND", msg[0] ? msg : "Ve khong ton tai", false);
    oled_show_ticket_status(OLED_TICKET_NOT_FOUND, clean_qr, "VE KO TON TAI");
    update_last_scan_state(raw_qr, "INVALID", msg[0] ? msg : "Khong tim thay hoac ve khong hop le", false);
    buzzer_beep(240);
    led_trigger_red_error();
    return OLED_TICKET_NOT_FOUND;
}

static esp_err_t handle_ticket_action_httpd(httpd_req_t *req, bool is_checkin)
{
    char *content = (char *)malloc(2048);
    if (!content) return httpd_resp_send_500(req);

    int ret = httpd_req_recv(req, content, 2047);
    if (ret <= 0) {
        free(content);
        return httpd_resp_send_500(req);
    }
    content[ret] = '\0';

    char qr_code[128] = {0};
    extract_json_str(content, "qrCode", qr_code, sizeof(qr_code));

    char clean_qr[64] = "";
    extract_ticket_code_c(qr_code, clean_qr, sizeof(clean_qr));
    oled_show_ticket_status(OLED_TICKET_CHECKING, clean_qr, NULL);

    char *https_result = (char *)malloc(4096);
    if (https_result) {
        https_result[0] = '\0';
        int http_code = 400;
        if (is_iot_device_paired_active()) {
            esp32_iot_scan_ticket(qr_code, is_checkin ? "checkin" : "verify",
                                  https_result, 4096, &http_code);
            process_iot_scan_response(qr_code, clean_qr, https_result, http_code, NULL);
            if (http_code == 400) httpd_resp_set_status(req, "400 Bad Request");
            else if (http_code == 401) httpd_resp_set_status(req, "401 Unauthorized");
            else if (http_code == 403) httpd_resp_set_status(req, "403 Forbidden");
            else if (http_code == 404) httpd_resp_set_status(req, "404 Not Found");
            else if (http_code == 409) httpd_resp_set_status(req, "409 Conflict");
            else if (http_code <= 0 || http_code >= 500) httpd_resp_set_status(req, "502 Bad Gateway");
        } else {
            snprintf(https_result, 4096,
                     "{\"success\":false,\"errorCode\":\"IOT_NOT_PAIRED\",\"statusCode\":400,"
                     "\"message\":\"ESP32 chua duoc Admin duyet ghep noi IoT!\"}");
            httpd_resp_set_status(req, "400 Bad Request");
            oled_show_ticket_status(OLED_TICKET_NET_ERR, clean_qr, "CHUA GHEP IOT");
            buzzer_beep(240);
        }
        httpd_resp_set_type(req, "application/json");
        send_cors_headers(req);
        httpd_resp_send(req, https_result, strlen(https_result));
        free(https_result);
    } else {
        httpd_resp_send_500(req);
    }

    free(content);
    return ESP_OK;
}

static esp_err_t verify_ticket_httpd_handler(httpd_req_t *req)
{
    return handle_ticket_action_httpd(req, false);
}

static esp_err_t checkin_ticket_httpd_handler(httpd_req_t *req)
{
    return handle_ticket_action_httpd(req, true);
}

static esp_err_t history_httpd_handler(httpd_req_t *req)
{
    char query[128] = "";
    int page = 1;
    int limit = 5;
    char filter[24] = "ALL";
    bool event_only = false;
    bool force_refresh = false;

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        char val[32];
        if (httpd_query_key_value(query, "page", val, sizeof(val)) == ESP_OK) {
            page = atoi(val);
            if (page < 1) page = 1;
        }
        if (httpd_query_key_value(query, "limit", val, sizeof(val)) == ESP_OK) {
            limit = atoi(val);
            if (limit < 1) limit = 5;
            if (limit > DEVICE_HISTORY_MAX_ITEMS) limit = DEVICE_HISTORY_MAX_ITEMS;
        }
        if (httpd_query_key_value(query, "result", val, sizeof(val)) == ESP_OK) {
            snprintf(filter, sizeof(filter), "%.23s", val);
        }
        if (httpd_query_key_value(query, "event_only", val, sizeof(val)) == ESP_OK) {
            event_only = (strcmp(val, "true") == 0 || strcmp(val, "1") == 0);
        }
        if (httpd_query_key_value(query, "refresh", val, sizeof(val)) == ESP_OK) {
            force_refresh = (strcmp(val, "true") == 0 || strcmp(val, "1") == 0);
        }
    }

    if (force_refresh || s_dev_history.is_stale ||
        page != s_dev_history.page ||
        limit != s_dev_history.limit ||
        strcmp(filter, s_dev_history.filter) != 0 ||
        event_only != s_dev_history.event_only) {
        fetchDeviceHistory(page, limit, filter, event_only);
    }

    char *resp = (char *)heap_caps_malloc(8192, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!resp) resp = (char *)malloc(8192);
    if (!resp) return httpd_resp_send_500(req);

    if (g_history_mutex) xSemaphoreTake(g_history_mutex, portMAX_DELAY);
    size_t off = 0;
    off += snprintf(resp + off, 8192 - off,
                    "{\"success\":true,\"device_id\":\"%s\",\"device_name\":\"%s\","
                    "\"pagination\":{\"page\":%d,\"limit\":%d,\"total\":%d,\"has_more\":%s},"
                    "\"stats\":{\"total_scans\":%d,\"ok_count\":%d,\"verified_count\":%d,\"failed_count\":%d},"
                    "\"filter\":\"%s\",\"event_only\":%s,\"offline_cached\":%s,\"loading\":%s,"
                    "\"last_error\":\"%s\",\"items\":[",
                    g_iot_device_id, g_iot_device_name,
                    s_dev_history.page, s_dev_history.limit, s_dev_history.total, s_dev_history.has_more ? "true" : "false",
                    s_dev_history.total_scans, s_dev_history.ok_count, s_dev_history.verified_count, s_dev_history.failed_count,
                    s_dev_history.filter, s_dev_history.event_only ? "true" : "false",
                    s_dev_history.offline_cached ? "true" : "false",
                    s_dev_history.loading ? "true" : "false",
                    s_dev_history.last_error);

    for (int i = 0; i < s_dev_history.item_count && off + 320 < 8192; i++) {
        const dev_history_item_t *it = &s_dev_history.items[i];
        char esc_tck[96], esc_att[96], esc_msg[160];
        json_escape_into(it->ticket_code, esc_tck, sizeof(esc_tck));
        json_escape_into(it->attendee_name, esc_att, sizeof(esc_att));
        json_escape_into(it->message, esc_msg, sizeof(esc_msg));

        off += snprintf(resp + off, 8192 - off,
                        "%s{\"id\":\"%s\",\"ticket_code\":\"%s\",\"attendee_name\":\"%s\","
                        "\"event_code\":\"%s\",\"action\":\"%s\",\"result\":\"%s\","
                        "\"success\":%s,\"message\":\"%s\",\"scanned_at\":\"%s\"}",
                        (i > 0) ? "," : "",
                        it->id, esc_tck, esc_att, it->event_code, it->action, it->result,
                        it->success ? "true" : "false", esc_msg, it->scanned_at);
    }
    off += snprintf(resp + off, 8192 - off, "]}");
    if (g_history_mutex) xSemaphoreGive(g_history_mutex);

    httpd_resp_set_type(req, "application/json");
    send_cors_headers(req);
    esp_err_t ret = httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    return ret;
}

static const char STANDALONE_HTML[] =
"<!DOCTYPE html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
"<title>ESP32-S3 Check-in Station (Standalone)</title>"
"<style>body{background:#0f172a;color:#f8fafc;font-family:sans-serif;margin:0;padding:16px;text-align:center;}"
".box{max-width:520px;margin:0 auto;background:#1e293b;border-radius:12px;padding:16px;border:1px solid #334155;}"
"img{width:100%;border-radius:8px;aspect-ratio:4/3;background:#000;object-fit:cover;}"
".badge{display:inline-block;padding:4px 10px;border-radius:20px;font-size:12px;font-weight:700;background:rgba(16,185,129,0.2);color:#34d399;margin:8px 4px;}"
".card{background:#0f172a;border:1px solid #334155;border-radius:10px;padding:12px;margin-top:12px;text-align:left;font-size:13px;}"
".row{display:flex;align-items:center;justify-content:space-between;gap:8px;}"
".btn{background:#3b82f6;color:#fff;border:none;border-radius:8px;padding:8px 14px;font-weight:700;cursor:pointer;}"
".btn:disabled{opacity:0.4;cursor:not-allowed;}"
".btn-ok{background:#10b981;}"
".ok{border-color:#10b981;background:rgba(16,185,129,0.14);}"
"input{width:100%;box-sizing:border-box;background:#1e293b;color:#f8fafc;border:1px solid #475569;border-radius:6px;padding:7px;margin-top:6px;}"
"</style></head><body><div class='box'>"
"<h2 style='margin:0 0 10px'>EventHub ESP32-S3 (Độc Lập)</h2>"
"<img src='/stream' alt='Live Video Stream'>"
"<div><span class='badge' id='b-wifi'>Wi-Fi Đang Hoạt Động</span>"
"<span class='badge' id='b-iot' style='background:rgba(59,130,246,0.2);color:#93c5fd'>IoT: Đang kiểm tra...</span></div>"
"<div class='card' id='iot-card'><div style='color:#94a3b8;font-size:11px'>ĐỊNH DANH & SỰ KIỆN ĐƯỢC GÁN (ADMIN)</div>"
"<div id='iot-main' style='font-weight:700;font-size:14px;margin:4px 0'>MAC: --</div>"
"<div id='iot-sub' style='color:#cbd5e1;font-size:12px'>Đang đồng bộ cấu hình từ máy chủ...</div>"
"<details style='margin-top:8px'><summary style='cursor:pointer;color:#93c5fd;font-size:12px'>⚙ Cấu hình IoT Server / Reset ghép nối</summary>"
"<input id='iot-url-in' placeholder='https://.../api/iot/v1'>"
"<div class='row' style='margin-top:8px'><button class='btn' style='flex:1' onclick='saveIotUrl()'>Lưu IoT URL</button>"
"<button class='btn' style='background:#ef4444' onclick='resetIot()'>Reset ghép nối</button></div></details></div>"
"<div class='card'><label class='row' style='cursor:pointer'><span><b>⚡ Tự động xác nhận vé (Auto Check-in)</b><br>"
"<small style='color:#94a3b8'>Đồng bộ từ Admin hoặc lưu vào Flash ESP32</small></span>"
"<input type='checkbox' id='auto-chk' style='width:20px;height:20px;margin:0' onchange='setAuto(this.checked)'></label>"
"<div id='auto-msg' style='font-size:11px;color:#34d399;margin-top:4px'></div></div>"
"<div class='card' id='scan-card'><div style='color:#94a3b8;font-size:11px'>TRẠNG THÁI QUÉT ĐỘC LẬP</div>"
"<div id='scan-qr' style='font-weight:700;font-size:15px;margin:4px 0'>Chờ mã QR...</div>"
"<div id='scan-msg' style='color:#cbd5e1'>Đưa mã QR vào trước camera ESP32</div>"
"<button id='btn-ci' class='btn btn-ok' style='display:none;margin-top:10px;width:100%' onclick='manualCi()'>Xác nhận soát vé ngay</button></div>"
"<div class='card' id='hist-card'><div class='row'>"
"<div style='color:#94a3b8;font-size:11px'>📜 LỊCH SỬ QUÉT VÉ THIẾT BỊ</div>"
"<button class='btn' style='padding:3px 9px;font-size:11px' onclick='loadHist(hPage,true)'>🔄 Làm mới</button></div>"
"<div style='margin:6px 0;font-size:11px'>"
"<span class='badge' id='h-stat-ok' style='margin:2px;font-size:11px'>OK: 0</span>"
"<span class='badge' id='h-stat-err' style='background:rgba(239,68,68,0.2);color:#f87171;margin:2px;font-size:11px'>Lỗi: 0</span>"
"<span class='badge' id='h-stat-all' style='background:rgba(148,163,184,0.2);color:#cbd5e1;margin:2px;font-size:11px'>Tổng: 0</span></div>"
"<div class='row' style='margin-bottom:8px;flex-wrap:wrap'>"
"<div style='display:flex;gap:4px'>"
"<button class='btn' id='hf-all' style='padding:3px 9px;font-size:11px' onclick='setHistFilter(\"ALL\")'>Tất cả</button>"
"<button class='btn' id='hf-ok' style='background:#1e293b;border:1px solid #475569;padding:3px 9px;font-size:11px' onclick='setHistFilter(\"OK\")'>OK</button>"
"<button class='btn' id='hf-err' style='background:#1e293b;border:1px solid #475569;padding:3px 9px;font-size:11px' onclick='setHistFilter(\"FAILED\")'>Lỗi</button></div>"
"<label style='font-size:11px;color:#94a3b8;display:flex;align-items:center;gap:4px;cursor:pointer'>"
"<input type='checkbox' id='h-event-only' style='width:auto;margin:0' onchange='loadHist(1,true)'>Chỉ SK gán</label></div>"
"<div id='hist-list' style='font-size:12px;display:flex;flex-direction:column;gap:6px;min-height:36px'>"
"<div style='color:#94a3b8'>Đang tải lịch sử...</div></div>"
"<div class='row' style='margin-top:8px'>"
"<button class='btn' id='h-btn-prev' style='padding:4px 10px;font-size:11px' onclick='prevHistPage()'>◀ Trước</button>"
"<span id='h-page-info' style='color:#94a3b8;font-size:11px'>Trang 1</span>"
"<button class='btn' id='h-btn-next' style='padding:4px 10px;font-size:11px' onclick='nextHistPage()'>Sau ▶</button></div></div>"
"<div class='card'><div class='row'><div style='color:#94a3b8;font-size:11px'>🛠️ KIỂM TRA PHẦN CỨNG (TEST HARDWARE)</div>"
"<button class='btn' style='background:#d97706;padding:3px 9px;font-size:11px' onclick='testDev(\"all\")'>⚡ Test toàn bộ</button></div>"
"<div class='row' style='margin-top:8px;gap:6px'>"
"<button class='btn' style='background:#10b981;flex:1;padding:5px;font-size:11px' onclick='testDev(\"green\",1)'>🟢 LED Xanh</button>"
"<button class='btn' style='background:#ef4444;flex:1;padding:5px;font-size:11px' onclick='testDev(\"red\",1)'>🔴 LED Đỏ</button>"
"<button class='btn' style='background:#3b82f6;flex:1;padding:5px;font-size:11px' onclick='testDev(\"buzzer\")'>🔊 Còi</button>"
"<button class='btn' style='background:#8b5cf6;flex:1;padding:5px;font-size:11px' onclick='testDev(\"servo\")'>🚪 Servo 90°</button></div>"
"<div class='row' style='margin-top:6px;gap:6px'>"
"<button class='btn' style='background:#334155;flex:1;padding:4px;font-size:11px' onclick='testDev(\"green\",0);testDev(\"red\",0)'>⚫ Tắt LED</button>"
"<button class='btn' style='background:#334155;flex:1;padding:4px;font-size:11px' onclick='testDev(\"servo\",0)'>🚪 Đóng Servo 0°</button></div></div>"
"</div><script>"
"let lastQr='';let lastSeq=-1;let busyAuto=0;let iotUrlInit=false;"
"let hPage=1;let hLimit=5;let hFilter='ALL';let hHasMore=false;"
"async function testDev(d,s){try{await fetch('/api/test',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({device:d,state:(s!==undefined?s:1),angle:(s===0?0:90),duration:120})});}catch(e){}}"
"function esc(s){return String(s||'').replace(/[&<>\"']/g,m=>({'&':'&amp;','<':'&lt;','>':'&gt;','\"':'&quot;',\"'\":'&#39;'}[m]));}"
"function fmtT(s){if(!s)return '';const d=new Date(s);if(!isNaN(d.getTime()))return d.toLocaleTimeString('vi-VN',{hour:'2-digit',minute:'2-digit'})+' '+d.toLocaleDateString('vi-VN',{day:'2-digit',month:'2-digit'});return s;}"
"async function setAuto(v){busyAuto=Date.now();const r=await fetch('/api/auto-confirm',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({enabled:v})});const d=await r.json();document.getElementById('auto-msg').innerText=d.auto_confirm?'✓ ESP32 đang BẬT tự xác nhận vé (đã lưu NVS)':'ESP32 đang TẮT tự xác nhận vé (đã lưu NVS)';}"
"async function saveIotUrl(){const u=document.getElementById('iot-url-in').value.trim();if(!u)return;await fetch('/api/iot-config',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({iot_base_url:u,sync:true})});alert('Đã lưu vĩnh viễn địa chỉ URL máy chủ vào Flash ESP32!');poll();}"
"async function resetIot(){if(!confirm('Xóa device_secret hiện tại để xin mã ghép nối mới?'))return;await fetch('/api/iot-config',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({reset:true})});poll();}"
"async function manualCi(){if(!lastQr)return;const r=await fetch('/api/check-in',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({qrCode:lastQr})});const d=await r.json();alert(d.message||(d.success?'Xác nhận thành công':'Lỗi xác nhận'));poll();loadHist(1,true);}"
"async function setHistFilter(f){hFilter=f;['all','ok','err'].forEach(k=>{const b=document.getElementById('hf-'+k);const sel=(f==='ALL'&&k==='all')||(f==='OK'&&k==='ok')||(f==='FAILED'&&k==='err');b.style.background=sel?'#3b82f6':'#1e293b';b.style.border=sel?'none':'1px solid #475569';});await loadHist(1,true);}"
"async function prevHistPage(){if(hPage>1){await loadHist(hPage-1,true);}}"
"async function nextHistPage(){if(hHasMore){await loadHist(hPage+1,true);}}"
"async function loadHist(p,force){try{hPage=p||1;const eo=document.getElementById('h-event-only').checked?'true':'false';"
"const u='/api/history?page='+hPage+'&limit='+hLimit+'&result='+encodeURIComponent(hFilter)+'&event_only='+eo+(force?'&refresh=1':'');"
"const r=await fetch(u);const d=await r.json();const st=d.stats||{};"
"document.getElementById('h-stat-ok').innerText='OK: '+(st.ok_count||0);"
"document.getElementById('h-stat-err').innerText='Lỗi: '+(st.failed_count||0);"
"document.getElementById('h-stat-all').innerText='Tổng: '+(st.total_scans||0);"
"const pag=d.pagination||{};hHasMore=!!pag.has_more;hPage=pag.page||hPage;"
"document.getElementById('h-page-info').innerText='Trang '+hPage+(pag.total?(' ('+pag.total+' lượt)'):'')+(d.offline_cached?' • Cache':'');"
"document.getElementById('h-btn-prev').disabled=(hPage<=1);document.getElementById('h-btn-next').disabled=!hHasMore;"
"const items=d.items||[];const hl=document.getElementById('hist-list');"
"if(!items.length){hl.innerHTML='<div style=\"color:#94a3b8;padding:4px 0\">Chưa có lượt quét nào'+(d.last_error?(' ('+esc(d.last_error)+')'):'')+'</div>';return;}"
"hl.innerHTML=items.map(it=>{const ok=(it.result==='OK'||it.success);const ver=(it.result==='VERIFIED');"
"const col=ok?'#34d399':(ver?'#93c5fd':'#f87171');const bg=ok?'rgba(16,185,129,0.15)':(ver?'rgba(59,130,246,0.15)':'rgba(239,68,68,0.15)');"
"const code=it.ticket_code||it.message||'--';const att=it.attendee_name?('<b style=\"color:#f8fafc\">'+esc(it.attendee_name)+'</b> • '):'';"
"return '<div style=\"background:#1e293b;border-radius:6px;padding:7px 9px;border-left:3px solid '+col+'\">'"
"+'<div class=\"row\"><span><span style=\"background:'+bg+';color:'+col+';padding:1px 5px;border-radius:4px;font-weight:700;font-size:10px\">'+esc(it.result)+'</span> '"
"+'<b style=\"margin-left:4px\">'+esc(code)+'</b></span><small style=\"color:#94a3b8\">'+esc(fmtT(it.scanned_at))+'</small></div>'"
"+'<div style=\"color:#cbd5e1;font-size:11px;margin-top:3px\">'+att+esc(it.message)+'</div></div>';}).join('');}catch(e){}}"
"async function poll(){try{const r=await fetch('/status');const d=await r.json();"
"document.getElementById('b-wifi').innerText=(d.wifi_connected?('📶 '+d.ssid+' ('+d.ip+')'):'Mất Wi-Fi');"
"const iot=d.iot||{};const bi=document.getElementById('b-iot');"
"bi.innerText='IoT: '+(iot.state||'UNPAIRED')+(iot.pairing_code?(' ['+iot.pairing_code+']'):'');"
"bi.style.color=iot.paired?'#34d399':(iot.state==='DISABLED'?'#f87171':'#93c5fd');"
"document.getElementById('iot-main').innerText='MAC: '+(iot.mac||'--')+(iot.device_name?(' • '+iot.device_name):'');"
"let sub='Chờ đăng ký với máy chủ...';"
"if(iot.state==='PENDING')sub='Mã ghép nối Admin: '+(iot.pairing_code||'--')+' (vào Admin -> Thiết bị để duyệt)';"
"else if(iot.state==='REPAIR_REQUIRED')sub='Thiết bị đã từng ghép nối. Vào Admin -> Reset ghép nối hoặc bấm Reset bên dưới.';"
"else if(iot.state==='DISABLED')sub='Thiết bị đang bị Admin KHÓA.';"
"else if(iot.paired)sub=iot.assigned_event_id?('Sự kiện: '+(iot.event_name||iot.event_code)+' ['+iot.event_code+']'):'Đã kích hoạt nhưng CHƯA GÁN SỰ KIỆN trên Admin Web.';"
"document.getElementById('iot-sub').innerText=sub;"
"if(!iotUrlInit&&iot.iot_base_url){document.getElementById('iot-url-in').value=iot.iot_base_url;iotUrlInit=true;}"
"if(Date.now()-busyAuto>2000){document.getElementById('auto-chk').checked=!!d.auto_confirm;document.getElementById('auto-msg').innerText=d.auto_confirm?'✓ Đang BẬT Auto Check-in':'Đang TẮT Auto Check-in';}"
"const ls=d.last_scan||{};lastQr=ls.qr||'';document.getElementById('scan-qr').innerText=ls.qr||'Chờ mã QR...';"
"document.getElementById('scan-msg').innerText=ls.message||'Đưa mã QR vào trước camera ESP32';"
"document.getElementById('scan-card').className='card'+(ls.status==='CONFIRMED'?' ok':'');"
"document.getElementById('btn-ci').style.display=(ls.status==='VERIFIED'&&!d.auto_confirm&&ls.qr)?'block':'none';"
"if(ls.seq!==undefined&&ls.seq!==lastSeq){const firstLoad=(lastSeq===-1);lastSeq=ls.seq;if(!firstLoad)loadHist(1,false);}"
"}catch(e){}setTimeout(poll,1000);}poll();loadHist(1,false);"
"</script></body></html>";

static esp_err_t servo_control_httpd_handler(httpd_req_t *req)
{
    char content[128];
    int ret = httpd_req_recv(req, content, sizeof(content) - 1);
    int angle = -1;
    if (ret > 0) {
        content[ret] = '\0';
        extract_json_int(content, "angle", &angle);
    }

    if (angle >= 0 && angle <= 180) {
        servo_set_angle((uint32_t)angle);
    } else {
        servo_trigger_gate_open();
    }

    const char *resp = "{\"success\":true,\"message\":\"Servo gate triggered OK\"}";
    httpd_resp_set_type(req, "application/json");
    send_cors_headers(req);
    return httpd_resp_send(req, resp, strlen(resp));
}

static esp_err_t test_hardware_httpd_handler(httpd_req_t *req)
{
    char buf[256];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) {
        httpd_resp_set_type(req, "application/json");
        send_cors_headers(req);
        return httpd_resp_send(req, "{\"success\":false,\"message\":\"No body\"}", HTTPD_RESP_USE_STRLEN);
    }
    buf[ret] = '\0';

    char dev[32] = "";
    extract_json_str(buf, "device", dev, sizeof(dev));
    if (dev[0] == '\0') {
        extract_json_str(buf, "action", dev, sizeof(dev));
    }
    int state = 0;
    extract_json_int(buf, "state", &state);

    char resp[160];
    if (strcmp(dev, "green") == 0 || strcmp(dev, "led_green") == 0) {
        led_set_green_direct(state);
        snprintf(resp, sizeof(resp), "{\"success\":true,\"device\":\"green\",\"state\":%d}", state ? 1 : 0);
    } else if (strcmp(dev, "red") == 0 || strcmp(dev, "led_red") == 0) {
        led_set_red_direct(state);
        snprintf(resp, sizeof(resp), "{\"success\":true,\"device\":\"red\",\"state\":%d}", state ? 1 : 0);
    } else if (strcmp(dev, "buzzer") == 0) {
        int b_ms = 150;
        extract_json_int(buf, "duration", &b_ms);
        buzzer_beep(b_ms > 0 ? b_ms : 150);
        snprintf(resp, sizeof(resp), "{\"success\":true,\"device\":\"buzzer\",\"beep_ms\":%d}", b_ms > 0 ? b_ms : 150);
    } else if (strcmp(dev, "servo") == 0) {
        int angle = 90;
        extract_json_int(buf, "angle", &angle);
        if (angle == 90) servo_trigger_gate_open();
        else servo_set_angle((uint32_t)angle);
        snprintf(resp, sizeof(resp), "{\"success\":true,\"device\":\"servo\",\"angle\":%d}", angle);
    } else if (strcmp(dev, "ir") == 0 || strcmp(dev, "sensor") == 0) {
        bool blocked = is_ir_obstacle_detected();
        snprintf(resp, sizeof(resp), "{\"success\":true,\"device\":\"ir\",\"detected\":%s,\"state\":%d}",
                 blocked ? "true" : "false", blocked ? 1 : 0);
    } else if (strcmp(dev, "all") == 0) {
        trigger_hardware_selftest();
        snprintf(resp, sizeof(resp), "{\"success\":true,\"device\":\"all\",\"message\":\"Self-test sequence started\"}");
    } else {
        snprintf(resp, sizeof(resp), "{\"success\":false,\"message\":\"Unknown test device\"}");
    }

    httpd_resp_set_type(req, "application/json");
    send_cors_headers(req);
    return httpd_resp_send(req, resp, strlen(resp));
}

static esp_err_t index_httpd_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, STANDALONE_HTML, strlen(STANDALONE_HTML));
}

static void start_standalone_webserver(void)
{
    if (g_http_server) return;

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.ctrl_port = 32768;
    config.max_open_sockets = 6;
    config.max_uri_handlers = 28;
    config.stack_size = 12288;

    if (httpd_start(&g_http_server, &config) == ESP_OK) {
        const httpd_uri_t uris[] = {
            { .uri = "/stream",           .method = HTTP_GET,     .handler = jpg_stream_httpd_handler,     .user_ctx = NULL },
            { .uri = "/status",           .method = HTTP_GET,     .handler = status_httpd_handler,         .user_ctx = NULL },
            { .uri = "/api/iot-status",   .method = HTTP_GET,     .handler = iot_status_httpd_handler,     .user_ctx = NULL },
            { .uri = "/api/iot-status",   .method = HTTP_OPTIONS, .handler = cors_options_httpd_handler,   .user_ctx = NULL },
            { .uri = "/api/iot-config",   .method = HTTP_POST,    .handler = iot_config_httpd_handler,     .user_ctx = NULL },
            { .uri = "/api/iot-config",   .method = HTTP_OPTIONS, .handler = cors_options_httpd_handler,   .user_ctx = NULL },
            { .uri = "/api/history",      .method = HTTP_GET,     .handler = history_httpd_handler,        .user_ctx = NULL },
            { .uri = "/api/history",      .method = HTTP_OPTIONS, .handler = cors_options_httpd_handler,   .user_ctx = NULL },
            { .uri = "/api/wifi",         .method = HTTP_POST,    .handler = wifi_cfg_httpd_handler,       .user_ctx = NULL },
            { .uri = "/api/wifi",         .method = HTTP_OPTIONS, .handler = cors_options_httpd_handler,   .user_ctx = NULL },
            { .uri = "/api/wifi-profiles", .method = HTTP_POST,   .handler = wifi_profiles_httpd_handler,  .user_ctx = NULL },
            { .uri = "/api/wifi-profiles", .method = HTTP_OPTIONS,.handler = cors_options_httpd_handler,   .user_ctx = NULL },
            { .uri = "/api/verify",       .method = HTTP_POST,    .handler = verify_ticket_httpd_handler,  .user_ctx = NULL },
            { .uri = "/api/verify",       .method = HTTP_OPTIONS, .handler = cors_options_httpd_handler,   .user_ctx = NULL },
            { .uri = "/api/check-in",     .method = HTTP_POST,    .handler = checkin_ticket_httpd_handler, .user_ctx = NULL },
            { .uri = "/api/check-in",     .method = HTTP_OPTIONS, .handler = cors_options_httpd_handler,   .user_ctx = NULL },
            { .uri = "/api/auto-confirm", .method = HTTP_POST,    .handler = auto_confirm_httpd_handler,   .user_ctx = NULL },
            { .uri = "/api/auto-confirm", .method = HTTP_OPTIONS, .handler = cors_options_httpd_handler,   .user_ctx = NULL },
            { .uri = "/api/servo",        .method = HTTP_POST,    .handler = servo_control_httpd_handler,  .user_ctx = NULL },
            { .uri = "/api/servo",        .method = HTTP_OPTIONS, .handler = cors_options_httpd_handler,   .user_ctx = NULL },
            { .uri = "/api/test",         .method = HTTP_POST,    .handler = test_hardware_httpd_handler,   .user_ctx = NULL },
            { .uri = "/api/test",         .method = HTTP_OPTIONS, .handler = cors_options_httpd_handler,   .user_ctx = NULL },
            { .uri = "/",                 .method = HTTP_GET,     .handler = index_httpd_handler,          .user_ctx = NULL },
        };
        for (size_t i = 0; i < sizeof(uris) / sizeof(uris[0]); i++) {
            httpd_register_uri_handler(g_http_server, &uris[i]);
        }

        ESP_LOGI(TAG, "Standalone Web Server running at http://%s:80", g_wifi_ip);
    }
}

/* ============================================================================
 * 7. TASK TRUYỀN STREAM VIDEO TỐC ĐỘ CAO QUA USB SERIAL (CORE 1)
 * ============================================================================ */
static void camera_stream_task(void *pvParameters)
{
    uint32_t frame_seq = 0;
    ESP_LOGI(TAG, "Starting camera streaming loop @ %d baud (sw_jpeg=%d)...",
             UART_BAUD_RATE, g_camera_sw_jpeg_mode ? 1 : 0);
    vTaskDelay(pdMS_TO_TICKS(400));

    while (1) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        /* Chế độ GRAYSCALE (tự cứu hộ khi cáp camera đứt chân D1):
         * 1. Cấp ảnh xám thô không nén (640x480 lossless) trực tiếp cho quét QR độc lập -> bỏ qua esp_jpeg_decode (vốn không hỗ trợ Grayscale JPEG), tăng tốc gấp 5 lần!
         * 2. Chỉ nén SW-JPEG khi cắm USB stream lên PC (khi chạy nguồn ngoài độc lập không tốn CPU nén JPEG). */
        if (fb->format == PIXFORMAT_GRAYSCALE) {
            size_t expected_len = (size_t)fb->width * (size_t)fb->height;
            if (s_snap_requested && !s_snap_ready && s_snap_gray_buf &&
                fb->width > 0 && fb->height > 0 &&
                expected_len <= SNAP_GRAY_MAX_BYTES && fb->len >= expected_len) {
                memcpy(s_snap_gray_buf, fb->buf, expected_len);
                s_snap_gray_w = (uint16_t)fb->width;
                s_snap_gray_h = (uint16_t)fb->height;
                s_snap_is_gray = true;
                s_snap_ready = true;
                s_snap_requested = false;
            }

            if (is_pc_usb_active() || xTaskGetTickCount() < pdMS_TO_TICKS(15000)) {
                uint8_t *jpg_buf = NULL;
                size_t jpg_len = 0;
                bool ok = frame2jpg(fb, 72, &jpg_buf, &jpg_len);
                /* Trả buffer DMA ngay sau khi nén xong để cảm biến chụp khung kế tiếp song song khi truyền UART */
                esp_camera_fb_return(fb);
                fb = NULL;
                if (ok && jpg_buf && jpg_len >= 4 && jpg_buf[0] == 0xFF && jpg_buf[1] == 0xD8) {
                    uint32_t payload_len = (uint32_t)jpg_len;
                    if (g_uart_mutex) xSemaphoreTake(g_uart_mutex, portMAX_DELAY);
                    uart_write_bytes(UART_PORT_NUM, (const char *)SYNC_MAGIC, 4);
                    uart_write_bytes(UART_PORT_NUM, (const char *)&payload_len, 4);
                    uart_write_bytes(UART_PORT_NUM, (const char *)&frame_seq, 4);
                    uart_write_bytes(UART_PORT_NUM, (const char *)jpg_buf, jpg_len);
                    uart_write_bytes(UART_PORT_NUM, (const char *)END_MAGIC, 4);
                    if (g_uart_mutex) xSemaphoreGive(g_uart_mutex);
                    frame_seq++;
                }
                if (jpg_buf) free(jpg_buf);
            } else {
                esp_camera_fb_return(fb);
                fb = NULL;
            }
            int delay_ms = is_pc_usb_active() ? 1 : 25;
            vTaskDelay(pdMS_TO_TICKS(delay_ms));
            continue;
        }

        uint8_t *jpg_buf = fb->buf;
        size_t jpg_len = fb->len;
        bool jpg_alloc = false;

        if (fb->format != PIXFORMAT_JPEG) {
            jpg_buf = NULL;
            jpg_len = 0;
            if (frame2jpg(fb, 72, &jpg_buf, &jpg_len) && jpg_buf && jpg_len >= 4) {
                jpg_alloc = true;
            } else {
                if (jpg_buf) free(jpg_buf);
                jpg_buf = NULL;
                jpg_len = 0;
            }
            esp_camera_fb_return(fb);
            fb = NULL;
        }

        if (jpg_buf && jpg_len >= 4 && jpg_buf[0] == 0xFF && jpg_buf[1] == 0xD8) {
            if (s_snap_requested && !s_snap_ready && s_snap_jpg_buf && jpg_len <= SNAP_JPG_MAX_BYTES) {
                memcpy(s_snap_jpg_buf, jpg_buf, jpg_len);
                s_snap_jpg_len = jpg_len;
                s_snap_is_gray = false;
                s_snap_ready = true;
                s_snap_requested = false;
            }

            if (is_pc_usb_active() || xTaskGetTickCount() < pdMS_TO_TICKS(15000)) {
                uint32_t payload_len = (uint32_t)jpg_len;

                if (g_uart_mutex) xSemaphoreTake(g_uart_mutex, portMAX_DELAY);
                uart_write_bytes(UART_PORT_NUM, (const char *)SYNC_MAGIC, 4);
                uart_write_bytes(UART_PORT_NUM, (const char *)&payload_len, 4);
                uart_write_bytes(UART_PORT_NUM, (const char *)&frame_seq, 4);
                uart_write_bytes(UART_PORT_NUM, (const char *)jpg_buf, jpg_len);
                uart_write_bytes(UART_PORT_NUM, (const char *)END_MAGIC, 4);
                if (g_uart_mutex) xSemaphoreGive(g_uart_mutex);

                frame_seq++;
            }
        }

        if (jpg_alloc && jpg_buf) {
            free(jpg_buf);
        }
        if (fb) {
            esp_camera_fb_return(fb);
        }
        int delay_ms = (is_pc_usb_active() && g_camera_sw_jpeg_mode) ? 1 : (is_pc_usb_active() ? 5 : 12);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}

static void standalone_qr_task(void *pvParameters)
{
    uint8_t *rgb_buf = NULL;
    size_t rgb_cap = 640 * 480 * 3;
    struct quirc *qr_ctx = NULL;
    struct quirc_code *qr_code = NULL;
    struct quirc_data *qr_data = NULL;
    int cur_w = 0, cur_h = 0;

    char last_code[128] = "";
    TickType_t last_code_tick = 0;
    uint32_t last_code_cooldown_ms = 8000;
    uint32_t scan_cycle = 0;
    bool resources_ready = false;
    bool logged_gray_direct = false;

    while (1) {
        if (is_pc_usb_active() || !g_wifi_connected || !is_iot_device_paired_active()) {
            vTaskDelay(pdMS_TO_TICKS(350));
            continue;
        }

        if (!resources_ready) {
            if (!s_snap_jpg_buf) {
                s_snap_jpg_buf = (uint8_t *)heap_caps_malloc(SNAP_JPG_MAX_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            }
            if (!s_snap_gray_buf) {
                s_snap_gray_buf = (uint8_t *)heap_caps_malloc(SNAP_GRAY_MAX_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            }
            rgb_buf = (uint8_t *)heap_caps_malloc(rgb_cap, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            qr_code = (struct quirc_code *)heap_caps_calloc(1, sizeof(struct quirc_code), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            qr_data = (struct quirc_data *)heap_caps_calloc(1, sizeof(struct quirc_data), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            qr_ctx = quirc_new();

            if (!s_snap_jpg_buf || !s_snap_gray_buf || !rgb_buf || !qr_code || !qr_data || !qr_ctx) {
                ESP_LOGE(TAG, "Standalone QR PSRAM init failed!");
                vTaskDelay(pdMS_TO_TICKS(2000));
                continue;
            }
            if (quirc_resize(qr_ctx, 640, 480) < 0) {
                ESP_LOGE(TAG, "quirc_resize(640,480) failed!");
                vTaskDelay(pdMS_TO_TICKS(2000));
                continue;
            }
            cur_w = 640;
            cur_h = 480;
            resources_ready = true;
            ESP_LOGI(TAG, "Standalone onboard QR scanner armed (auto_confirm=%d, sw_gray=%d).",
                     g_auto_confirm ? 1 : 0, g_camera_sw_jpeg_mode ? 1 : 0);
        }

        s_snap_is_gray = false;
        s_snap_ready = false;
        s_snap_requested = true;
        int wait_tries = 0;
        while (!s_snap_ready && wait_tries < 40) {
            vTaskDelay(pdMS_TO_TICKS(15));
            wait_tries++;
        }
        if (!s_snap_ready || (!s_snap_is_gray && s_snap_jpg_len < 128)) {
            s_snap_requested = false;
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        int src_w = 0, src_h = 0;
        bool use_direct_gray = s_snap_is_gray;

        if (use_direct_gray) {
            src_w = (int)s_snap_gray_w;
            src_h = (int)s_snap_gray_h;
            if (src_w <= 0 || src_h <= 0 || (size_t)src_w * (size_t)src_h > SNAP_GRAY_MAX_BYTES) {
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }
            if (!logged_gray_direct) {
                logged_gray_direct = true;
                ESP_LOGI(TAG, "Standalone QR active: direct lossless GRAYSCALE %dx%d -> quirc!", src_w, src_h);
            }
        } else {
            esp_jpeg_image_cfg_t jcfg = {
                .indata = s_snap_jpg_buf,
                .indata_size = (uint32_t)s_snap_jpg_len,
                .outbuf = rgb_buf,
                .outbuf_size = (uint32_t)rgb_cap,
                .out_format = JPEG_IMAGE_FORMAT_RGB888,
                .out_scale = JPEG_IMAGE_SCALE_0,
                .flags = { .swap_color_bytes = 0 },
            };
            esp_jpeg_image_output_t jout = {0};
            if (esp_jpeg_decode(&jcfg, &jout) != ESP_OK || jout.width == 0 || jout.height == 0) {
                vTaskDelay(pdMS_TO_TICKS(180));
                continue;
            }
            src_w = (int)jout.width;
            src_h = (int)jout.height;
        }

        if (src_w != cur_w || src_h != cur_h) {
            if (quirc_resize(qr_ctx, src_w, src_h) == 0) {
                cur_w = src_w;
                cur_h = src_h;
            } else {
                vTaskDelay(pdMS_TO_TICKS(250));
                continue;
            }
        }

        int qw = 0, qh = 0;
        uint8_t *gray = quirc_begin(qr_ctx, &qw, &qh);
        if (gray) {
            bool invert_pass = ((scan_cycle++ & 1) != 0);
            size_t total_px = (size_t)src_w * (size_t)src_h;
            if (use_direct_gray) {
                if (!invert_pass) {
                    memcpy(gray, s_snap_gray_buf, total_px);
                } else {
                    for (size_t i = 0; i < total_px; i++) {
                        gray[i] = (uint8_t)(255U - s_snap_gray_buf[i]);
                    }
                }
            } else {
                const uint8_t *rgb = rgb_buf;
                for (size_t i = 0; i < total_px; i++) {
                    uint8_t lum = (uint8_t)((77U * rgb[0] + 150U * rgb[1] + 29U * rgb[2]) >> 8);
                    gray[i] = invert_pass ? (uint8_t)(255U - lum) : lum;
                    rgb += 3;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(5));
            quirc_end(qr_ctx);

            int count = quirc_count(qr_ctx);
            char detected_code[128] = "";
            for (int c_idx = 0; c_idx < count; c_idx++) {
                quirc_extract(qr_ctx, c_idx, qr_code);
                quirc_decode_error_t qerr = quirc_decode(qr_code, qr_data);
                if (qerr != QUIRC_SUCCESS) {
                    quirc_flip(qr_code);
                    qerr = quirc_decode(qr_code, qr_data);
                }
                if (qerr == QUIRC_SUCCESS && qr_data->payload_len > 0) {
                    int plen = qr_data->payload_len;
                    if (plen >= QUIRC_MAX_PAYLOAD) plen = QUIRC_MAX_PAYLOAD - 1;
                    qr_data->payload[plen] = '\0';
                    extract_ticket_code_c((const char *)qr_data->payload, detected_code, sizeof(detected_code));
                    if (detected_code[0] != '\0') break;
                }
            }

            if (detected_code[0] != '\0') {
                TickType_t now_t = xTaskGetTickCount();
                bool in_cooldown = (strcmp(detected_code, last_code) == 0) &&
                                   ((now_t - last_code_tick) < pdMS_TO_TICKS(last_code_cooldown_ms));
                if (!in_cooldown && !is_pc_usb_active()) {
                    snprintf(last_code, sizeof(last_code), "%s", detected_code);
                    last_code_tick = now_t;

                    char clean_code[64] = "";
                    extract_ticket_code_c(detected_code, clean_code, sizeof(clean_code));
                    if (clean_code[0] == '\0') {
                        snprintf(clean_code, sizeof(clean_code), "%s", detected_code);
                    }

                    if (!is_valid_iot_url(g_iot_base_url)) {
                        ESP_LOGW(TAG, "[STANDALONE QR] API URL not configured or invalid: '%s'", g_iot_base_url);
                        oled_show_ticket_status(OLED_TICKET_NET_ERR, clean_code, "CHUA CAU HINH URL");
                        buzzer_beep(240);
                        led_trigger_red_error();
                        last_code_cooldown_ms = 4000;
                    } else {
                        ESP_LOGI(TAG, "[STANDALONE QR] Detected '%s' (iot=%d, auto_confirm=%d)",
                                 detected_code, is_iot_device_paired_active() ? 1 : 0, g_auto_confirm ? 1 : 0);
                        oled_show_ticket_status(OLED_TICKET_CHECKING, clean_code, NULL);
                        char *v_resp = (char *)malloc(4096);
                        if (v_resp) {
                            v_resp[0] = '\0';
                            int http_code = -1;
                            bool net_err = false;
                            esp32_iot_scan_ticket(detected_code, "auto", v_resp, 4096, &http_code);
                            process_iot_scan_response(detected_code, clean_code, v_resp, http_code, &net_err);
                            char *uart_line = (char *)malloc(4120);
                            if (uart_line) {
                                for (char *rp = v_resp; *rp; rp++) {
                                    if (*rp == '\r' || *rp == '\n') *rp = ' ';
                                }
                                snprintf(uart_line, 4120, "\nCHECKINRESP:%s\n", v_resp);
                                safe_uart_write_string(uart_line);
                                free(uart_line);
                            }
                            last_code_cooldown_ms = net_err ? 2500 : 8000;
                            free(v_resp);
                        }
                    }
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(120));
    }
}

static void iot_wait_interruptible_ms(uint32_t total_ms)
{
    uint32_t elapsed = 0;
    while (elapsed < total_ms) {
        if (g_iot_force_sync) {
            g_iot_force_sync = false;
            return;
        }
        uint32_t step = (total_ms - elapsed) > 400 ? 400 : (total_ms - elapsed);
        vTaskDelay(pdMS_TO_TICKS(step));
        elapsed += step;
    }
}

static void apply_iot_server_config_from_json(const char *json, bool from_scan)
{
    if (!json) return;

    char evt_id[64] = "";
    char evt_code[64] = "";
    char evt_name[96] = "";
    char dev_name[64] = "";
    bool has_evt = false;
    bool evt_cleared = false;
    bool auto_val = g_auto_confirm;
    bool has_auto = extract_json_bool(json, "auto_confirm", &auto_val);
    int cfg_ver = g_iot_config_version;
    bool has_ver = extract_json_int(json, "config_version", &cfg_ver);

    if (is_json_null_val(json, "assigned_event_id")) {
        evt_cleared = true;
    } else if (extract_json_str(json, "assigned_event_id", evt_id, sizeof(evt_id))) {
        has_evt = true;
        if (evt_id[0] == '\0') evt_cleared = true;
    }
    extract_json_str(json, "event_code", evt_code, sizeof(evt_code));
    extract_json_str(json, "event_name", evt_name, sizeof(evt_name));
    if (!extract_json_str(json, "device_name", dev_name, sizeof(dev_name))) {
        extract_json_str(json, "name", dev_name, sizeof(dev_name));
    }

    bool changed = false;
    bool allow_auto_override = !(from_scan && g_iot_pending_auto_sync);
    if (g_iot_mutex) xSemaphoreTake(g_iot_mutex, portMAX_DELAY);
    if (evt_cleared) {
        if (g_iot_assigned_event_id[0] != '\0' || g_iot_event_code[0] != '\0') changed = true;
        g_iot_assigned_event_id[0] = '\0';
        g_iot_event_code[0] = '\0';
        g_iot_event_name[0] = '\0';
        save_iot_nvs_str("evt_id", "");
        save_iot_nvs_str("evt_code", "");
        save_iot_nvs_str("evt_name", "");
    } else if (has_evt) {
        if (strcmp(g_iot_assigned_event_id, evt_id) != 0 ||
            strcmp(g_iot_event_code, evt_code) != 0 ||
            strcmp(g_iot_event_name, evt_name) != 0) {
            changed = true;
            snprintf(g_iot_assigned_event_id, sizeof(g_iot_assigned_event_id), "%s", evt_id);
            snprintf(g_iot_event_code, sizeof(g_iot_event_code), "%s", evt_code);
            snprintf(g_iot_event_name, sizeof(g_iot_event_name), "%s", evt_name);
            save_iot_nvs_str("evt_id", g_iot_assigned_event_id);
            save_iot_nvs_str("evt_code", g_iot_event_code);
            save_iot_nvs_str("evt_name", g_iot_event_name);
        }
    }
    if (dev_name[0] != '\0' && strcmp(g_iot_device_name, dev_name) != 0) {
        snprintf(g_iot_device_name, sizeof(g_iot_device_name), "%s", dev_name);
        save_iot_nvs_str("dev_name", g_iot_device_name);
        changed = true;
    }
    if (allow_auto_override && has_ver && g_iot_config_version != cfg_ver) {
        g_iot_config_version = cfg_ver;
        save_iot_nvs_i32("cfg_ver", cfg_ver);
        changed = true;
    }
    if (g_iot_mutex) xSemaphoreGive(g_iot_mutex);

    if (allow_auto_override && has_auto && g_auto_confirm != auto_val) {
        g_auto_confirm = auto_val;
        save_nvs_u8("auto_confirm", auto_val ? 1 : 0);
        changed = true;
    }
    if (changed) {
        ESP_LOGI(TAG, "IoT Config synced: event=%s (%s), auto_confirm=%d, cfg_ver=%d",
                 g_iot_event_code[0] ? g_iot_event_code : "NONE",
                 g_iot_assigned_event_id[0] ? g_iot_assigned_event_id : "-",
                 g_auto_confirm ? 1 : 0, g_iot_config_version);
        sync_oled_network_state();
    }
}

static void iot_device_task(void *pvParameters)
{
    (void)pvParameters;
    char *resp = (char *)malloc(2048);
    char *body = (char *)malloc(512);
    if (!resp || !body) {
        free(resp);
        free(body);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "IoT Device Task started (MAC=%s, server=%s, paired=%d)",
             g_device_mac, g_iot_base_url, is_iot_device_paired_active() ? 1 : 0);

    while (1) {
        if (!g_wifi_connected || !is_valid_iot_url(g_iot_base_url)) {
            iot_wait_interruptible_ms(1500);
            continue;
        }

        if (is_iot_device_paired_active()) {
            char url[256], dev_id[64], dev_sec[160];
            build_iot_url("/devices/heartbeat", url, sizeof(url));

            bool send_override = false;
            bool desired_auto = false;
            int base_ver = 0;
            if (g_iot_mutex) xSemaphoreTake(g_iot_mutex, portMAX_DELAY);
            snprintf(dev_id, sizeof(dev_id), "%s", g_iot_device_id);
            snprintf(dev_sec, sizeof(dev_sec), "%s", g_iot_device_secret);
            send_override = g_iot_pending_auto_sync;
            desired_auto = g_auto_confirm;
            base_ver = g_iot_config_version;
            if (g_iot_mutex) xSemaphoreGive(g_iot_mutex);

            uint32_t free_heap = (uint32_t)esp_get_free_heap_size();
            uint32_t uptime_sec = (uint32_t)(esp_timer_get_time() / 1000000ULL);
            if (send_override) {
                snprintf(body, 512,
                         "{\"rssi\":%d,\"ip\":\"%s\",\"free_heap\":%lu,\"uptime_sec\":%lu,\"firmware_version\":\"%s\","
                         "\"auto_confirm_override\":%s,\"base_config_version\":%d}",
                         (int)g_wifi_rssi, g_wifi_ip, (unsigned long)free_heap,
                         (unsigned long)uptime_sec, EVENTHUB_FIRMWARE_VERSION,
                         desired_auto ? "true" : "false", base_ver);
            } else {
                snprintf(body, 512,
                         "{\"rssi\":%d,\"ip\":\"%s\",\"free_heap\":%lu,\"uptime_sec\":%lu,\"firmware_version\":\"%s\"}",
                         (int)g_wifi_rssi, g_wifi_ip, (unsigned long)free_heap,
                         (unsigned long)uptime_sec, EVENTHUB_FIRMWARE_VERSION);
            }

            int http_code = -1;
            resp[0] = '\0';
            esp_err_t err = esp32_http_json_request(HTTP_METHOD_POST, url, body,
                                                    NULL, NULL, dev_id, dev_sec,
                                                    resp, 2048, &http_code);
            if (err == ESP_OK && (http_code == 200 || http_code == 201)) {
                bool toggled_midflight = (send_override && g_auto_confirm != desired_auto);
                bool cfg_applied = false;
                bool has_applied = extract_json_bool(resp, "config_applied", &cfg_applied);

                if (g_iot_mutex) xSemaphoreTake(g_iot_mutex, portMAX_DELAY);
                g_iot_state = IOT_STATE_ACTIVE;
                g_iot_last_err[0] = '\0';
                if (send_override && (!toggled_midflight || (has_applied && !cfg_applied))) {
                    g_iot_pending_auto_sync = false;
                    save_iot_nvs_i32("pend_auto", 0);
                }
                if (g_iot_mutex) xSemaphoreGive(g_iot_mutex);

                if (send_override) {
                    ESP_LOGI(TAG, "IoT 2-way auto_confirm sync -> override=%s, base_ver=%d, config_applied=%d",
                             desired_auto ? "true" : "false", base_ver, has_applied ? (cfg_applied ? 1 : 0) : -1);
                }

                if (!toggled_midflight || (has_applied && !cfg_applied)) {
                    apply_iot_server_config_from_json(resp, false);
                    sync_oled_network_state();
                    if (!s_dev_history.initial_synced) {
                        if (fetchDeviceHistory(1, 5, "ALL", false) == ESP_OK) {
                            s_dev_history.initial_synced = true;
                        }
                    }
                    iot_wait_interruptible_ms(10000);
                } else {
                    iot_wait_interruptible_ms(200);
                }
            } else if (http_code == 403) {
                char msg[96] = "Thiet bi bi khoa hoac thu hoi";
                extract_json_str(resp, "message", msg, sizeof(msg));
                if (g_iot_mutex) xSemaphoreTake(g_iot_mutex, portMAX_DELAY);
                g_iot_state = IOT_STATE_DISABLED;
                snprintf(g_iot_last_err, sizeof(g_iot_last_err), "%s", msg);
                if (g_iot_mutex) xSemaphoreGive(g_iot_mutex);
                sync_oled_network_state();
                iot_wait_interruptible_ms(15000);
            } else if (http_code == 401) {
                ESP_LOGW(TAG, "IoT Heartbeat HTTP 401 -> Secret invalid/reset, returning to pairing");
                clear_iot_credentials_nvs(true);
                sync_oled_network_state();
                iot_wait_interruptible_ms(1500);
            } else {
                char msg[96] = "Loi heartbeat server";
                extract_json_str(resp, "message", msg, sizeof(msg));
                if (g_iot_mutex) xSemaphoreTake(g_iot_mutex, portMAX_DELAY);
                snprintf(g_iot_last_err, sizeof(g_iot_last_err), "%s", msg);
                if (g_iot_mutex) xSemaphoreGive(g_iot_mutex);
                iot_wait_interruptible_ms(6000);
            }
            continue;
        }

        /* Unpaired / Pending pairing flow */
        bool has_claim = false;
        char claim_tok[160] = "";
        if (g_iot_mutex) xSemaphoreTake(g_iot_mutex, portMAX_DELAY);
        has_claim = (g_iot_claim_token[0] != '\0');
        snprintf(claim_tok, sizeof(claim_tok), "%s", g_iot_claim_token);
        if (g_iot_mutex) xSemaphoreGive(g_iot_mutex);

        if (!has_claim) {
            char url[256];
            build_iot_url("/devices/register", url, sizeof(url));
            snprintf(body, 512,
                     "{\"mac_address\":\"%s\",\"hardware\":\"%s\",\"firmware_version\":\"%s\"}",
                     g_device_mac, EVENTHUB_DEVICE_HARDWARE, EVENTHUB_FIRMWARE_VERSION);

            int http_code = -1;
            resp[0] = '\0';
            esp_err_t err = esp32_http_json_request(HTTP_METHOD_POST, url, body,
                                                    NULL, NULL, NULL, NULL,
                                                    resp, 2048, &http_code);
            if (err == ESP_OK && (http_code == 200 || http_code == 201)) {
                char dev_id[64] = "", pcode[16] = "", ctok[160] = "";
                int poll_sec = 5;
                extract_json_str(resp, "device_id", dev_id, sizeof(dev_id));
                extract_json_str(resp, "pairing_code", pcode, sizeof(pcode));
                extract_json_str(resp, "claim_token", ctok, sizeof(ctok));
                extract_json_int(resp, "poll_interval_sec", &poll_sec);
                if (poll_sec < 3) poll_sec = 3;
                if (poll_sec > 30) poll_sec = 30;

                if (ctok[0] != '\0') {
                    if (g_iot_mutex) xSemaphoreTake(g_iot_mutex, portMAX_DELAY);
                    snprintf(g_iot_device_id, sizeof(g_iot_device_id), "%s", dev_id);
                    snprintf(g_iot_pairing_code, sizeof(g_iot_pairing_code), "%s", pcode);
                    snprintf(g_iot_claim_token, sizeof(g_iot_claim_token), "%s", ctok);
                    g_iot_poll_interval_sec = poll_sec;
                    g_iot_state = IOT_STATE_PENDING;
                    g_iot_last_err[0] = '\0';
                    if (g_iot_mutex) xSemaphoreGive(g_iot_mutex);
                    ESP_LOGI(TAG, "IoT Pairing Pending! MAC=%s, Code=%s", g_device_mac, pcode);
                    sync_oled_network_state();
                    iot_wait_interruptible_ms((uint32_t)poll_sec * 1000U);
                    continue;
                }
            } else if (http_code == 409) {
                if (g_iot_mutex) xSemaphoreTake(g_iot_mutex, portMAX_DELAY);
                g_iot_state = IOT_STATE_REPAIR_REQUIRED;
                snprintf(g_iot_last_err, sizeof(g_iot_last_err), "Can Admin bam Reset ghep noi");
                if (g_iot_mutex) xSemaphoreGive(g_iot_mutex);
                sync_oled_network_state();
                iot_wait_interruptible_ms(12000);
                continue;
            } else if (http_code == 403) {
                if (g_iot_mutex) xSemaphoreTake(g_iot_mutex, portMAX_DELAY);
                g_iot_state = IOT_STATE_DISABLED;
                snprintf(g_iot_last_err, sizeof(g_iot_last_err), "Thiet bi dang bi Admin khoa");
                if (g_iot_mutex) xSemaphoreGive(g_iot_mutex);
                sync_oled_network_state();
                iot_wait_interruptible_ms(15000);
                continue;
            } else {
                char msg[96] = "Cho server /api/iot/v1";
                extract_json_str(resp, "message", msg, sizeof(msg));
                if (g_iot_mutex) xSemaphoreTake(g_iot_mutex, portMAX_DELAY);
                snprintf(g_iot_last_err, sizeof(g_iot_last_err), "%s", msg);
                if (g_iot_mutex) xSemaphoreGive(g_iot_mutex);
                sync_oled_network_state();
                iot_wait_interruptible_ms(10000);
                continue;
            }
        } else {
            char url[320];
            char path[96];
            snprintf(path, sizeof(path), "/devices/pairing-status?mac_address=%s", g_device_mac);
            build_iot_url(path, url, sizeof(url));

            int http_code = -1;
            resp[0] = '\0';
            esp_err_t err = esp32_http_json_request(HTTP_METHOD_GET, url, NULL,
                                                    NULL, claim_tok, NULL, NULL,
                                                    resp, 2048, &http_code);
            if (err == ESP_OK && http_code == 200) {
                char status_str[24] = "";
                char dev_id[64] = "";
                char dev_sec[160] = "";
                extract_json_str(resp, "status", status_str, sizeof(status_str));
                extract_json_str(resp, "device_id", dev_id, sizeof(dev_id));
                extract_json_str(resp, "device_secret", dev_sec, sizeof(dev_sec));

                if (strcmp(status_str, "ACTIVE") == 0 && dev_sec[0] != '\0') {
                    if (g_iot_mutex) xSemaphoreTake(g_iot_mutex, portMAX_DELAY);
                    if (dev_id[0] != '\0') {
                        snprintf(g_iot_device_id, sizeof(g_iot_device_id), "%s", dev_id);
                    }
                    snprintf(g_iot_device_secret, sizeof(g_iot_device_secret), "%s", dev_sec);
                    g_iot_claim_token[0] = '\0';
                    g_iot_pairing_code[0] = '\0';
                    g_iot_state = IOT_STATE_ACTIVE;
                    g_iot_last_err[0] = '\0';
                    if (g_iot_mutex) xSemaphoreGive(g_iot_mutex);

                    save_iot_nvs_str("dev_id", g_iot_device_id);
                    save_iot_nvs_str("dev_secret", g_iot_device_secret);
                    apply_iot_server_config_from_json(resp, false);
                    sync_oled_network_state();
                    if (fetchDeviceHistory(1, 5, "ALL", false) == ESP_OK) {
                        s_dev_history.initial_synced = true;
                    }

                    ESP_LOGI(TAG, "IoT Pairing COMPLETED! device_id=%s", g_iot_device_id);
                    buzzer_beep(65);
                    vTaskDelay(pdMS_TO_TICKS(100));
                    buzzer_beep(90);
                    iot_wait_interruptible_ms(1000);
                    continue;
                } else if (strcmp(status_str, "DISABLED") == 0 || strcmp(status_str, "REVOKED") == 0) {
                    if (g_iot_mutex) xSemaphoreTake(g_iot_mutex, portMAX_DELAY);
                    g_iot_claim_token[0] = '\0';
                    g_iot_pairing_code[0] = '\0';
                    g_iot_state = IOT_STATE_DISABLED;
                    if (g_iot_mutex) xSemaphoreGive(g_iot_mutex);
                    sync_oled_network_state();
                    iot_wait_interruptible_ms(15000);
                    continue;
                }
            } else if (http_code == 401 || http_code == 404 || http_code == 410) {
                if (g_iot_mutex) xSemaphoreTake(g_iot_mutex, portMAX_DELAY);
                g_iot_claim_token[0] = '\0';
                g_iot_pairing_code[0] = '\0';
                g_iot_state = IOT_STATE_UNPAIRED;
                if (g_iot_mutex) xSemaphoreGive(g_iot_mutex);
                iot_wait_interruptible_ms(2000);
                continue;
            }
        }

        iot_wait_interruptible_ms((uint32_t)g_iot_poll_interval_sec * 1000U);
    }
}

/* ============================================================================
 * 8. TASK NHẬN LỆNH & GỬI TELEMETRY QUA UART (CORE 0)
 * ============================================================================ */
static esp_err_t perform_sta_wifi_scan(wifi_ap_record_t **out_records, uint16_t *out_count)
{
    if (!out_records || !out_count) return ESP_ERR_INVALID_ARG;
    *out_records = NULL;
    *out_count = 0;

    esp_wifi_scan_stop();
    if (!g_wifi_connected) {
        bool prev_reconf = g_wifi_is_reconfiguring;
        g_wifi_is_reconfiguring = true;
        esp_wifi_disconnect();
        vTaskDelay(pdMS_TO_TICKS(150));
        g_wifi_connecting_now = false;
        g_wifi_is_reconfiguring = prev_reconf;
    } else {
        vTaskDelay(pdMS_TO_TICKS(40));
    }

    wifi_scan_config_t scan_config = {
        .ssid = NULL,
        .bssid = NULL,
        .channel = 0,
        .show_hidden = false,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time.active.min = 120,
        .scan_time.active.max = 250
    };
    esp_err_t sc_err = esp_wifi_scan_start(&scan_config, true);
    if (sc_err == ESP_ERR_WIFI_STATE) {
        bool prev_reconf = g_wifi_is_reconfiguring;
        g_wifi_is_reconfiguring = true;
        esp_wifi_scan_stop();
        esp_wifi_disconnect();
        vTaskDelay(pdMS_TO_TICKS(250));
        g_wifi_connecting_now = false;
        g_wifi_is_reconfiguring = prev_reconf;
        sc_err = esp_wifi_scan_start(&scan_config, true);
    }
    if (sc_err != ESP_OK) {
        return sc_err;
    }

    uint16_t ap_num = 0;
    esp_wifi_scan_get_ap_num(&ap_num);
    if (ap_num > 20) ap_num = 20;
    if (ap_num == 0) {
        return ESP_OK;
    }

    wifi_ap_record_t *ap_records = (wifi_ap_record_t *)malloc(sizeof(wifi_ap_record_t) * ap_num);
    if (!ap_records) {
        return ESP_ERR_NO_MEM;
    }
    esp_err_t rec_err = esp_wifi_scan_get_ap_records(&ap_num, ap_records);
    if (rec_err != ESP_OK) {
        free(ap_records);
        return rec_err;
    }
    *out_records = ap_records;
    *out_count = ap_num;
    return ESP_OK;
}

static void uart_cmd_task(void *pvParameters)
{
    uint8_t rx_buf[256];
    static char s_cmd_str[2048];
    int cmd_idx = 0;
    uint32_t last_telemetry_tick = 0;

    sensor_t *s = esp_camera_sensor_get();

    while (1) {
        if (g_wifi_just_connected_led) {
            g_wifi_just_connected_led = false;
            if (!is_pc_usb_active()) {
                set_led_duty(45);
                vTaskDelay(pdMS_TO_TICKS(120));
                set_led_duty(0);
            }
        }

        uint32_t now = xTaskGetTickCount();
        if (g_wifi_connecting_now && (now - g_wifi_last_attempt_tick) > pdMS_TO_TICKS(12000)) {
            g_wifi_connecting_now = false;
            g_wifi_user_connecting = false;
            g_wifi_bg_fails++;
        }

        if (!g_wifi_connected && !g_wifi_is_reconfiguring && !g_wifi_user_connecting &&
            !g_wifi_connecting_now && (now - g_wifi_last_attempt_tick) > pdMS_TO_TICKS(4000)) {
            if (g_wifi_profile_count > 1 && g_wifi_bg_fails >= 1) {
                wifi_ap_record_t *ap_records = NULL;
                uint16_t ap_num = 0;
                if (perform_sta_wifi_scan(&ap_records, &ap_num) == ESP_OK && ap_records && ap_num > 0) {
                    int best_idx = -1;
                    int best_rssi = -128;
                    int alt_idx = -1;
                    int alt_rssi = -128;
                    for (int p = 0; p < g_wifi_profile_count; p++) {
                        if (g_wifi_profiles[p].ssid[0] == '\0') continue;
                        for (uint16_t a = 0; a < ap_num; a++) {
                            if (strcmp((const char *)ap_records[a].ssid, g_wifi_profiles[p].ssid) == 0) {
                                int rssi = (int)ap_records[a].rssi;
                                if (rssi > best_rssi) {
                                    best_rssi = rssi;
                                    best_idx = p;
                                }
                                if (strcmp(g_wifi_profiles[p].ssid, g_wifi_ssid) != 0 && rssi > alt_rssi) {
                                    alt_rssi = rssi;
                                    alt_idx = p;
                                }
                            }
                        }
                    }
                    int chosen = best_idx;
                    if (chosen >= 0 && strcmp(g_wifi_profiles[chosen].ssid, g_wifi_ssid) == 0 &&
                        alt_idx >= 0 && g_wifi_bg_fails >= 2) {
                        chosen = alt_idx;
                    }
                    if (chosen >= 0 && strcmp(g_wifi_profiles[chosen].ssid, g_wifi_ssid) != 0) {
                        wifi_config_t wcfg;
                        snprintf(g_wifi_ssid, sizeof(g_wifi_ssid), "%s", g_wifi_profiles[chosen].ssid);
                        snprintf(g_wifi_pass, sizeof(g_wifi_pass), "%s", g_wifi_profiles[chosen].pass);
                        fill_sta_wifi_config(&wcfg, g_wifi_ssid, g_wifi_pass);
                        esp_wifi_set_config(WIFI_IF_STA, &wcfg);
                        ESP_LOGI(TAG, "Auto-switching to saved Wi-Fi '%s' (RSSI %d)",
                                 g_wifi_ssid, (chosen == alt_idx) ? alt_rssi : best_rssi);
                    }
                }
                free(ap_records);
                g_wifi_retry_count = 0;
                if (g_wifi_ssid[0] != '\0') {
                    trigger_sta_connect();
                } else {
                    g_wifi_last_attempt_tick = xTaskGetTickCount();
                }
            } else if (g_wifi_ssid[0] != '\0') {
                ESP_LOGI(TAG, "Background Wi-Fi retry (%lu) for '%s'", (unsigned long)g_wifi_bg_fails, g_wifi_ssid);
                g_wifi_retry_count = 0;
                trigger_sta_connect();
            } else {
                g_wifi_last_attempt_tick = now;
            }
        }

        // Gửi telemetry ESPSTAT mỗi 2 giây
        now = xTaskGetTickCount();
        if ((now - last_telemetry_tick) > pdMS_TO_TICKS(2000)) {
            last_telemetry_tick = now;
            if (g_wifi_connected) {
                wifi_ap_record_t ap_info;
                if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
                    g_wifi_rssi = ap_info.rssi;
                }
            }
            char tele_buf[256];
            snprintf(tele_buf, sizeof(tele_buf), "ESPSTAT:WIFI=%d,IP=%s,RSSI=%d,SSID=%s,AUTO=%d,TOKEN=%d,PROFILES=%d\n",
                     g_wifi_connected ? 1 : 0, g_wifi_ip, (int)g_wifi_rssi, g_wifi_ssid,
                     g_auto_confirm ? 1 : 0, is_iot_device_paired_active() ? 1 : 0, g_wifi_profile_count);
            safe_uart_write_string(tele_buf);

            char iot_tele[512];
            char iot_creds[320] = "";
            if (g_iot_mutex) xSemaphoreTake(g_iot_mutex, portMAX_DELAY);
            snprintf(iot_tele, sizeof(iot_tele),
                     "IOTSTAT:MAC=%s|STATE=%s|CODE=%s|EVT=%s|NAME=%s|URL=%s|VER=%d|PEND=%d\n",
                     g_device_mac,
                     iot_state_to_str(g_iot_state),
                     g_iot_pairing_code,
                     g_iot_event_code,
                     g_iot_device_name,
                     g_iot_base_url,
                     g_iot_config_version,
                     g_iot_pending_auto_sync ? 1 : 0);
            if (g_iot_state == IOT_STATE_ACTIVE && g_iot_device_id[0] != '\0' && g_iot_device_secret[0] != '\0') {
                snprintf(iot_creds, sizeof(iot_creds), "IOTCREDS:ID=%s|SEC=%s\n",
                         g_iot_device_id, g_iot_device_secret);
            }
            if (g_iot_mutex) xSemaphoreGive(g_iot_mutex);
            safe_uart_write_string(iot_tele);
            if (iot_creds[0] != '\0') {
                safe_uart_write_string(iot_creds);
            }

            sync_oled_network_state();
        }

        /* Đọc ngay lập tức dữ liệu đang có trong UART RX để không bị trễ timeout 80ms mỗi lệnh */
        size_t buffered = 0;
        int len = 0;
        uart_get_buffered_data_len(UART_PORT_NUM, &buffered);
        if (buffered > 0) {
            size_t to_read = buffered < (sizeof(rx_buf) - 1) ? buffered : (sizeof(rx_buf) - 1);
            len = uart_read_bytes(UART_PORT_NUM, rx_buf, to_read, 0);
        } else {
            len = uart_read_bytes(UART_PORT_NUM, rx_buf, 1, pdMS_TO_TICKS(40));
            if (len == 1) {
                uart_get_buffered_data_len(UART_PORT_NUM, &buffered);
                if (buffered > 0) {
                    size_t extra = buffered < (sizeof(rx_buf) - 2) ? buffered : (sizeof(rx_buf) - 2);
                    int len2 = uart_read_bytes(UART_PORT_NUM, rx_buf + 1, extra, 0);
                    if (len2 > 0) len += len2;
                }
            }
        }
        if (len > 0) {
            for (int i = 0; i < len; i++) {
                char c = (char)rx_buf[i];
                if (c == '\n' || c == '\r') {
                    if (cmd_idx > 0) {
                        s_cmd_str[cmd_idx] = '\0';
                        g_pc_usb_last_tick = xTaskGetTickCount();
                        int val = 0;

                        char *setwifi_p = strstr(s_cmd_str, "SETWIFI:");
                        char *savewifi_p = strstr(s_cmd_str, "SAVEWIFI:");
                        char *verify_p = strstr(s_cmd_str, "VERIFY:");
                        char *checkin_p = strstr(s_cmd_str, "CHECKIN:");
                        char *oled_p = strstr(s_cmd_str, "OLED:");

                        if (setwifi_p) {
                            char *ssid_p = setwifi_p + 8;
                            char *pass_p = strchr(ssid_p, '|');
                            if (pass_p) {
                                *pass_p = '\0';
                                pass_p++;
                                safe_uart_write_string("WIFIRESP:CONNECTING\n");
                                connect_to_new_wifi(ssid_p, pass_p);
                            }
                        } else if (savewifi_p) {
                            char *ssid_p = savewifi_p + 9;
                            char *pass_p = strchr(ssid_p, '|');
                            if (pass_p) {
                                *pass_p = '\0';
                                pass_p++;
                                bool had_ssid = (g_wifi_ssid[0] != '\0');
                                add_or_promote_wifi_profile(ssid_p, pass_p, false);
                                if (!had_ssid && g_wifi_ssid[0] != '\0' && !g_wifi_connected && !g_wifi_connecting_now) {
                                    wifi_config_t wcfg;
                                    fill_sta_wifi_config(&wcfg, g_wifi_ssid, g_wifi_pass);
                                    esp_wifi_set_config(WIFI_IF_STA, &wcfg);
                                    trigger_sta_connect();
                                }
                                char rep[48];
                                snprintf(rep, sizeof(rep), "WIFISAVED:%d\n", g_wifi_profile_count);
                                safe_uart_write_string(rep);
                            }
                        } else if (strstr(s_cmd_str, "GETWIFI?")) {
                            char rep[128];
                            snprintf(rep, sizeof(rep), "WIFISTAT:%s|%s|%d|%s\n",
                                     g_wifi_connected ? "CONNECTED" : "DISCONNECTED",
                                     g_wifi_ip, (int)g_wifi_rssi, g_wifi_ssid);
                            safe_uart_write_string(rep);
                        } else if (strstr(s_cmd_str, "SCANWIFI")) {
                            ESP_LOGI(TAG, "UART: Received SCANWIFI request (wifi_connected=%d)", g_wifi_connected);
                            wifi_ap_record_t *ap_records = NULL;
                            uint16_t ap_num = 0;
                            esp_err_t sc_err = perform_sta_wifi_scan(&ap_records, &ap_num);
                            if (sc_err == ESP_OK) {
                                char *rep = (char *)malloc(1536);
                                if (rep) {
                                    strcpy(rep, "WIFISCAN:");
                                    int written_count = 0;
                                    for (int a = 0; a < ap_num; a++) {
                                        if (strlen((char *)ap_records[a].ssid) == 0) continue;
                                        char item[80];
                                        snprintf(item, sizeof(item), "%s%s|%d", (written_count > 0 ? "," : ""), (char*)ap_records[a].ssid, (int)ap_records[a].rssi);
                                        if (strlen(rep) + strlen(item) < 1530) {
                                            strcat(rep, item);
                                            written_count++;
                                        }
                                    }
                                    strcat(rep, "\n");
                                    safe_uart_write_string(rep);
                                    free(rep);
                                }
                                free(ap_records);
                            } else {
                                ESP_LOGW(TAG, "esp_wifi_scan_start returned error: %s", esp_err_to_name(sc_err));
                                char err_rep[96];
                                snprintf(err_rep, sizeof(err_rep), "WIFISCAN:ERROR|%s\n", esp_err_to_name(sc_err));
                                safe_uart_write_string(err_rep);
                            }
                            if (!g_wifi_connected && strlen(g_wifi_ssid) > 0 && !g_wifi_is_reconfiguring) {
                                g_wifi_retry_count = 0;
                                trigger_sta_connect();
                            }
                        } else if (verify_p || checkin_p) {
                            bool is_checkin = (checkin_p != NULL);
                            char *qr_p = is_checkin ? (checkin_p + 8) : (verify_p + 7);
                            char *tok_p = strchr(qr_p, '|');
                            if (tok_p) {
                                *tok_p = '\0';
                            }
                            char clean_qr[64] = "";
                            extract_ticket_code_c(qr_p, clean_qr, sizeof(clean_qr));
                            oled_show_ticket_status(OLED_TICKET_CHECKING, clean_qr, NULL);

                            char *api_resp = (char *)malloc(4096);
                            char *uart_line = (char *)malloc(4120);
                            if (api_resp && uart_line) {
                                api_resp[0] = '\0';
                                int http_code = -1;
                                bool paired_ok = is_iot_device_paired_active();
                                if (paired_ok) {
                                    esp32_iot_scan_ticket(qr_p, is_checkin ? "checkin" : "verify",
                                                          api_resp, 4096, &http_code);
                                } else {
                                    snprintf(api_resp, 4096,
                                             "{\"success\":false,\"errorCode\":\"IOT_NOT_PAIRED\",\"statusCode\":400,"
                                             "\"message\":\"ESP32 chua duoc Admin duyet ghep noi IoT!\"}");
                                }
                                for (char *rp = api_resp; *rp; rp++) {
                                    if (*rp == '\r' || *rp == '\n') *rp = ' ';
                                }
                                /* Gửi kết quả JSON về PC NGAY LẬP TỨC trước khi bíp còi/mở servo để giao diện Web cập nhật tức thì! */
                                snprintf(uart_line, 4120, "\n%s:%s\n", is_checkin ? "CHECKINRESP" : "VERIFYRESP", api_resp);
                                safe_uart_write_string(uart_line);

                                if (paired_ok) {
                                    process_iot_scan_response(qr_p, clean_qr, api_resp, http_code, NULL);
                                } else {
                                    oled_show_ticket_status(OLED_TICKET_NET_ERR, clean_qr, "CHUA GHEP IOT");
                                    buzzer_beep(240);
                                }
                            }
                            free(uart_line);
                            free(api_resp);
                        } else if (sscanf(s_cmd_str, "SETAUTO:%d", &val) == 1) {
                            bool enabled = (val != 0);
                            mark_local_auto_confirm_changed(enabled);
                            char ack[48];
                            snprintf(ack, sizeof(ack), "AUTOCFG:%d\n", enabled ? 1 : 0);
                            safe_uart_write_string(ack);
                            ESP_LOGI(TAG, "UART: Saved auto_confirm=%d to NVS (pend_iot=%d)",
                                     enabled ? 1 : 0, g_iot_pending_auto_sync ? 1 : 0);
                        } else if (strncmp(s_cmd_str, "APPLYCFG:", 9) == 0) {
                            int cfg_auto = 0, cfg_ver = 0;
                            if (sscanf(s_cmd_str + 9, "%d|%d", &cfg_auto, &cfg_ver) == 2) {
                                if (g_iot_mutex) xSemaphoreTake(g_iot_mutex, portMAX_DELAY);
                                bool can_apply = !g_iot_pending_auto_sync && cfg_ver >= g_iot_config_version;
                                if (can_apply) {
                                    g_iot_config_version = cfg_ver;
                                    save_iot_nvs_i32("cfg_ver", cfg_ver);
                                }
                                if (g_iot_mutex) xSemaphoreGive(g_iot_mutex);
                                if (can_apply) {
                                    g_auto_confirm = (cfg_auto != 0);
                                    save_nvs_u8("auto_confirm", g_auto_confirm ? 1 : 0);
                                    sync_oled_network_state();
                                }
                            }
                        } else if (strncmp(s_cmd_str, "SETIOTURL:", 10) == 0) {
                            const char *u = s_cmd_str + 10;
                            while (*u == ' ') u++;
                            char norm_url[256] = "";
                            normalize_iot_url(u, norm_url, sizeof(norm_url));
                            if (norm_url[0] != '\0') {
                                if (g_iot_mutex) xSemaphoreTake(g_iot_mutex, portMAX_DELAY);
                                snprintf(g_iot_base_url, sizeof(g_iot_base_url), "%s", norm_url);
                                if (g_iot_mutex) xSemaphoreGive(g_iot_mutex);
                                save_iot_nvs_str("iot_url", g_iot_base_url);
                                g_iot_force_sync = true;
                                ESP_LOGI(TAG, "UART: Updated & saved persistent IoT URL -> %s", g_iot_base_url);
                                char ack[300];
                                snprintf(ack, sizeof(ack), "IOTURLSAVED:%s\n", g_iot_base_url);
                                safe_uart_write_string(ack);
                            }
                        } else if (strcmp(s_cmd_str, "GETIOTURL?") == 0 || strcmp(s_cmd_str, "GETIOTURL") == 0) {
                            char resp_line[300];
                            snprintf(resp_line, sizeof(resp_line), "IOTURLSTAT:%s\n", g_iot_base_url);
                            safe_uart_write_string(resp_line);
                        } else if (strcmp(s_cmd_str, "RESETIOT") == 0) {
                            clear_iot_credentials_nvs(true);
                            sync_oled_network_state();
                            ESP_LOGW(TAG, "UART: Cleared IoT pairing credentials");
                        } else if (strcmp(s_cmd_str, "HIST?") == 0 || strcmp(s_cmd_str, "HIST") == 0) {
                            if (s_dev_history.is_stale && g_wifi_connected && is_iot_device_paired_active()) {
                                fetchDeviceHistory(s_dev_history.page, s_dev_history.limit, s_dev_history.filter, s_dev_history.event_only);
                            }
                            if (g_history_mutex) xSemaphoreTake(g_history_mutex, portMAX_DELAY);
                            char line[256];
                            snprintf(line, sizeof(line),
                                     "HISTINFO:page=%d|limit=%d|total=%d|has_more=%d|all=%d|ok=%d|ver=%d|fail=%d|filter=%s|offline=%d|items=%d\n",
                                     s_dev_history.page, s_dev_history.limit, s_dev_history.total,
                                     s_dev_history.has_more ? 1 : 0, s_dev_history.total_scans,
                                     s_dev_history.ok_count, s_dev_history.verified_count, s_dev_history.failed_count,
                                     s_dev_history.filter, s_dev_history.offline_cached ? 1 : 0, s_dev_history.item_count);
                            safe_uart_write_string(line);
                            for (int hi = 0; hi < s_dev_history.item_count; hi++) {
                                const dev_history_item_t *it = &s_dev_history.items[hi];
                                snprintf(line, sizeof(line), "HISTITEM:%d|%s|%s|%s|%s|%s\n",
                                         hi + 1, it->result,
                                         it->ticket_code[0] ? it->ticket_code : "--",
                                         it->attendee_name[0] ? it->attendee_name : "--",
                                         it->scanned_at, it->message);
                                safe_uart_write_string(line);
                            }
                            if (g_history_mutex) xSemaphoreGive(g_history_mutex);
                        } else if (strncmp(s_cmd_str, "HIST:", 5) == 0) {
                            int hp = 1, hl = 5, heo = 0;
                            char hflt[24] = "ALL";
                            int matched = sscanf(s_cmd_str + 5, "%d,%d,%23[^,],%d", &hp, &hl, hflt, &heo);
                            if (matched >= 1) {
                                fetchDeviceHistory(hp, (matched >= 2) ? hl : 5, hflt, (heo != 0));
                                char ack[96];
                                snprintf(ack, sizeof(ack), "HISTOK:page=%d|total=%d|items=%d|has_more=%d\n",
                                         s_dev_history.page, s_dev_history.total, s_dev_history.item_count, s_dev_history.has_more ? 1 : 0);
                                safe_uart_write_string(ack);
                            }
                        } else if (sscanf(s_cmd_str, "HISTMODE:%d", &val) == 1) {
                            bool vis = (val != 0);
                            if (vis && s_dev_history.is_stale && g_wifi_connected && is_iot_device_paired_active()) {
                                fetchDeviceHistory(1, 5, s_dev_history.filter, s_dev_history.event_only);
                            }
                            oled_set_history_mode(vis, 0);
                            char ack[32];
                            snprintf(ack, sizeof(ack), "HISTMODEOK:%d\n", vis ? 1 : 0);
                            safe_uart_write_string(ack);
                        } else if (strcmp(s_cmd_str, "HISTNEXT") == 0) {
                            int cur = oled_get_history_selected_idx();
                            if (!oled_is_history_mode()) {
                                oled_set_history_mode(true, 0);
                            } else if (cur + 1 < s_dev_history.item_count) {
                                oled_set_history_mode(true, cur + 1);
                            } else if (s_dev_history.has_more) {
                                fetchDeviceHistory(s_dev_history.page + 1, s_dev_history.limit, s_dev_history.filter, s_dev_history.event_only);
                                oled_set_history_mode(true, 0);
                            }
                        } else if (strcmp(s_cmd_str, "HISTPREV") == 0) {
                            int cur = oled_get_history_selected_idx();
                            if (!oled_is_history_mode()) {
                                oled_set_history_mode(true, 0);
                            } else if (cur > 0) {
                                oled_set_history_mode(true, cur - 1);
                            } else if (s_dev_history.page > 1) {
                                fetchDeviceHistory(s_dev_history.page - 1, s_dev_history.limit, s_dev_history.filter, s_dev_history.event_only);
                                oled_set_history_mode(true, 0);
                            }
                        } else if (strncmp(s_cmd_str, "HISTFILTER:", 11) == 0) {
                            const char *flt = s_cmd_str + 11;
                            if (flt[0] != '\0') {
                                fetchDeviceHistory(1, 5, flt, s_dev_history.event_only);
                                oled_set_history_mode(true, 0);
                            }
                        } else if (strcmp(s_cmd_str, "SERVO:OPEN") == 0 || strcmp(s_cmd_str, "OPENSERVO") == 0) {
                            servo_trigger_gate_open();
                            safe_uart_write_string("SERVO:OPENED\n");
                            ESP_LOGI(TAG, "UART: Manual trigger servo gate open");
                        } else if (strcmp(s_cmd_str, "SERVO:CLOSE") == 0 || strcmp(s_cmd_str, "CLOSESERVO") == 0) {
                            servo_set_angle(SERVO_GATE_CLOSE_DEGREE);
                            safe_uart_write_string("SERVO:CLOSED\n");
                            ESP_LOGI(TAG, "UART: Manual trigger servo gate close");
                        } else if (sscanf(s_cmd_str, "SERVO:ANGLE:%d", &val) == 1) {
                            servo_set_angle((uint32_t)val);
                            char s_ack[48];
                            snprintf(s_ack, sizeof(s_ack), "SERVO:ANGLE=%d\n", val);
                            safe_uart_write_string(s_ack);
                            ESP_LOGI(TAG, "UART: Set servo angle to %d deg", val);
                        } else if (oled_p) {
                            char *st_p = oled_p + 5;
                            char *code_p = strchr(st_p, '|');
                            if (code_p) {
                                *code_p = '\0';
                                code_p++;
                            } else {
                                code_p = "";
                            }
                            char clean_c[32] = "";
                            if (code_p[0] != '\0') {
                                extract_ticket_code_c(code_p, clean_c, sizeof(clean_c));
                            }
                            if (strcmp(st_p, "OK") == 0) {
                                oled_show_ticket_status(OLED_TICKET_OK, clean_c, "VE HOP LE");
                                servo_trigger_gate_open();
                                led_trigger_green_ok();
                            } else if (strcmp(st_p, "VERIFIED") == 0) {
                                oled_show_ticket_status(OLED_TICKET_VERIFIED, clean_c, g_auto_confirm ? "DANG XAC NHAN" : "BAM XAC NHAN");
                                led_trigger_green_ok();
                            } else if (strcmp(st_p, "USED") == 0) {
                                oled_show_ticket_status(OLED_TICKET_USED, clean_c, "DA CHECKIN ROI");
                                led_trigger_red_error();
                            } else if (strcmp(st_p, "NOTFOUND") == 0) {
                                oled_show_ticket_status(OLED_TICKET_NOT_FOUND, clean_c, "VE KO TON TAI");
                                led_trigger_red_error();
                            } else if (strcmp(st_p, "CHECKING") == 0) {
                                oled_show_ticket_status(OLED_TICKET_CHECKING, clean_c, NULL);
                            } else if (strcmp(st_p, "ERROR") == 0) {
                                oled_show_ticket_status(OLED_TICKET_NET_ERR, clean_c, "KIEM TRA LAI");
                                led_trigger_red_error();
                            } else if (strcmp(st_p, "IDLE") == 0) {
                                oled_show_ticket_status(OLED_TICKET_IDLE, "", NULL);
                            }
                        } else if (strcmp(s_cmd_str, "LED:GREEN") == 0 || strcmp(s_cmd_str, "LEDGREEN") == 0) {
                            led_trigger_green_ok();
                            safe_uart_write_string("LED:GREEN_ON_60S\n");
                        } else if (strcmp(s_cmd_str, "LED:RED") == 0 || strcmp(s_cmd_str, "LEDRED") == 0) {
                            led_trigger_red_error();
                            safe_uart_write_string("LED:RED_ON_60S\n");
                        } else if (strcmp(s_cmd_str, "LED:OFF") == 0 || strcmp(s_cmd_str, "LEDOFF") == 0) {
                            led_turn_off_all();
                            safe_uart_write_string("LED:ALL_OFF\n");
                        } else if (sscanf(s_cmd_str, "TEST:GREEN:%d", &val) == 1 || sscanf(s_cmd_str, "TEST:LED_GREEN:%d", &val) == 1) {
                            led_set_green_direct(val);
                            char t_ack[48];
                            snprintf(t_ack, sizeof(t_ack), "TESTRESP:GREEN=%d\n", val ? 1 : 0);
                            safe_uart_write_string(t_ack);
                        } else if (sscanf(s_cmd_str, "TEST:RED:%d", &val) == 1 || sscanf(s_cmd_str, "TEST:LED_RED:%d", &val) == 1) {
                            led_set_red_direct(val);
                            char t_ack[48];
                            snprintf(t_ack, sizeof(t_ack), "TESTRESP:RED=%d\n", val ? 1 : 0);
                            safe_uart_write_string(t_ack);
                        } else if (sscanf(s_cmd_str, "TEST:BUZZER:%d", &val) == 1 || strcmp(s_cmd_str, "TEST:BUZZER") == 0) {
                            int b_dur = (val > 0) ? val : 150;
                            buzzer_beep(b_dur);
                            char t_ack[48];
                            snprintf(t_ack, sizeof(t_ack), "TESTRESP:BUZZER=%dms\n", b_dur);
                            safe_uart_write_string(t_ack);
                        } else if (sscanf(s_cmd_str, "TEST:SERVO:%d", &val) == 1) {
                            servo_set_angle((uint32_t)val);
                            char t_ack[48];
                            snprintf(t_ack, sizeof(t_ack), "TESTRESP:SERVO=%d\n", val);
                            safe_uart_write_string(t_ack);
                        } else if (strcmp(s_cmd_str, "TEST:IR") == 0 || strcmp(s_cmd_str, "TEST:SENSOR") == 0) {
                            bool obs = is_ir_obstacle_detected();
                            char t_ack[48];
                            snprintf(t_ack, sizeof(t_ack), "TESTRESP:IR=%d\n", obs ? 1 : 0);
                            safe_uart_write_string(t_ack);
                        } else if (strcmp(s_cmd_str, "TEST:ALL") == 0 || strcmp(s_cmd_str, "TEST:SYSTEM") == 0) {
                            trigger_hardware_selftest();
                            safe_uart_write_string("TESTRESP:ALL_STARTED\n");
                        } else if (sscanf(s_cmd_str, "PCHB:%d", &val) == 1) {
                            /* Heartbeat from PC host updates g_pc_usb_last_tick above */
                        } else if (sscanf(s_cmd_str, "AE:%d", &val) == 1 && s != NULL) {
                            s->set_ae_level(s, val);
                        } else if (sscanf(s_cmd_str, "CONTRAST:%d", &val) == 1 && s != NULL) {
                            s->set_contrast(s, val);
                        } else if (sscanf(s_cmd_str, "BRIGHT:%d", &val) == 1 && s != NULL) {
                            s->set_brightness(s, val);
                        } else if (sscanf(s_cmd_str, "SHARP:%d", &val) == 1 && s != NULL) {
                            s->set_sharpness(s, val);
                        } else if (sscanf(s_cmd_str, "QUAL:%d", &val) == 1 && s != NULL) {
                            s->set_quality(s, val);
                        } else if (sscanf(s_cmd_str, "FLIP:%d", &val) == 1 && s != NULL) {
                            s->set_vflip(s, val);
                        } else if (sscanf(s_cmd_str, "MIRROR:%d", &val) == 1 && s != NULL) {
                            s->set_hmirror(s, val);
                        } else if (strcmp(s_cmd_str, "BEEP:DOUBLE") == 0 || strcmp(s_cmd_str, "BEEP:2") == 0) {
                            buzzer_beep(70);
                            vTaskDelay(pdMS_TO_TICKS(80));
                            buzzer_beep(90);
                        } else if (strcmp(s_cmd_str, "BEEP:LONG") == 0 || strcmp(s_cmd_str, "BEEP:ERROR") == 0) {
                            buzzer_beep(350);
                        } else if (sscanf(s_cmd_str, "BEEP:%d", &val) == 1) {
                            buzzer_beep(val);
                        } else if (sscanf(s_cmd_str, "LED:%d", &val) == 1) {
                            set_led_duty(val);
                        }

                        cmd_idx = 0;
                    }
                } else if (cmd_idx < (int)sizeof(s_cmd_str) - 1) {
                    s_cmd_str[cmd_idx++] = c;
                }
            }
        }
    }
}

/* ============================================================================
 * 9. HÀM MAIN KHỞI ĐỘNG HỆ THỐNG & NÚT NHẤN LỊCH SỬ (GPIO0 BOOT)
 * ============================================================================ */
#define BOOT_BUTTON_PIN GPIO_NUM_0

static void history_btn_task(void *pvParameters)
{
    (void)pvParameters;
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BOOT_BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    /* Wait until boot stabilization */
    vTaskDelay(pdMS_TO_TICKS(1500));

    int last_lvl = 1;
    TickType_t press_tick = 0;

    while (1) {
        int lvl = gpio_get_level(BOOT_BUTTON_PIN);
        if (lvl == 0 && last_lvl == 1) {
            press_tick = xTaskGetTickCount();
        } else if (lvl == 1 && last_lvl == 0) {
            TickType_t dur = xTaskGetTickCount() - press_tick;
            if (dur >= pdMS_TO_TICKS(45)) {
                if (dur >= pdMS_TO_TICKS(3000)) {
                    /* Nhấn giữ siêu dài > 3s: Kích hoạt Hardware Self-Test ngay từ nút BOOT */
                    ESP_LOGI(TAG, "BOOT button held > 3s: Triggering Hardware Self-Test...");
                    buzzer_beep(200);
                    trigger_hardware_selftest();
                } else if (dur < pdMS_TO_TICKS(750)) {
                    /* Short press: open History or advance item/page */
                    if (!oled_is_history_mode()) {
                        if (s_dev_history.is_stale && g_wifi_connected && is_iot_device_paired_active()) {
                            fetchDeviceHistory(1, 5, s_dev_history.filter, s_dev_history.event_only);
                        }
                        oled_set_history_mode(true, 0);
                        ESP_LOGI(TAG, "BOOT button: Opened History view");
                    } else {
                        int cur = oled_get_history_selected_idx();
                        int max_i = 0;
                        bool has_more = false;
                        int cur_page = 1;
                        char cur_flt[24] = "ALL";
                        bool cur_eo = false;

                        if (g_history_mutex) xSemaphoreTake(g_history_mutex, portMAX_DELAY);
                        max_i = s_dev_history.item_count;
                        has_more = s_dev_history.has_more;
                        cur_page = s_dev_history.page;
                        snprintf(cur_flt, sizeof(cur_flt), "%s", s_dev_history.filter);
                        cur_eo = s_dev_history.event_only;
                        if (g_history_mutex) xSemaphoreGive(g_history_mutex);

                        if (cur + 1 < max_i) {
                            oled_set_history_mode(true, cur + 1);
                        } else if (has_more) {
                            fetchDeviceHistory(cur_page + 1, 5, cur_flt, cur_eo);
                            oled_set_history_mode(true, 0);
                        } else if (cur_page > 1) {
                            fetchDeviceHistory(1, 5, cur_flt, cur_eo);
                            oled_set_history_mode(true, 0);
                        } else {
                            oled_set_history_mode(false, 0);
                        }
                    }
                } else {
                    /* Long press: cycle filter ALL -> OK -> FAILED -> exit */
                    if (oled_is_history_mode()) {
                        const char *next_flt = "ALL";
                        bool exit_mode = false;
                        if (strcmp(s_dev_history.filter, "ALL") == 0) next_flt = "OK";
                        else if (strcmp(s_dev_history.filter, "OK") == 0) next_flt = "FAILED";
                        else exit_mode = true;

                        if (exit_mode) {
                            snprintf(s_dev_history.filter, sizeof(s_dev_history.filter), "ALL");
                            oled_set_history_mode(false, 0);
                        } else {
                            fetchDeviceHistory(1, 5, next_flt, s_dev_history.event_only);
                            oled_set_history_mode(true, 0);
                        }
                        buzzer_beep(40);
                    } else {
                        fetchDeviceHistory(1, 5, "ALL", false);
                        oled_set_history_mode(true, 0);
                        buzzer_beep(50);
                    }
                }
            }
        }
        last_lvl = lvl;
        vTaskDelay(pdMS_TO_TICKS(40));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "==========================================================");
    ESP_LOGI(TAG, " EventHub Dual-Mode QR Station (USB + Wi-Fi Standalone)   ");
    ESP_LOGI(TAG, "==========================================================");

    /* 0. Khởi tạo UART, HTTPS & Standalone Scan Mutex */
    g_uart_mutex = xSemaphoreCreateMutex();
    if (!g_uart_mutex) {
        ESP_LOGE(TAG, "Failed to create UART mutex!");
    }
    g_https_mutex = xSemaphoreCreateMutex();
    if (!g_https_mutex) {
        ESP_LOGE(TAG, "Failed to create HTTPS mutex!");
    }
    g_scan_state_mutex = xSemaphoreCreateMutex();
    if (!g_scan_state_mutex) {
        ESP_LOGE(TAG, "Failed to create scan state mutex!");
    }
    g_iot_mutex = xSemaphoreCreateMutex();
    if (!g_iot_mutex) {
        ESP_LOGE(TAG, "Failed to create IoT mutex!");
    }
    g_history_mutex = xSemaphoreCreateMutex();
    if (!g_history_mutex) {
        ESP_LOGE(TAG, "Failed to create History mutex!");
    }

    /* 1. Khởi tạo NVS Flash & đọc MAC + IoT Device Config */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    init_device_mac();
    load_iot_config_nvs();

    /* 2. Khởi tạo Còi Buzzer, Đèn LED PWM, Servo Cổng, Đèn LED Trạng Thái & Màn hình OLED 0.96" I2C (SSD1306) */
    init_buzzer();
    init_led_pwm();
    init_status_leds();
    init_servo_pwm();
    init_ir_sensor();
    oled_init();
    buzzer_beep(80);

    /* 3. Khởi tạo Camera OV5640 & Tối ưu ISP */
    if (init_camera() != ESP_OK) {
        ESP_LOGE(TAG, "Camera initialization failed! Halting.");
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    /* 4. Khởi tạo Wi-Fi Station (Tự kết nối theo cấu hình trong NVS) */
    init_wifi_sta();
    sync_oled_network_state();

    /* 5. Cấu hình UART0 sang chế độ High-Speed (2,000,000 baud) */
    ESP_LOGI(TAG, "Switching UART0 to %d baud for USB High-Speed Streaming...", UART_BAUD_RATE);
    fflush(stdout);
    vTaskDelay(pdMS_TO_TICKS(100));

    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, UART_RX_BUF_SIZE, UART_TX_BUF_SIZE, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));

    /* Khóa tất cả ESP_LOGI/W/E qua g_uart_mutex để log hệ thống không bao giờ xen ngang làm đứt JSON hoặc frame camera */
    esp_log_set_vprintf(safe_uart_log_vprintf);

    /* 6. Khởi chạy Task Streaming Video trên Core 1 (12KB stack cho SW-JPEG recovery khi cần) */
    xTaskCreatePinnedToCore(camera_stream_task, "cam_stream", 12288, NULL, 5, NULL, 1);

    /* 7. Khởi chạy Task Nhận Lệnh & Telemetry trên Core 0 */
    xTaskCreatePinnedToCore(uart_cmd_task, "uart_cmd", 14336, NULL, 4, NULL, 0);

    /* 8. Khởi chạy Task Quét QR Độc Lập khi rút khỏi PC (Core 0, mức ưu tiên 1 nhường CPU cho camera) */
    xTaskCreatePinnedToCore(standalone_qr_task, "qr_standalone", 14336, NULL, 1, NULL, 0);

    /* 9. Khởi chạy Task Đăng ký IoT, Ghép nối & Heartbeat trên Core 0 */
    xTaskCreatePinnedToCore(iot_device_task, "iot_device", 12288, NULL, 3, NULL, 0);

    /* 10. Khởi chạy Task Nút Nhấn BOOT (GPIO0) xem Lịch Sử trên OLED */
    xTaskCreatePinnedToCore(history_btn_task, "hist_btn", 4096, NULL, 1, NULL, 0);

    /* 11. Khởi chạy Task Điều Khiển Servo Nâng Cần Mở Cổng trên Core 0 */
    xTaskCreatePinnedToCore(servo_gate_task, "servo_gate", 4096, NULL, 2, &s_servo_task_handle, 0);

    ESP_LOGI(TAG, "EventHub Dual-Mode Station ready! MAC=%s", g_device_mac);
}
