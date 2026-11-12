#include "oled_ssd1306.h"
#include "camera_pins.h"

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "OLED_SSD1306";

#define OLED_WIDTH   128
#define OLED_HEIGHT  64
#define OLED_PAGES   (OLED_HEIGHT / 8)
#define OLED_FB_SIZE (OLED_WIDTH * OLED_PAGES)

#ifndef OLED_I2C_SDA_PIN
#define OLED_I2C_SDA_PIN 21
#endif
#ifndef OLED_I2C_SCL_PIN
#define OLED_I2C_SCL_PIN 47
#endif

#define OLED_ADDR_PRIMARY   0x3C
#define OLED_ADDR_SECONDARY 0x3D
#define OLED_I2C_FREQ_HZ    100000
#define OLED_RESULT_HOLD_MS (3U * 60U * 1000U) /* Keep scanned ticket result for 3 minutes */
#define OLED_CHECK_HOLD_MS  12000

/* Classic 5x7 ASCII font table (0x20..0x7E) */
static const uint8_t FONT5X7[95][5] = {
    {0x00,0x00,0x00,0x00,0x00}, /* ' ' */
    {0x00,0x00,0x5F,0x00,0x00}, /* '!' */
    {0x00,0x07,0x00,0x07,0x00}, /* '"' */
    {0x14,0x7F,0x14,0x7F,0x14}, /* '#' */
    {0x24,0x2A,0x7F,0x2A,0x12}, /* '$' */
    {0x23,0x13,0x08,0x64,0x62}, /* '%' */
    {0x36,0x49,0x55,0x22,0x50}, /* '&' */
    {0x00,0x05,0x03,0x00,0x00}, /* ''' */
    {0x00,0x1C,0x22,0x41,0x00}, /* '(' */
    {0x00,0x41,0x22,0x1C,0x00}, /* ')' */
    {0x14,0x08,0x3E,0x08,0x14}, /* '*' */
    {0x08,0x08,0x3E,0x08,0x08}, /* '+' */
    {0x00,0x50,0x30,0x00,0x00}, /* ',' */
    {0x08,0x08,0x08,0x08,0x08}, /* '-' */
    {0x00,0x60,0x60,0x00,0x00}, /* '.' */
    {0x20,0x10,0x08,0x04,0x02}, /* '/' */
    {0x3E,0x51,0x49,0x45,0x3E}, /* '0' */
    {0x00,0x42,0x7F,0x40,0x00}, /* '1' */
    {0x42,0x61,0x51,0x49,0x46}, /* '2' */
    {0x21,0x41,0x45,0x4B,0x31}, /* '3' */
    {0x18,0x14,0x12,0x7F,0x10}, /* '4' */
    {0x27,0x45,0x45,0x45,0x39}, /* '5' */
    {0x3C,0x4A,0x49,0x49,0x30}, /* '6' */
    {0x01,0x71,0x09,0x05,0x03}, /* '7' */
    {0x36,0x49,0x49,0x49,0x36}, /* '8' */
    {0x06,0x49,0x49,0x29,0x1E}, /* '9' */
    {0x00,0x36,0x36,0x00,0x00}, /* ':' */
    {0x00,0x56,0x36,0x00,0x00}, /* ';' */
    {0x08,0x14,0x22,0x41,0x00}, /* '<' */
    {0x14,0x14,0x14,0x14,0x14}, /* '=' */
    {0x00,0x41,0x22,0x14,0x08}, /* '>' */
    {0x02,0x01,0x51,0x09,0x06}, /* '?' */
    {0x32,0x49,0x79,0x41,0x3E}, /* '@' */
    {0x7E,0x11,0x11,0x11,0x7E}, /* 'A' */
    {0x7F,0x49,0x49,0x49,0x36}, /* 'B' */
    {0x3E,0x41,0x41,0x41,0x22}, /* 'C' */
    {0x7F,0x41,0x41,0x22,0x1C}, /* 'D' */
    {0x7F,0x49,0x49,0x49,0x41}, /* 'E' */
    {0x7F,0x09,0x09,0x09,0x01}, /* 'F' */
    {0x3E,0x41,0x49,0x49,0x7A}, /* 'G' */
    {0x7F,0x08,0x08,0x08,0x7F}, /* 'H' */
    {0x00,0x41,0x7F,0x41,0x00}, /* 'I' */
    {0x20,0x40,0x41,0x3F,0x01}, /* 'J' */
    {0x7F,0x08,0x14,0x22,0x41}, /* 'K' */
    {0x7F,0x40,0x40,0x40,0x40}, /* 'L' */
    {0x7F,0x02,0x0C,0x02,0x7F}, /* 'M' */
    {0x7F,0x04,0x08,0x10,0x7F}, /* 'N' */
    {0x3E,0x41,0x41,0x41,0x3E}, /* 'O' */
    {0x7F,0x09,0x09,0x09,0x06}, /* 'P' */
    {0x3E,0x41,0x51,0x21,0x5E}, /* 'Q' */
    {0x7F,0x09,0x19,0x29,0x46}, /* 'R' */
    {0x46,0x49,0x49,0x49,0x31}, /* 'S' */
    {0x01,0x01,0x7F,0x01,0x01}, /* 'T' */
    {0x3F,0x40,0x40,0x40,0x3F}, /* 'U' */
    {0x1F,0x20,0x40,0x20,0x1F}, /* 'V' */
    {0x3F,0x40,0x38,0x40,0x3F}, /* 'W' */
    {0x63,0x14,0x08,0x14,0x63}, /* 'X' */
    {0x07,0x08,0x70,0x08,0x07}, /* 'Y' */
    {0x61,0x51,0x49,0x45,0x43}, /* 'Z' */
    {0x00,0x7F,0x41,0x41,0x00}, /* '[' */
    {0x02,0x04,0x08,0x10,0x20}, /* '\' */
    {0x00,0x41,0x41,0x7F,0x00}, /* ']' */
    {0x04,0x02,0x01,0x02,0x04}, /* '^' */
    {0x40,0x40,0x40,0x40,0x40}, /* '_' */
    {0x00,0x01,0x02,0x04,0x00}, /* '`' */
    {0x20,0x54,0x54,0x54,0x78}, /* 'a' */
    {0x7F,0x48,0x44,0x44,0x38}, /* 'b' */
    {0x38,0x44,0x44,0x44,0x20}, /* 'c' */
    {0x38,0x44,0x44,0x48,0x7F}, /* 'd' */
    {0x38,0x54,0x54,0x54,0x18}, /* 'e' */
    {0x08,0x7E,0x09,0x01,0x02}, /* 'f' */
    {0x0C,0x52,0x52,0x52,0x3E}, /* 'g' */
    {0x7F,0x08,0x04,0x04,0x78}, /* 'h' */
    {0x00,0x44,0x7D,0x40,0x00}, /* 'i' */
    {0x20,0x40,0x44,0x3D,0x00}, /* 'j' */
    {0x7F,0x10,0x28,0x44,0x00}, /* 'k' */
    {0x00,0x41,0x7F,0x40,0x00}, /* 'l' */
    {0x7C,0x04,0x18,0x04,0x78}, /* 'm' */
    {0x7C,0x08,0x04,0x04,0x78}, /* 'n' */
    {0x38,0x44,0x44,0x44,0x38}, /* 'o' */
    {0x7C,0x14,0x14,0x14,0x08}, /* 'p' */
    {0x08,0x14,0x14,0x18,0x7C}, /* 'q' */
    {0x7C,0x08,0x04,0x04,0x08}, /* 'r' */
    {0x48,0x54,0x54,0x54,0x20}, /* 's' */
    {0x04,0x3F,0x44,0x40,0x20}, /* 't' */
    {0x3C,0x40,0x40,0x20,0x7C}, /* 'u' */
    {0x1C,0x20,0x40,0x20,0x1C}, /* 'v' */
    {0x3C,0x40,0x30,0x40,0x3C}, /* 'w' */
    {0x44,0x28,0x10,0x28,0x44}, /* 'x' */
    {0x0C,0x50,0x50,0x50,0x3C}, /* 'y' */
    {0x44,0x64,0x54,0x4C,0x44}, /* 'z' */
    {0x00,0x08,0x36,0x41,0x00}, /* '{' */
    {0x00,0x00,0x7F,0x00,0x00}, /* '|' */
    {0x00,0x41,0x36,0x08,0x00}, /* '}' */
    {0x08,0x04,0x08,0x10,0x08}, /* '~' */
};

typedef struct {
    bool wifi_connected;
    char ssid[33];
    char ip[16];
    int8_t rssi;
    bool auto_confirm;
    bool pc_usb_active;
    bool has_token;

    oled_iot_status_t iot_status;
    char pairing_code[12];
    char event_code[24];
    char mac_short[16];

    oled_ticket_status_t ticket_status;
    char ticket_code[24];
    char sub_msg[24];
    TickType_t ticket_tick;
    bool dirty;
} oled_state_t;

static i2c_master_bus_handle_t s_i2c_bus = NULL;
static i2c_master_dev_handle_t s_oled_dev = NULL;
static SemaphoreHandle_t s_oled_mutex = NULL;
static TaskHandle_t s_oled_task = NULL;
static bool s_oled_ready = false;
static uint8_t s_fb[OLED_FB_SIZE];
static oled_state_t s_state = {
    .wifi_connected = false,
    .ssid = "",
    .ip = "0.0.0.0",
    .rssi = 0,
    .auto_confirm = false,
    .pc_usb_active = false,
    .has_token = false,
    .iot_status = OLED_IOT_NONE,
    .pairing_code = "",
    .event_code = "",
    .mac_short = "",
    .ticket_status = OLED_TICKET_IDLE,
    .ticket_code = "",
    .sub_msg = "",
    .ticket_tick = 0,
    .dirty = true,
};

static esp_err_t ssd1306_send_cmds(const uint8_t *cmds, size_t len)
{
    if (!s_oled_dev || !cmds || len == 0 || len > 31) return ESP_ERR_INVALID_STATE;
    uint8_t buf[32];
    buf[0] = 0x00; /* Co=0, D/C#=0 (command stream) */
    memcpy(buf + 1, cmds, len);
    return i2c_master_transmit(s_oled_dev, buf, len + 1, 100);
}

static void ssd1306_flush(void)
{
    if (!s_oled_ready || !s_oled_dev) return;
    static const uint8_t addr_cmds[] = {
        0x21, 0x00, 0x7F, /* Column 0..127 */
        0x22, 0x00, 0x07  /* Page 0..7 */
    };
    if (ssd1306_send_cmds(addr_cmds, sizeof(addr_cmds)) != ESP_OK) return;

    static const uint8_t data_prefix = 0x40;
    i2c_master_transmit_multi_buffer_info_t bufs[2] = {
        { .write_buffer = &data_prefix, .buffer_size = 1 },
        { .write_buffer = s_fb, .buffer_size = OLED_FB_SIZE },
    };
    i2c_master_multi_buffer_transmit(s_oled_dev, bufs, 2, 150);
}

static void fb_clear(void)
{
    memset(s_fb, 0, sizeof(s_fb));
}

static inline void fb_pixel(int x, int y, bool on)
{
    if (x < 0 || x >= OLED_WIDTH || y < 0 || y >= OLED_HEIGHT) return;
    size_t idx = (size_t)(y >> 3) * OLED_WIDTH + (size_t)x;
    uint8_t mask = (uint8_t)(1U << (y & 7));
    if (on) {
        s_fb[idx] |= mask;
    } else {
        s_fb[idx] &= (uint8_t)(~mask);
    }
}

static void fb_hline(int x, int y, int w)
{
    for (int i = 0; i < w; i++) {
        fb_pixel(x + i, y, true);
    }
}

static void fb_fill_rect(int x, int y, int w, int h)
{
    for (int dy = 0; dy < h; dy++) {
        for (int dx = 0; dx < w; dx++) {
            fb_pixel(x + dx, y + dy, true);
        }
    }
}

static void fb_rect(int x, int y, int w, int h)
{
    if (w <= 0 || h <= 0) return;
    for (int i = 0; i < w; i++) {
        fb_pixel(x + i, y, true);
        fb_pixel(x + i, y + h - 1, true);
    }
    for (int j = 0; j < h; j++) {
        fb_pixel(x, y + j, true);
        fb_pixel(x + w - 1, y + j, true);
    }
}

static void fb_circle(int cx, int cy, int r)
{
    int x = r;
    int y = 0;
    int err = 1 - x;
    while (x >= y) {
        fb_pixel(cx + x, cy + y, true);
        fb_pixel(cx + y, cy + x, true);
        fb_pixel(cx - y, cy + x, true);
        fb_pixel(cx - x, cy + y, true);
        fb_pixel(cx - x, cy - y, true);
        fb_pixel(cx - y, cy - x, true);
        fb_pixel(cx + y, cy - x, true);
        fb_pixel(cx + x, cy - y, true);
        y++;
        if (err < 0) {
            err += 2 * y + 1;
        } else {
            x--;
            err += 2 * (y - x) + 1;
        }
    }
}

static void fb_thick_line(int x0, int y0, int x1, int y1, int half_thick)
{
    int dx = (x1 > x0) ? (x1 - x0) : (x0 - x1);
    int sx = (x0 < x1) ? 1 : -1;
    int dy = -((y1 > y0) ? (y1 - y0) : (y0 - y1));
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy;

    while (1) {
        for (int ox = -half_thick; ox <= half_thick; ox++) {
            for (int oy = -half_thick; oy <= half_thick; oy++) {
                if (ox * ox + oy * oy <= half_thick * half_thick + 1) {
                    fb_pixel(x0 + ox, y0 + oy, true);
                }
            }
        }
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y0 += sy;
        }
    }
}

static void fb_draw_char(int x, int y, char c, int scale)
{
    uint8_t uc = (uint8_t)c;
    if (uc < 0x20 || uc > 0x7E) uc = '?';
    const uint8_t *glyph = FONT5X7[uc - 0x20];
    for (int col = 0; col < 5; col++) {
        uint8_t bits = glyph[col];
        for (int row = 0; row < 7; row++) {
            if (bits & (1U << row)) {
                if (scale <= 1) {
                    fb_pixel(x + col, y + row, true);
                } else {
                    fb_fill_rect(x + col * scale, y + row * scale, scale, scale);
                }
            }
        }
    }
}

static void fb_draw_text(int x, int y, const char *str, int max_chars, int scale)
{
    if (!str) return;
    int step = 6 * (scale > 1 ? scale : 1);
    int drawn = 0;
    const uint8_t *p = (const uint8_t *)str;

    while (*p && drawn < max_chars && x + 5 * (scale > 1 ? scale : 1) <= OLED_WIDTH) {
        if (*p < 0x20) {
            p++;
            continue;
        }
        if (*p >= 0x80) {
            /* Skip multi-byte UTF-8 glyphs cleanly on ASCII 5x7 font */
            while (*p >= 0x80) p++;
            fb_draw_char(x, y, '?', scale);
        } else {
            fb_draw_char(x, y, (char)*p, scale);
            p++;
        }
        x += step;
        drawn++;
    }
}

static void draw_icon_wifi_bars(int x, int y, bool connected, int8_t rssi)
{
    if (!connected) {
        /* Small 'x' + base dot indicating Wi-Fi disconnected */
        fb_pixel(x + 1, y + 1, true);
        fb_pixel(x + 5, y + 1, true);
        fb_pixel(x + 2, y + 2, true);
        fb_pixel(x + 4, y + 2, true);
        fb_pixel(x + 3, y + 3, true);
        fb_pixel(x + 2, y + 4, true);
        fb_pixel(x + 4, y + 4, true);
        fb_pixel(x + 1, y + 5, true);
        fb_pixel(x + 5, y + 5, true);
        fb_fill_rect(x + 8, y + 5, 2, 2);
        return;
    }

    int bars = 1;
    if (rssi >= -55) bars = 4;
    else if (rssi >= -67) bars = 3;
    else if (rssi >= -78) bars = 2;

    for (int b = 0; b < 4; b++) {
        int bh = 2 + b * 2;
        int bx = x + b * 3;
        int by = y + (8 - bh);
        if (b < bars) {
            fb_fill_rect(bx, by, 2, bh);
        } else {
            fb_fill_rect(bx, y + 6, 2, 1);
        }
    }
}

static void draw_icon_tick(int cx, int cy)
{
    fb_circle(cx, cy, 15);
    fb_thick_line(cx - 7, cy, cx - 2, cy + 6, 1);
    fb_thick_line(cx - 2, cy + 6, cx + 8, cy - 6, 1);
}

static void draw_icon_cross(int cx, int cy)
{
    fb_circle(cx, cy, 15);
    fb_thick_line(cx - 7, cy - 7, cx + 7, cy + 7, 1);
    fb_thick_line(cx + 7, cy - 7, cx - 7, cy + 7, 1);
}

static void draw_icon_qr(int x, int y)
{
    /* Neat 24x24 QR finder emblem */
    fb_rect(x, y, 9, 9);
    fb_fill_rect(x + 3, y + 3, 3, 3);
    fb_rect(x + 15, y, 9, 9);
    fb_fill_rect(x + 18, y + 3, 3, 3);
    fb_rect(x, y + 15, 9, 9);
    fb_fill_rect(x + 3, y + 18, 3, 3);
    /* Data modules inside */
    fb_fill_rect(x + 11, y + 2, 2, 2);
    fb_fill_rect(x + 11, y + 7, 3, 3);
    fb_fill_rect(x + 3, y + 11, 3, 2);
    fb_fill_rect(x + 10, y + 12, 4, 4);
    fb_fill_rect(x + 17, y + 11, 3, 3);
    fb_fill_rect(x + 12, y + 18, 3, 3);
    fb_fill_rect(x + 18, y + 18, 5, 5);
}

static void render_oled_frame(const oled_state_t *st, uint32_t anim_step)
{
    fb_clear();

    /* ---- Top Bar (y = 0..15): Network Status ---- */
    draw_icon_wifi_bars(1, 0, st->wifi_connected, st->rssi);

    if (st->wifi_connected) {
        const char *ssid_txt = st->ssid[0] ? st->ssid : "WiFi OK";
        fb_draw_text(15, 0, ssid_txt, 14, 1);
    } else if (st->ssid[0]) {
        char net_line[20];
        static const char *const dots[4] = {"", ".", "..", "..."};
        snprintf(net_line, sizeof(net_line), "TIM:%.7s%s", st->ssid, dots[anim_step & 3]);
        fb_draw_text(15, 0, net_line, 14, 1);
    } else {
        fb_draw_text(15, 0, "CHUA CO WIFI", 14, 1);
    }

    fb_draw_text(104, 0, st->auto_confirm ? "AUTO" : "MAN ", 4, 1);

    if (st->wifi_connected) {
        char ip_line[24];
        snprintf(ip_line, sizeof(ip_line), "%s", st->ip);
        fb_draw_text(1, 8, ip_line, 15, 1);
        char right_tag[8];
        if (st->pc_usb_active) {
            snprintf(right_tag, sizeof(right_tag), "USB");
        } else {
            snprintf(right_tag, sizeof(right_tag), "%ddB", (int)st->rssi);
        }
        int rx = OLED_WIDTH - (int)strlen(right_tag) * 6;
        if (rx < 92) rx = 92;
        fb_draw_text(rx, 8, right_tag, 6, 1);
    } else {
        fb_draw_text(1, 8, st->pc_usb_active ? "CHE DO: PC USB" : "DANG KET NOI LAI", 16, 1);
        fb_draw_text(110, 8, st->pc_usb_active ? "USB" : "IND", 3, 1);
    }

    fb_hline(0, 15, OLED_WIDTH);

    /* ---- Body (y = 17..63): Ticket Check-in Status ---- */
    const char *code = st->ticket_code[0] ? st->ticket_code : "--";
    switch (st->ticket_status) {
        case OLED_TICKET_OK:
            draw_icon_tick(17, 39);
            fb_draw_text(39, 20, "CHECKIN OK!", 14, 1);
            fb_draw_text(39, 34, code, 14, 1);
            fb_draw_text(39, 48, st->sub_msg[0] ? st->sub_msg : "VE HOP LE", 14, 1);
            break;

        case OLED_TICKET_VERIFIED:
            draw_icon_tick(17, 39);
            fb_draw_text(39, 20, "VE HOP LE!", 14, 1);
            fb_draw_text(39, 34, code, 14, 1);
            fb_draw_text(39, 48, st->sub_msg[0] ? st->sub_msg : "BAM XAC NHAN", 14, 1);
            break;

        case OLED_TICKET_USED:
            draw_icon_cross(17, 39);
            fb_draw_text(39, 20, "VE DA DUNG!", 14, 1);
            fb_draw_text(39, 34, code, 14, 1);
            fb_draw_text(39, 48, st->sub_msg[0] ? st->sub_msg : "DA CHECKIN ROI", 14, 1);
            break;

        case OLED_TICKET_NOT_FOUND:
            draw_icon_cross(17, 39);
            fb_draw_text(39, 20, "KHONG TIM THAY", 14, 1);
            fb_draw_text(39, 34, code, 14, 1);
            fb_draw_text(39, 48, st->sub_msg[0] ? st->sub_msg : "VE KO TON TAI", 14, 1);
            break;

        case OLED_TICKET_WRONG_EVENT:
            draw_icon_cross(17, 39);
            fb_draw_text(39, 20, "SAI SU KIEN!", 14, 1);
            fb_draw_text(39, 34, code, 14, 1);
            fb_draw_text(39, 48, st->sub_msg[0] ? st->sub_msg : "VE KHAC SU KIEN", 14, 1);
            break;

        case OLED_TICKET_NET_ERR:
            draw_icon_cross(17, 39);
            fb_draw_text(39, 20, "LOI KET NOI!", 14, 1);
            fb_draw_text(39, 34, code, 14, 1);
            fb_draw_text(39, 48, st->sub_msg[0] ? st->sub_msg : "KIEM TRA MANG", 14, 1);
            break;

        case OLED_TICKET_CHECKING:
            draw_icon_qr(6, 27);
            fb_draw_text(39, 20, "DANG KIEM TRA", 14, 1);
            fb_draw_text(39, 34, code, 14, 1);
            fb_draw_text(39, 48, "VUI LONG CHO..", 14, 1);
            break;

        case OLED_TICKET_IDLE:
        default:
            draw_icon_qr(6, 27);
            if (st->iot_status == OLED_IOT_PENDING && st->pairing_code[0]) {
                char line2[24];
                char line3[24];
                snprintf(line2, sizeof(line2), "MA: %s", st->pairing_code);
                snprintf(line3, sizeof(line3), "MAC:%s", st->mac_short[0] ? st->mac_short : "--");
                fb_draw_text(39, 20, "CHO DUYET!", 14, 1);
                fb_draw_text(39, 34, line2, 14, 1);
                fb_draw_text(39, 48, line3, 14, 1);
            } else if (st->iot_status == OLED_IOT_REPAIR_REQUIRED) {
                fb_draw_text(39, 20, "CAN RESET GHEP", 14, 1);
                fb_draw_text(39, 34, "MO ADMIN WEB", 14, 1);
                fb_draw_text(39, 48, "DE CAP MA MOI", 14, 1);
            } else if (st->iot_status == OLED_IOT_DISABLED) {
                fb_draw_text(39, 20, "MAY BI KHOA!", 14, 1);
                fb_draw_text(39, 34, "TRANG THAI OFF", 14, 1);
                fb_draw_text(39, 48, "LIEN HE ADMIN", 14, 1);
            } else if (st->iot_status == OLED_IOT_NO_EVENT) {
                fb_draw_text(39, 20, "CHUA CO SU KIEN", 14, 1);
                fb_draw_text(39, 34, "GAN EVENT TREN", 14, 1);
                fb_draw_text(39, 48, "TRANG ADMIN", 14, 1);
            } else if (!st->pc_usb_active && !st->has_token && st->iot_status != OLED_IOT_READY) {
                fb_draw_text(39, 20, "DANG DANG KY", 14, 1);
                fb_draw_text(39, 34, "KET NOI SERVER", 14, 1);
                fb_draw_text(39, 48, st->wifi_connected ? "CHO PHAN HOI.." : "CAN KET NOI WIFI", 14, 1);
            } else {
                fb_draw_text(39, 20, "SAN SANG QUET", 14, 1);
                if (st->iot_status == OLED_IOT_READY && st->event_code[0]) {
                    char evt_line[24];
                    snprintf(evt_line, sizeof(evt_line), "SK:%.11s", st->event_code);
                    fb_draw_text(39, 34, evt_line, 14, 1);
                } else {
                    fb_draw_text(39, 34, st->auto_confirm ? "AUTO: BAT" : "AUTO: TAT", 14, 1);
                }
                fb_draw_text(39, 48, st->pc_usb_active ? "CHEDO: PC USB" :
                                     (st->iot_status == OLED_IOT_READY ? "CHEDO: IOT MAY" : "CHEDO: DOC LAP"), 14, 1);
            }
            break;
    }
}

static void cleanup_i2c_handles(void)
{
    if (s_oled_dev) {
        i2c_master_bus_rm_device(s_oled_dev);
        s_oled_dev = NULL;
    }
    if (s_i2c_bus) {
        i2c_del_master_bus(s_i2c_bus);
        s_i2c_bus = NULL;
    }
}

static bool try_direct_write_init(int sda_pin, int scl_pin, uint16_t addr, uint32_t freq_hz, bool quiet_log)
{
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = freq_hz,
    };
    esp_err_t err = i2c_master_bus_add_device(s_i2c_bus, &dev_cfg, &s_oled_dev);
    if (err != ESP_OK) {
        if (!quiet_log) {
            ESP_LOGW(TAG, "Add dev @0x%02X failed: %s", addr, esp_err_to_name(err));
        }
        return false;
    }

    static const uint8_t probe_off[] = { 0xAE };
    err = ssd1306_send_cmds(probe_off, sizeof(probe_off));
    if (err != ESP_OK) {
        if (!quiet_log) {
            ESP_LOGW(TAG, "Direct TX @0x%02X (SDA=%d, SCL=%d, %luHz) failed: %s",
                     addr, sda_pin, scl_pin, (unsigned long)freq_hz, esp_err_to_name(err));
        }
        i2c_master_bus_rm_device(s_oled_dev);
        s_oled_dev = NULL;
        return false;
    }

    static const uint8_t init_cmds[] = {
        0xAE,       /* Display OFF */
        0xD5, 0x80, /* Clock divide ratio / osc freq */
        0xA8, 0x3F, /* Multiplex 64 lines */
        0xD3, 0x00, /* Display offset 0 */
        0x40,       /* Start line 0 */
        0x8D, 0x14, /* Charge pump ON */
        0x20, 0x00, /* Horizontal addressing mode */
        0xA1,       /* Segment remap */
        0xC8,       /* COM output scan direction remapped */
        0xDA, 0x12, /* COM pins hardware config (128x64) */
        0x81, 0xCF, /* Contrast */
        0xD9, 0xF1, /* Pre-charge period */
        0xDB, 0x40, /* VCOMH deselect level */
        0xA4,       /* Resume to RAM content */
        0xA6,       /* Normal display (non-inverted) */
        0xAF        /* Display ON */
    };
    if (ssd1306_send_cmds(init_cmds, sizeof(init_cmds)) != ESP_OK) {
        ESP_LOGW(TAG, "SSD1306 init sequence failed @0x%02X (SDA=%d, SCL=%d)", addr, sda_pin, scl_pin);
        i2c_master_bus_rm_device(s_oled_dev);
        s_oled_dev = NULL;
        return false;
    }

    s_oled_ready = true;
    ESP_LOGI(TAG, "SSD1306 128x64 OLED initialized @0x%02X (SDA=GPIO%d, SCL=GPIO%d, %luHz)",
             addr, sda_pin, scl_pin, (unsigned long)freq_hz);

    oled_state_t init_snap = s_state;
    if (s_oled_mutex) xSemaphoreTake(s_oled_mutex, portMAX_DELAY);
    init_snap = s_state;
    s_state.dirty = false;
    if (s_oled_mutex) xSemaphoreGive(s_oled_mutex);

    render_oled_frame(&init_snap, 0);
    ssd1306_flush();
    return true;
}

static bool try_probe_and_init_oled_pins(int sda_pin, int scl_pin, bool scan_all, bool quiet_log)
{
    cleanup_i2c_handles();

    gpio_reset_pin((gpio_num_t)sda_pin);
    gpio_reset_pin((gpio_num_t)scl_pin);
    gpio_set_direction((gpio_num_t)sda_pin, GPIO_MODE_INPUT);
    gpio_set_direction((gpio_num_t)scl_pin, GPIO_MODE_INPUT);
    gpio_set_pull_mode((gpio_num_t)sda_pin, GPIO_PULLDOWN_ONLY);
    gpio_set_pull_mode((gpio_num_t)scl_pin, GPIO_PULLDOWN_ONLY);
    vTaskDelay(pdMS_TO_TICKS(5));
    int sda_pd = gpio_get_level((gpio_num_t)sda_pin);
    int scl_pd = gpio_get_level((gpio_num_t)scl_pin);

    gpio_set_pull_mode((gpio_num_t)sda_pin, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)scl_pin, GPIO_PULLUP_ONLY);
    vTaskDelay(pdMS_TO_TICKS(5));
    int sda_lvl = gpio_get_level((gpio_num_t)sda_pin);
    int scl_lvl = gpio_get_level((gpio_num_t)scl_pin);
    if (!quiet_log) {
        ESP_LOGI(TAG, "I2C pin check SDA(GPIO%d)=%d(PD=%d), SCL(GPIO%d)=%d(PD=%d)",
                 sda_pin, sda_lvl, sda_pd, scl_pin, scl_lvl, scl_pd);
    }

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = (gpio_num_t)sda_pin,
        .scl_io_num = (gpio_num_t)scl_pin,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t err = i2c_new_master_bus(&bus_cfg, &s_i2c_bus);
    if (err != ESP_OK) {
        if (!quiet_log) {
            ESP_LOGW(TAG, "Failed to init I2C master bus (SDA=%d, SCL=%d): %s",
                     sda_pin, scl_pin, esp_err_to_name(err));
        }
        return false;
    }

    uint16_t found_addr = 0;
    esp_err_t p3c = i2c_master_probe(s_i2c_bus, OLED_ADDR_PRIMARY, 120);
    esp_err_t p3d = (p3c == ESP_OK) ? ESP_FAIL : i2c_master_probe(s_i2c_bus, OLED_ADDR_SECONDARY, 120);
    if (p3c == ESP_OK) {
        found_addr = OLED_ADDR_PRIMARY;
    } else if (p3d == ESP_OK) {
        found_addr = OLED_ADDR_SECONDARY;
    } else {
        if (!quiet_log) {
            ESP_LOGW(TAG, "I2C probe (SDA=%d, SCL=%d): 0x3C->%s, 0x3D->%s",
                     sda_pin, scl_pin, esp_err_to_name(p3c), esp_err_to_name(p3d));
        }
        if (scan_all) {
            for (uint16_t addr = 0x08; addr <= 0x77; addr++) {
                if (addr == OLED_ADDR_PRIMARY || addr == OLED_ADDR_SECONDARY) continue;
                if (i2c_master_probe(s_i2c_bus, addr, 30) == ESP_OK) {
                    ESP_LOGI(TAG, "I2C device detected at 0x%02X (SDA=%d, SCL=%d)", addr, sda_pin, scl_pin);
                    found_addr = addr;
                    break;
                }
            }
        }
    }

    if (found_addr != 0) {
        if (try_direct_write_init(sda_pin, scl_pin, found_addr, OLED_I2C_FREQ_HZ, quiet_log)) {
            return true;
        }
    } else {
        /* Fallback: some SSD1306 clones fail 0-byte i2c_master_probe or 100kHz with weak pullups; try 50kHz direct write */
        if (try_direct_write_init(sda_pin, scl_pin, OLED_ADDR_PRIMARY, 50000, quiet_log)) {
            return true;
        }
        if (try_direct_write_init(sda_pin, scl_pin, OLED_ADDR_SECONDARY, 50000, quiet_log)) {
            return true;
        }
    }

    cleanup_i2c_handles();
    return false;
}

static void oled_display_task(void *pvParameters)
{
    (void)pvParameters;
    uint32_t anim_step = 0;
    TickType_t last_probe_tick = xTaskGetTickCount();

    while (1) {
        oled_state_t snap;
        bool should_flush = false;
        TickType_t now = xTaskGetTickCount();

        if (!s_oled_ready && (now - last_probe_tick) >= pdMS_TO_TICKS(2500)) {
            last_probe_tick = now;
            if (!try_probe_and_init_oled_pins(OLED_I2C_SDA_PIN, OLED_I2C_SCL_PIN, false, true)) {
                try_probe_and_init_oled_pins(OLED_I2C_SCL_PIN, OLED_I2C_SDA_PIN, false, true);
            }
        }

        if (s_oled_mutex) xSemaphoreTake(s_oled_mutex, portMAX_DELAY);
        if (s_state.ticket_status != OLED_TICKET_IDLE) {
            uint32_t hold_ms = (s_state.ticket_status == OLED_TICKET_CHECKING) ? OLED_CHECK_HOLD_MS : OLED_RESULT_HOLD_MS;
            if ((now - s_state.ticket_tick) >= pdMS_TO_TICKS(hold_ms)) {
                s_state.ticket_status = OLED_TICKET_IDLE;
                s_state.ticket_code[0] = '\0';
                s_state.sub_msg[0] = '\0';
                s_state.dirty = true;
            }
        }
        if (!s_state.wifi_connected) {
            anim_step++;
            s_state.dirty = true;
        }
        if (s_state.dirty) {
            snap = s_state;
            s_state.dirty = false;
            should_flush = true;
        }
        if (s_oled_mutex) xSemaphoreGive(s_oled_mutex);

        if (should_flush && s_oled_ready) {
            render_oled_frame(&snap, anim_step);
            ssd1306_flush();
        }

        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(450));
    }
}

void oled_init(void)
{
    if (!s_oled_mutex) {
        s_oled_mutex = xSemaphoreCreateMutex();
    }

    vTaskDelay(pdMS_TO_TICKS(60));

    bool ok = try_probe_and_init_oled_pins(OLED_I2C_SDA_PIN, OLED_I2C_SCL_PIN, true, false);
    if (!ok) {
        ok = try_probe_and_init_oled_pins(OLED_I2C_SCL_PIN, OLED_I2C_SDA_PIN, false, false);
    }
    if (!ok) {
        ESP_LOGW(TAG, "No SSD1306 OLED detected on GPIO %d/%d (0x3C/0x3D). Will auto-retry in background.",
                 OLED_I2C_SDA_PIN, OLED_I2C_SCL_PIN);
    }

    if (!s_oled_task) {
        xTaskCreatePinnedToCore(oled_display_task, "oled_ui", 4096, NULL, 2, &s_oled_task, 0);
    }
}

bool oled_is_ready(void)
{
    return s_oled_ready;
}

void oled_update_network(bool wifi_connected,
                         const char *ssid,
                         const char *ip,
                         int8_t rssi,
                         bool auto_confirm,
                         bool pc_usb_active,
                         bool has_token)
{
    if (!s_oled_mutex) return;
    const char *s = ssid ? ssid : "";
    const char *p = ip ? ip : "0.0.0.0";

    xSemaphoreTake(s_oled_mutex, portMAX_DELAY);
    bool changed = (s_state.wifi_connected != wifi_connected) ||
                   (s_state.auto_confirm != auto_confirm) ||
                   (s_state.pc_usb_active != pc_usb_active) ||
                   (s_state.has_token != has_token) ||
                   (s_state.rssi != rssi) ||
                   (strncmp(s_state.ssid, s, sizeof(s_state.ssid) - 1) != 0) ||
                   (strncmp(s_state.ip, p, sizeof(s_state.ip) - 1) != 0);
    if (changed) {
        s_state.wifi_connected = wifi_connected;
        snprintf(s_state.ssid, sizeof(s_state.ssid), "%s", s);
        snprintf(s_state.ip, sizeof(s_state.ip), "%s", p);
        s_state.rssi = rssi;
        s_state.auto_confirm = auto_confirm;
        s_state.pc_usb_active = pc_usb_active;
        s_state.has_token = has_token;
        s_state.dirty = true;
    }
    xSemaphoreGive(s_oled_mutex);

    if (changed && s_oled_task) {
        xTaskNotifyGive(s_oled_task);
    }
}

void oled_show_ticket_status(oled_ticket_status_t status, const char *ticket_code, const char *sub_msg)
{
    if (!s_oled_mutex) return;

    xSemaphoreTake(s_oled_mutex, portMAX_DELAY);
    s_state.ticket_status = status;
    snprintf(s_state.ticket_code, sizeof(s_state.ticket_code), "%s", ticket_code ? ticket_code : "");
    snprintf(s_state.sub_msg, sizeof(s_state.sub_msg), "%s", sub_msg ? sub_msg : "");
    s_state.ticket_tick = xTaskGetTickCount();
    s_state.dirty = true;
    xSemaphoreGive(s_oled_mutex);

    if (s_oled_task) {
        xTaskNotifyGive(s_oled_task);
    }
}

void oled_update_iot_status(oled_iot_status_t iot_status,
                            const char *pairing_code,
                            const char *event_code,
                            const char *mac_short)
{
    if (!s_oled_mutex) return;
    const char *pc = pairing_code ? pairing_code : "";
    const char *ec = event_code ? event_code : "";
    const char *mc = mac_short ? mac_short : "";

    xSemaphoreTake(s_oled_mutex, portMAX_DELAY);
    bool changed = (s_state.iot_status != iot_status) ||
                   (strncmp(s_state.pairing_code, pc, sizeof(s_state.pairing_code) - 1) != 0) ||
                   (strncmp(s_state.event_code, ec, sizeof(s_state.event_code) - 1) != 0) ||
                   (strncmp(s_state.mac_short, mc, sizeof(s_state.mac_short) - 1) != 0);
    if (changed) {
        s_state.iot_status = iot_status;
        snprintf(s_state.pairing_code, sizeof(s_state.pairing_code), "%s", pc);
        snprintf(s_state.event_code, sizeof(s_state.event_code), "%s", ec);
        snprintf(s_state.mac_short, sizeof(s_state.mac_short), "%s", mc);
        s_state.dirty = true;
    }
    xSemaphoreGive(s_oled_mutex);

    if (changed && s_oled_task) {
        xTaskNotifyGive(s_oled_task);
    }
}
