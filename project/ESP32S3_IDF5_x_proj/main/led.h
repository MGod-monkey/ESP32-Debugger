#ifndef WIFI_RSSI_LED_H
#define WIFI_RSSI_LED_H

#include "hal_gpio.h"

typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} rgb_color_t;

typedef enum {
    LED_COLOR_ORANGE,
    LED_COLOR_PURPLE,
    LED_COLOR_RED,
    LED_COLOR_GREEN,
    LED_COLOR_BLUE,
    LED_COLOR_WHITE,
    LED_COLOR_BLACK
} led_color_type_t;

// 颜色定义
static const rgb_color_t led_colors[] = {
    [LED_COLOR_ORANGE] = {.r = 255, .g = 128, .b = 0},
    [LED_COLOR_PURPLE] = {.r = 128, .g = 0, .b = 255},
    [LED_COLOR_RED]    = {.r = 255, .g = 0, .b = 0},
    [LED_COLOR_GREEN]  = {.r = 0, .g = 255, .b = 0},
    [LED_COLOR_BLUE]   = {.r = 0, .g = 0, .b = 255},
    [LED_COLOR_WHITE]  = {.r = 255, .g = 255, .b = 255},
    [LED_COLOR_BLACK]  = {.r = 0, .g = 0, .b = 0}
};

#ifdef __cplusplus
extern "C" {
#endif

void ws2812_init(void);
void ws2812_task(void *pvParameters);
void ws2812_set_color(led_color_type_t color);
void ws2812_blink(uint32_t delay_ms, uint8_t blink_count, led_color_type_t color);
void ws2812_breath(led_color_type_t color, uint32_t breath_period_ms);

#ifdef __cplusplus
}
#endif

#endif // WIFI_RSSI_LED_H