#include "stdio.h"
#include "led_strip.h"
#include "wifi_rssi_led.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_err.h"

// LED strip common configuration
led_strip_config_t strip_config = {
    .strip_gpio_num = GPIO_LED_WIFI_STATUS,  // The GPIO that connected to the LED strip's data line
    .max_leds = 1,                 // The number of LEDs in the strip,
    .led_model = LED_MODEL_WS2812, // LED strip model, it determines the bit timing
    .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB, // The color component format is G-R-B
    .flags = {
        .invert_out = false, // don't invert the output signal
    }
};

/// RMT backend specific configuration
led_strip_rmt_config_t rmt_config = {
    .clk_src = RMT_CLK_SRC_DEFAULT,    // different clock source can lead to different power consumption
    .resolution_hz = 10 * 1000 * 1000, // RMT counter clock frequency: 10MHz
    .mem_block_symbols = 64,           // the memory size of each RMT channel, in words (4 bytes)
    .flags = {
        .with_dma = false, // DMA feature is available on chips like ESP32-S3/P4
    }
};

/// Create the LED strip object
led_strip_handle_t led_strip;

void wifi_rssi_led_init(void){
    // 初始化信号灯
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
}

/**
 * @brief LED闪烁控制函数
 * @param delay_ms: 闪烁延时(ms)
 * @param blink_count: 闪烁次数
 * @param r: 红色分量(0-255)
 * @param g: 绿色分量(0-255)
 * @param b: 蓝色分量(0-255)
 */
void wifi_rssi_led_blink(uint32_t delay_ms, uint8_t blink_count, uint8_t r, uint8_t g, uint8_t b)
{
    for (uint8_t i = 0; i < blink_count; i++) {
        // 点亮LED
        led_strip_set_pixel(led_strip, 0, r, g, b);
        led_strip_refresh(led_strip);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));

        // 熄灭LED
        led_strip_set_pixel(led_strip, 0, 0, 0, 0);
        led_strip_refresh(led_strip);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}

void signalLED_change_task(void *pvParameters) {
    while (1) {
        wifi_ap_record_t ap_info;
        esp_err_t status = esp_wifi_sta_get_ap_info(&ap_info);
        if (status == ESP_OK) {
            int8_t rssi = ap_info.rssi;
            int delay_time = 1000; // 默认1秒
            printf("rssi: %d", rssi);

            if (rssi > -50) {
                // 信号强度非常好，LED常亮绿色
                led_strip_set_pixel(led_strip, 0, 0, 255, 0);
                led_strip_refresh(led_strip);
                vTaskDelay(pdMS_TO_TICKS(delay_time));
                continue;
            } else if (rssi > -60) {
                // 信号强度好，LED慢闪绿色
                delay_time = 1000;
            } else if (rssi > -70) {
                // 信号强度一般，LED中速闪烁绿色
                delay_time = 500;
            } else if (rssi > -80) {
                // 信号强度差，LED快速闪烁绿色
                delay_time = 250;
            } else {
                // 信号非常差，LED非常快速闪烁绿色
                delay_time = 100;
            }

            led_strip_set_pixel(led_strip, 0, 0, 255, 0);
            led_strip_refresh(led_strip);
            vTaskDelay(pdMS_TO_TICKS(delay_time));
            led_strip_set_pixel(led_strip, 0, 0, 0, 0);
            led_strip_refresh(led_strip);
            vTaskDelay(pdMS_TO_TICKS(delay_time));
        } else {
            // WiFi未连接，LED显示红色
            led_strip_set_pixel(led_strip, 0, 255, 0, 0);
            led_strip_refresh(led_strip);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}