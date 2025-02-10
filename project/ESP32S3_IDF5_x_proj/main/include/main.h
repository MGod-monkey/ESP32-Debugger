#ifndef __main__h
#define __main__h

// 标准库头文件
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <errno.h>
#include <dirent.h>
#include <arpa/inet.h>

// ESP-IDF 头文件
#include "sdkconfig.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_timer.h"
#include "esp_sleep.h"
#include "esp_task_wdt.h"
#include "esp_spiffs.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "nvs_flash.h"
#include "spi_flash_mmap.h"

// FreeRTOS 头文件
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

// 驱动程序头文件
#include "driver/gpio.h"
#include "driver/i2s.h"
#include "driver/ledc.h"
#include "driver/uart.h"

// 网络相关头文件
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

// TinyUSB 头文件
#include "tinyusb.h"
#include "tusb_cdc_acm.h"
#include "tusb_config.h"
#include "usb_desc.h"
#include "msc_disk.h"
#include "tusb_tasks.h"
#include "tusb_msc_storage.h"

#include "os.h"
#include "cJSON.h"
#include "DAP.h"
#include "DAP_config.h"
#include "wifi_handle.h"
#include "wifi_configuration.h"
#include "led_strip.h"
#include "cdc_uart.h"
#include "web_handler.h"
#include "usb_cdc_handler.h"
#include "programmer.h"
#include "protocol_examples_common.h"
#include "mdns.h"
#include "uart_bridge.h"
#include "hal_gpio.h"
#include "led.h"

typedef enum
{
    LOG_NONE  = 0,   /*!< No log output */
    LOG_ERROR = 1,  /*!< Critical errors, software module can not recover on its own */
    LOG_WARN  = 2,   /*!< Error conditions from which recovery measures have been taken */
    LOG_INFO  = 3,   /*!< Information messages which describe normal flow of events */
    LOG_DEBUG = 4,  /*!< Extra information which is not necessary for normal use (values, pointers, sizes, etc). */
    LOG_VERBOSE = 5 /*!< Bigger chunks of debugging information, or frequent messages which can potentially flood the output. */
} log_level_t;

#define COLOR_RESET   "\033[0m"
#define COLOR_WHITE   "\033[37m"
#define COLOR_RED     "\033[31m"
#define COLOR_GREEN   "\033[32m"
#define COLOR_YELLOW  "\033[33m"
#define COLOR_BLUE    "\033[34m"
#define COLOR_CYAN    "\033[36m"

static const char* get_color(log_level_t level) {
    switch (level) {
        case LOG_ERROR:
            return COLOR_RED;
        case LOG_WARN:
            return COLOR_YELLOW;
        case LOG_DEBUG:
            return COLOR_WHITE;
        case LOG_INFO:
            return COLOR_GREEN;
        case LOG_VERBOSE:
            return COLOR_BLUE;
        default:
            return COLOR_RESET;
    }
}

// 自定义日志打印函数，添加文件名、行号和颜色
static void custom_log(log_level_t level, const char* tag, const char* file, int line, const char* format, ...) {
    const char* color = get_color(level);

    // 获取可变参数
    va_list args;
    va_start(args, format);

    // 打印日志信息
    printf("%s[%s] %s:%d: ", color, tag, file, line); // 打印调用 log_printf 的文件名和行号
    vprintf(format, args);
    printf(COLOR_RESET "\n");

    va_end(args);
}

// 定义控制日志级别的宏
#define SHOW_DEBUG 2  // 0: 不显示日志, 1: 仅显示 ERROR 和 INFO, 2: 显示所有日志

#if SHOW_DEBUG == 0
    #define log_printf(tag, level, format, ...)
#elif SHOW_DEBUG == 1
    #define log_printf(tag, level, format, ...) do { \
        if ((level) == LOG_ERROR || (level) == LOG_INFO) { \
            custom_log(level, tag, __FILE__, __LINE__, format, ##__VA_ARGS__); \
        } \
    } while (0)
#elif SHOW_DEBUG == 2
    #define log_printf(tag, level, format, ...) do { \
        custom_log(level, tag, __FILE__, __LINE__, format, ##__VA_ARGS__); \
    } while (0)
#else
    #define log_printf(tag, level, format, ...)
#endif

#define LV_LVGL_H_INCLUDE_SIMPLE

#endif
