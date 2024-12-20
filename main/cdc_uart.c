/*
 * Copyright (c) 2023-2023, lihongquan
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-9-8      lihongquan   add license declaration
 */

#include "cdc_uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "tinyusb.h"
#include "tusb_cdc_acm.h"

typedef struct
{
    uart_port_t uart;
    cdc_uart_cb_t cb[CDC_UART_HANDLER_NUM];
} cdc_uart_t;

// 添加消息队列定义
static QueueHandle_t uart_queue;
typedef struct {
    uint8_t buf[CONFIG_TINYUSB_CDC_RX_BUFSIZE + 1];
    size_t buf_len;
    uint8_t itf;
} uart_message_t;

static cdc_uart_t s_cdc_uart = {0};
static const char *TAG = "cdc_uart";
static void cdc_uart_rx_task(void *param);

bool cdc_uart_init(uart_port_t uart, gpio_num_t tx_pin, gpio_num_t rx_pin, int baudrate)
{
    bool ret = false;
    uart_config_t uart_config = {
        .baud_rate = baudrate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    s_cdc_uart.uart = uart;
    ret = (ESP_OK == uart_driver_install(s_cdc_uart.uart, 2 * 1024, 2 * 1024, 0, NULL, 0));
    ret = ret && (ESP_OK == uart_param_config(s_cdc_uart.uart, &uart_config));
    ret = ret && (ESP_OK == uart_set_pin(s_cdc_uart.uart, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    // 创建消息队列
    uart_queue = xQueueCreate(5, sizeof(uart_message_t));
    assert(uart_queue);

    if (ret) {
        xTaskCreate(cdc_uart_rx_task, "cdc_uart_rx_task", 4096, (void *)&s_cdc_uart, 10, NULL);
    }

    return ret;
}

bool cdc_uart_set_baudrate(uint32_t baudrate)
{
    return (ESP_OK == uart_set_baudrate(s_cdc_uart.uart, baudrate));
}

bool cdc_uart_get_baudrate(uint32_t *baudrate)
{
    return (ESP_OK == uart_get_baudrate(s_cdc_uart.uart, baudrate));
}

bool cdc_uart_write(const void *src, size_t size)
{
    return (0 <= uart_write_bytes(s_cdc_uart.uart, src, size));
}

void cdc_uart_register_rx_handler(cdc_uart_handler_def handler, cdc_uart_rx_callback_t func, void *context)
{
    if ((handler >= CDC_UART_HANDLER_NUM) || !func)
    {
        return;
    }

    s_cdc_uart.cb[handler].func = func;
    s_cdc_uart.cb[handler].usr_data = context;
}

static void cdc_uart_rx_task(void *param)
{
    #define RX_BUF_SIZE 256  // 增大缓冲区
    uint8_t data[RX_BUF_SIZE] = {0};
    cdc_uart_t *cdc_uart = (cdc_uart_t *)param;
    int read = 0;

    while (1) {
        // 直接读取并发送数据
        read = uart_read_bytes(cdc_uart->uart, data, RX_BUF_SIZE, pdMS_TO_TICKS(50));

        if (read > 0) {
            // 方法1：直接发送到CDC
            tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, data, read);
            tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, 0);

            // // 方法2：通过回调发送
            // for (int i = 0; i < CDC_UART_HANDLER_NUM; i++) {
            //     if (s_cdc_uart.cb[i].func) {
            //         s_cdc_uart.cb[i].func(s_cdc_uart.cb[i].usr_data, data, read);
            //     }
            // }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}