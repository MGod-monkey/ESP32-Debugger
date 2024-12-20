/*
 * Copyright (c) 2023-2023, lihongquan
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-9-8      lihongquan   add license declaration
 */
#include "usb_cdc_handler.h"
#include "cdc_uart.h"
#include "esp_log.h"

#define TAG "usb_cdc_handler"

void usb_cdc_send_to_host(void *context, uint8_t *data, size_t size)
{
    ESP_LOGD(TAG, "data %p, size %d", data, size);

    // 添加调试日志
    ESP_LOGI(TAG, "CDC connected: %d", tud_cdc_n_connected((int)context));
    
    if (tud_cdc_n_connected((int)context))
    {
        // 检查写入结果
        size_t written = tinyusb_cdcacm_write_queue((tinyusb_cdcacm_itf_t)context, data, size);
        ESP_LOGI(TAG, "Written bytes: %d", written);
        
        // 确保数据被刷新
        esp_err_t flush_result = tinyusb_cdcacm_write_flush((tinyusb_cdcacm_itf_t)context, 100); // 增加超时时间
        ESP_LOGI(TAG, "Flush result: %d", flush_result);
    }
}

void usb_cdc_set_line_codinig(int itf, cdcacm_event_t *event)
{
    if (!event || !event->line_coding_changed_data.p_line_coding) {
        // ESP_LOGE(TAG, "Invalid line coding event");
        return;
    }

    cdc_line_coding_t const *coding = event->line_coding_changed_data.p_line_coding;
    uint32_t baudrate = 0;

    if (cdc_uart_get_baudrate(&baudrate) && (baudrate != coding->bit_rate))
    {
        // ESP_LOGI(TAG, "Changing baudrate from %lu to %lu", baudrate, coding->bit_rate);
        cdc_uart_set_baudrate(coding->bit_rate);
    }
}

void usb_cdc_send_to_uart(int itf, cdcacm_event_t *event)
{
    static uint8_t buf[CONFIG_TINYUSB_CDC_RX_BUFSIZE + 1];
    size_t rx_size = 0;
    esp_err_t ret = tinyusb_cdcacm_read(itf, buf, CONFIG_TINYUSB_CDC_RX_BUFSIZE, &rx_size);

    if (ret == ESP_OK)
    {
        cdc_uart_write(buf, rx_size);
    }
}