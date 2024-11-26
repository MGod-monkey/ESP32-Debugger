/*
 * Copyright (c) 2023-2023, lihongquan
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-9-8      lihongquan   add license declaration
 */

#include <stdint.h>
#include "esp_log.h"
#include <errno.h>
#include <dirent.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <nvs_flash.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tinyusb.h"
#include "tusb_cdc_acm.h"
#include "sdkconfig.h"
#include "cdc_uart.h"
#include "tusb_config.h"
#include "DAP_config.h"
#include "DAP.h"
#include "usb_desc.h"
#include "msc_disk.h"
#include "esp_netif.h"
#include "web_handler.h"
#include "usb_cdc_handler.h"
#include "esp_http_server.h"
#include "programmer.h"
#include "protocol_examples_common.h"
#include "wifi_configuration.h"

#include "wifi_handle.h"
#include "mdns.h"

static const char *TAG = "main";
TaskHandle_t kDAPTaskHandle = NULL;
extern httpd_handle_t http_server;

void mdns_setup() {
    // initialize mDNS
    int ret;
    ret = mdns_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "mDNS initialize failed:%d", ret);
        return;
    }

    // set mDNS hostname
    ret = mdns_hostname_set(MDNS_HOSTNAME);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "mDNS set hostname failed:%d", ret);
        return;
    }
    ESP_LOGI(TAG, "mDNS hostname set to: [%s]", MDNS_HOSTNAME);

    // set default mDNS instance name
    ret = mdns_instance_name_set(MDNS_INSTANCE);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "mDNS set instance name failed:%d", ret);
        return;
    }
    ESP_LOGI(TAG, "mDNS instance name set to: [%s]", MDNS_INSTANCE);
}

extern "C" void tcp_server_task(void *pvParameters);
extern "C" void DAP_Thread(void *pvParameters);

extern "C" uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen)
{
    return 0;
}

#ifdef CONFIG_TINYUSB_VENDOR_ENABLED
extern "C" bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request)
{
    uint16_t total_len = 0;
    uint8_t *desc_ms_os_20 = NULL;
    tusb_desc_webusb_url_t *url = NULL;

    /* Nothing to do with DATA & ACK stage */
    if (stage != CONTROL_STAGE_SETUP)
    {
        return true;
    }

    switch (request->bmRequestType_bit.type)
    {
    case TUSB_REQ_TYPE_VENDOR:
        switch (request->bRequest)
        {
        case VENDOR_REQUEST_WEBUSB:
            /* Match vendor request in BOS descriptor and get landing page url */
            url = get_webusb_url();
            return tud_control_xfer(rhport, request, reinterpret_cast<void *>(url), url->bLength);
        case VENDOR_REQUEST_MICROSOFT:
            if (request->wIndex != 7)
            {
                return false;
            }

            /* Get Microsoft OS 2.0 compatible descriptor */

            desc_ms_os_20 = get_ms_descriptor();
            total_len = *reinterpret_cast<uint16_t *>(desc_ms_os_20 + 8);
            return tud_control_xfer(rhport, request, reinterpret_cast<void *>(desc_ms_os_20), total_len);
        default:
            break;
        }
        break;
    case TUSB_REQ_TYPE_CLASS:
        if (request->bRequest == 0x22)
        {
            /* Webserial simulates the CDC_REQUEST_SET_CONTROL_LINE_STATE (0x22) to connect and disconnect */
            return tud_control_status(rhport, request);
        }
        break;
    default:
        break;
    }

    /* Stall unknown request */
    return false;
}

extern "C" void tud_vendor_rx_cb(uint8_t itf)
{
    static uint8_t in[DAP_PACKET_SIZE] = {0};
    static uint8_t out[DAP_PACKET_SIZE] = {0};

    if (tud_vendor_n_read(itf, in, sizeof(in)) > 0)
    {
        tud_vendor_n_write(itf, out, DAP_ProcessCommand(in, out) & 0xFFFF);
        tud_vendor_n_flush(itf);
    }
}
#endif

extern "C" void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer, uint16_t bufsize)
{
    static uint8_t s_tx_buf[CFG_TUD_HID_EP_BUFSIZE];

    DAP_ProcessCommand(buffer, s_tx_buf);
    tud_hid_report(0, s_tx_buf, sizeof(s_tx_buf));
}

extern "C" void app_main(void)
{
    bool ret = false;

    ESP_ERROR_CHECK(nvs_flash_init());
    // ESP_ERROR_CHECK(example_connect());

    #if (USE_UART_BRIDGE == 1)
        uart_bridge_init();
    #endif
        wifi_init();
        DAP_Setup();

    #if (USE_MDNS == 1)
        mdns_setup();
    #endif

    tinyusb_config_t tusb_cfg = {
        .device_descriptor = NULL,
        .string_descriptor = NULL,
        .string_descriptor_count = 0,
        .external_phy = false,
        .configuration_descriptor = NULL,
        .self_powered = false,
        .vbus_monitor_io = 0};

    tinyusb_config_cdcacm_t acm_cfg = {
        .usb_dev = TINYUSB_USBDEV_0,
        .cdc_port = TINYUSB_CDC_ACM_0,
        .rx_unread_buf_sz = 64,
        .callback_rx = usb_cdc_send_to_uart, // the first way to register a callback
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = NULL,
        .callback_line_coding_changed = usb_cdc_set_line_codinig};

    ESP_LOGI(TAG, "USB initialization");

    // ret = msc_dick_mount(CONFIG_TINYUSB_MSC_MOUNT_PATH);
    ret = msc_dick_mount(CONFIG_TINYUSB_MSC_MOUNT_PATH);
    if (!ret) {
        ESP_LOGE(TAG, "Failed to mount MSC disk");
        return;
    }
    tusb_cfg.configuration_descriptor = get_configuration_descriptor(false);
    tusb_cfg.string_descriptor = get_string_descriptor(false);
    tusb_cfg.string_descriptor_count = get_string_descriptor_count();
    tusb_cfg.device_descriptor = get_device_descriptor();

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    // ESP_ERROR_CHECK(tusb_cdc_acm_init(&acm_cfg));

    programmer_init();
    cdc_uart_init(UART_NUM_1, GPIO_NUM_13, GPIO_NUM_14, 115200);
    cdc_uart_register_rx_handler(CDC_UART_USB_HANDLER, usb_cdc_send_to_host, (void *)TINYUSB_CDC_ACM_0);
    // cdc_uart_register_rx_handler(CDC_UART_WEB_HANDLER, web_send_to_clients, &http_server);
    ESP_LOGI(TAG, "USB initialization DONE");

    // Specify the usbip server task
#if (USE_TCP_NETCONN == 1)
    xTaskCreate(tcp_netconn_task, "tcp_server", 4096, NULL, 14, NULL);
#else // BSD style
    xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 14, NULL);
#endif

    // DAP handle task
    xTaskCreate(DAP_Thread, "DAP_Task", 2048, NULL, 10, &kDAPTaskHandle);
}
