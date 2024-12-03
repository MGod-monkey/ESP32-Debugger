// /*
//  * Copyright (c) 2023-2023, lihongquan
//  *
//  * SPDX-License-Identifier: Apache-2.0
//  *
//  * Change Logs:
//  * Date           Author       Notes
//  * 2023-9-8      lihongquan   add license declaration
//  */

// #include <stdint.h>
// #include "esp_log.h"
// #include <errno.h>
// #include <dirent.h>
// #include <esp_wifi.h>
// #include <esp_event.h>
// #include <nvs_flash.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "tinyusb.h"
// #include "tusb_cdc_acm.h"
// #include "sdkconfig.h"
// #include "cdc_uart.h"
// #include "tusb_config.h"
// #include "DAP_config.h"
// #include "DAP.h"
// #include "usb_desc.h"
// #include "msc_disk.h"
// #include "esp_netif.h"
// #include "web_handler.h"
// #include "usb_cdc_handler.h"
// #include "esp_http_server.h"
// #include "programmer.h"
// #include "protocol_examples_common.h"
// #include "wifi_configuration.h"

// #include "wifi_handle.h"
// #include "mdns.h"

// static const char *TAG = "main";
// TaskHandle_t kDAPTaskHandle = NULL;
// extern httpd_handle_t http_server;

// void mdns_setup() {
//     // initialize mDNS
//     int ret;
//     ret = mdns_init();
//     if (ret != ESP_OK) {
//         ESP_LOGW(TAG, "mDNS initialize failed:%d", ret);
//         return;
//     }

//     // set mDNS hostname
//     ret = mdns_hostname_set(MDNS_HOSTNAME);
//     if (ret != ESP_OK) {
//         ESP_LOGW(TAG, "mDNS set hostname failed:%d", ret);
//         return;
//     }
//     ESP_LOGI(TAG, "mDNS hostname set to: [%s]", MDNS_HOSTNAME);

//     // set default mDNS instance name
//     ret = mdns_instance_name_set(MDNS_INSTANCE);
//     if (ret != ESP_OK) {
//         ESP_LOGW(TAG, "mDNS set instance name failed:%d", ret);
//         return;
//     }
//     ESP_LOGI(TAG, "mDNS instance name set to: [%s]", MDNS_INSTANCE);
// }

// extern "C" void tcp_server_task(void *pvParameters);
// extern "C" void DAP_Thread(void *pvParameters);

// extern "C" uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen)
// {
//     return 0;
// }

// #ifdef CONFIG_TINYUSB_VENDOR_ENABLED
// extern "C" bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request)
// {
//     uint16_t total_len = 0;
//     uint8_t *desc_ms_os_20 = NULL;
//     tusb_desc_webusb_url_t *url = NULL;

//     /* Nothing to do with DATA & ACK stage */
//     if (stage != CONTROL_STAGE_SETUP)
//     {
//         return true;
//     }

//     switch (request->bmRequestType_bit.type)
//     {
//     case TUSB_REQ_TYPE_VENDOR:
//         switch (request->bRequest)
//         {
//         case VENDOR_REQUEST_WEBUSB:
//             /* Match vendor request in BOS descriptor and get landing page url */
//             url = get_webusb_url();
//             return tud_control_xfer(rhport, request, reinterpret_cast<void *>(url), url->bLength);
//         case VENDOR_REQUEST_MICROSOFT:
//             if (request->wIndex != 7)
//             {
//                 return false;
//             }

//             /* Get Microsoft OS 2.0 compatible descriptor */

//             desc_ms_os_20 = get_ms_descriptor();
//             total_len = *reinterpret_cast<uint16_t *>(desc_ms_os_20 + 8);
//             return tud_control_xfer(rhport, request, reinterpret_cast<void *>(desc_ms_os_20), total_len);
//         default:
//             break;
//         }
//         break;
//     case TUSB_REQ_TYPE_CLASS:
//         if (request->bRequest == 0x22)
//         {
//             /* Webserial simulates the CDC_REQUEST_SET_CONTROL_LINE_STATE (0x22) to connect and disconnect */
//             return tud_control_status(rhport, request);
//         }
//         break;
//     default:
//         break;
//     }

//     /* Stall unknown request */
//     return false;
// }

// extern "C" void tud_vendor_rx_cb(uint8_t itf)
// {
//     static uint8_t in[DAP_PACKET_SIZE] = {0};
//     static uint8_t out[DAP_PACKET_SIZE] = {0};

//     if (tud_vendor_n_read(itf, in, sizeof(in)) > 0)
//     {
//         tud_vendor_n_write(itf, out, DAP_ProcessCommand(in, out) & 0xFFFF);
//         tud_vendor_n_flush(itf);
//     }
// }
// #endif

// extern "C" void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer, uint16_t bufsize)
// {
//     static uint8_t s_tx_buf[CFG_TUD_HID_EP_BUFSIZE];

//     DAP_ProcessCommand(buffer, s_tx_buf);
//     tud_hid_report(0, s_tx_buf, sizeof(s_tx_buf));
// }

// void wifi_init_softap(void)
// {
//     ESP_ERROR_CHECK(esp_netif_init());
//     ESP_ERROR_CHECK(esp_event_loop_create_default());
//     esp_netif_create_default_wifi_ap();

//     wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
//     ESP_ERROR_CHECK(esp_wifi_init(&cfg));

//     wifi_config_t wifi_config = {
//         .ap = {
//             .ssid = WIFI_SSID,
//             .ssid_len = strlen(WIFI_SSID),
//             .channel = 1,
//             .password = WIFI_PASS,
//             .max_connection = 4,
//             .authmode = WIFI_AUTH_WPA_WPA2_PSK
//         },
//     };
    
//     ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
//     ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
//     ESP_ERROR_CHECK(esp_wifi_start());
// }

// extern "C" void app_main(void)
// {
// //     bool ret = false;

// //     ESP_ERROR_CHECK(nvs_flash_init());
// //     // ESP_ERROR_CHECK(example_connect());

// //     #if (USE_UART_BRIDGE == 1)
// //         uart_bridge_init();
// //     #endif
// //         // wifi_init();
// //         DAP_Setup();

// //     #if (USE_MDNS == 1)
// //         mdns_setup();
// //     #endif

// //     tinyusb_config_t tusb_cfg = {
// //         .device_descriptor = NULL,
// //         .string_descriptor = NULL,
// //         .string_descriptor_count = 0,
// //         .external_phy = false,
// //         .configuration_descriptor = NULL,
// //         .self_powered = false,
// //         .vbus_monitor_io = 0};

// //     tinyusb_config_cdcacm_t acm_cfg = {
// //         .usb_dev = TINYUSB_USBDEV_0,
// //         .cdc_port = TINYUSB_CDC_ACM_0,
// //         .rx_unread_buf_sz = 64,
// //         .callback_rx = usb_cdc_send_to_uart, // the first way to register a callback
// //         .callback_rx_wanted_char = NULL,
// //         .callback_line_state_changed = NULL,
// //         .callback_line_coding_changed = usb_cdc_set_line_codinig};

// //     ESP_LOGI(TAG, "USB initialization");

// //     // ret = msc_dick_mount(CONFIG_TINYUSB_MSC_MOUNT_PATH);
// //     ret = msc_dick_mount(CONFIG_TINYUSB_MSC_MOUNT_PATH);
// //     if (!ret) {
// //         ESP_LOGE(TAG, "Failed to mount MSC disk");
// //         return;
// //     }
// //     tusb_cfg.configuration_descriptor = get_configuration_descriptor(ret);
// //     tusb_cfg.string_descriptor = get_string_descriptor(ret);
// //     tusb_cfg.string_descriptor_count = get_string_descriptor_count();
// //     tusb_cfg.device_descriptor = get_device_descriptor();

// //     ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
// //     ESP_ERROR_CHECK(tusb_cdc_acm_init(&acm_cfg));

// //     programmer_init();
// //     cdc_uart_init(UART_NUM_1, GPIO_NUM_13, GPIO_NUM_14, 115200);
// //     cdc_uart_register_rx_handler(CDC_UART_USB_HANDLER, usb_cdc_send_to_host, (void *)TINYUSB_CDC_ACM_0);
// //     // cdc_uart_register_rx_handler(CDC_UART_WEB_HANDLER, web_send_to_clients, &http_server);
// //     ESP_LOGI(TAG, "USB initialization DONE");

// //     // Specify the usbip server task
// // #if (USE_TCP_NETCONN == 1)
// //     xTaskCreate(tcp_netconn_task, "tcp_server", 4096, NULL, 14, NULL);
// // #else // BSD style
// //     xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 14, NULL);
// // #endif

// //     // DAP handle task
// //     xTaskCreate(DAP_Thread, "DAP_Task", 2048, NULL, 10, &kDAPTaskHandle);
//     // 初始化NVS
//     ESP_ERROR_CHECK(nvs_flash_init());

//     // 初始化WiFi
//     wifi_init_softap();
    
//     // 初始化DAP
//     DAP_Setup();

//     // 创建TCP服务器任务
//     xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 5, &tcp_server_handle);

//     // USB设备初始化
//     tinyusb_config_t tusb_cfg = {
//         .device_descriptor = NULL,
//         .string_descriptor = NULL,
//         .string_descriptor_count = 0,
//         .external_phy = false,
//         .configuration_descriptor = NULL,
//         .self_powered = false,
//         .vbus_monitor_io = 0
//     };

//     // 配置USB描述符
//     tusb_cfg.configuration_descriptor = get_configuration_descriptor(false);
//     tusb_cfg.string_descriptor = get_string_descriptor(false);
//     tusb_cfg.string_descriptor_count = get_string_descriptor_count();
//     tusb_cfg.device_descriptor = get_device_descriptor();

//     ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

//     // 初始化串口
//     cdc_uart_init(UART_NUM_1, GPIO_NUM_13, GPIO_NUM_14, 115200);

//     // 创建DAP处理任务
//     xTaskCreate(DAP_Thread, "DAP_Task", 2048, NULL, 10, &kDAPTaskHandle);
// }
#include <string>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "DAP.h"
#include "wifi_handle.h"
#include "DAP_config.h"
#include <lwip/netdb.h>
#include "wifi_configuration.h"

extern TaskHandle_t kDAPTaskHandle;
extern int kRestartDAPHandle;
// extern httpd_handle_t http_server;

int kSock = -1;

// 2. 定义DAP相关变量
static uint8_t USB_Request[1][64];
static uint8_t USB_Response[1][64];

#define WIFI_SSID "ESP0000"
#define WIFI_PASS "1234567890"
#define SERVER_IP "192.168.4.1"
#define SERVER_PORT 8080

// 缓冲区定义
static uint8_t USART_RX_BUF[1280] = {0};
static uint8_t USART2_RX_BUF[64] = {0};
static uint8_t USART3_RX_BUF[512] = {0};
static uint8_t Response_RX_BUF[64] = {0};
static uint8_t datatemp[8] = {0,0,0,0,9,6,0,0};
static uint8_t data[8] = {0,0,0,0,0,0,0,0};

// 状态变量
static uint16_t USART_RX_STA = 0;
static uint16_t USART2_RX_STA = 0;
static uint16_t USART3_RX_STA = 0;
static long baud = 0;
static uint8_t Res;
static uint8_t a;
std::string str1="";

static uint16_t Flag_3=0,Flag_4=0,Flag_5=0,D1_len=0,D2_len=0,D3_len=0,len=0,t=0,num = 0;
static uint8_t num2 = 0, num3 = 0;

static const char *TAG = "wifi_tcp_client";
static int sock = -1;

// // WiFi event handler
// static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
// {
//     if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
//         esp_wifi_connect();
//     } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
//         esp_wifi_connect();
//     } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
//         ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
//         ESP_LOGI(TAG, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
//     }
// }

// GPIO初试化
static void gpio_init(void)
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL<<GPIO_NUM_2) | (1ULL<<GPIO_NUM_14) | (1ULL<<GPIO_NUM_12);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    gpio_set_level(GPIO_NUM_2, 1);
    gpio_set_level(GPIO_NUM_14, 1);
    gpio_set_level(GPIO_NUM_12, 1);
}

// UART初试化
static void uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_DEFAULT
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, 1, 3, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 1024, 1024, 0, NULL, 0));
}

void tcp_client_task(void *pvParameters)
{
    uint8_t tcp_rx_buffer[1500];
    char addr_str[128];
    int addr_family = AF_INET;
    int ip_protocol = IPPROTO_IP;
    int on = 1;
    uint8_t Res;
    
    while (1) {
        // 检查WiFi连接
        wifi_ap_record_t ap_info;
        esp_err_t status = esp_wifi_sta_get_ap_info(&ap_info);
        if (status != ESP_OK) {
            gpio_set_level(GPIO_NUM_2, 1);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        } 

        // 配置目标服务器地址
        struct sockaddr_in destAddr;
        destAddr.sin_addr.s_addr = inet_addr(SERVER_IP); // 服务器IP
        destAddr.sin_family = AF_INET;
        destAddr.sin_port = htons(SERVER_PORT);  // 服务器端口

        // 创建socket
        int sock = socket(addr_family, SOCK_STREAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // 设置socket选项
        setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, (void *)&on, sizeof(on));
        setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, (void *)&on, sizeof(on));

        // 连接服务器
        if (connect(sock, (struct sockaddr *)&destAddr, sizeof(destAddr)) != 0) {
            ESP_LOGE(TAG, "Socket connect failed errno=%d", errno);
            close(sock);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // 连接成功，设置LED
        gpio_set_level(GPIO_NUM_2, 0);
        vTaskDelay(pdMS_TO_TICKS(100));

        // 数据接收循环
        while (1) {
            uint8_t rx_byte;
            int len = recv(sock, &rx_byte, 1, 0);  // 单字节读取
            
            if (len > 0) {
                // 处理接收到的数据
                Res = rx_byte;
                if (((USART_RX_STA & 0x8000) || (USART2_RX_STA & 0x8000) || (USART3_RX_STA & 0x8000)) == 0)
                {
                    if (((USART_RX_STA & 0x4000) || (USART2_RX_STA & 0x4000) || (USART3_RX_STA & 0x4000)) == 0)
                    {
                        if (Res == 0x66)
                        {
                            USART_RX_STA |= 0x4000;
                            Flag_3 = 1;
                        }
                        else if (Res == 0x67)
                        {
                            USART3_RX_STA |= 0x4000;
                            Flag_4 = 1;
                        }
                        else if (Res == 0x68)
                        {
                            USART2_RX_STA |= 0x4000;
                            Flag_5 = 1;
                        }
                        else
                        {
                            USART_RX_STA = 0;
                            USART2_RX_STA = 0;
                            USART3_RX_STA = 0;
                        }
                    }
                    else if (USART2_RX_STA & 0x4000)
                    {
                        if (Flag_5)
                        {
                            D2_len = Res;
                            Flag_5 = 0;
                        }
                        else
                        {
                            USART2_RX_BUF[USART2_RX_STA & 0X0FFF] = Res;
                            USART2_RX_STA++;
                            if ((USART2_RX_STA & 0x0FFF) == D2_len)
                                USART2_RX_STA |= 0x8000;
                        }
                    }
                    else if ((((USART_RX_STA & 0x4000)) && (~(USART3_RX_STA & 0x4000))))
                    {
                        if (Flag_3)
                        {
                            D1_len = Res;
                            if (D1_len > 64)
                                D1_len = (D1_len - 64) * 64;
                            Flag_3 = 0;
                        }
                        else
                        {
                            USART_RX_BUF[USART_RX_STA & 0X0FFF] = Res;
                            USART_RX_STA++;
                            if ((USART_RX_STA & 0x0FFF) == D1_len)
                                USART_RX_STA |= 0x8000; //
                        }
                    }
                    else if ((((~USART_RX_STA & 0x4000)) && ((USART3_RX_STA & 0x4000))))
                    {
                        if (Flag_4)
                        {
                            D3_len = Res;
                            Flag_4 = 0;
                        }
                        else
                        {
                            USART3_RX_BUF[USART3_RX_STA & 0X0FFF] = Res;
                            USART3_RX_STA++;
                            if ((USART3_RX_STA & 0x0FFF) == D3_len)
                                USART3_RX_STA |= 0x8000; //
                        }
                    }
                }
                else if ((((USART_RX_STA & 0x8000)) && (~(USART3_RX_STA & 0x8000))))
                {
                    if ((USART3_RX_STA & 0x4000) == 0)
                    {
                        if (Res == 0x67)
                        {
                            USART3_RX_STA |= 0x4000; //
                            Flag_4 = 1;
                        }
                        else
                        {
                            USART3_RX_STA = 0;
                        }
                    }
                    else
                    {
                        if (Flag_4)
                        {
                            D3_len = Res;
                            Flag_4 = 0;
                        }
                        else
                        {
                            USART3_RX_BUF[USART3_RX_STA & 0X0FFF] = Res;
                            USART3_RX_STA++;
                            if ((USART3_RX_STA & 0x0FFF) == D3_len)
                                USART3_RX_STA |= 0x8000;
                        }
                    }
                }
                else if (((~(USART_RX_STA & 0x8000)) && ((USART3_RX_STA & 0x8000))))
                {
                    if ((USART_RX_STA & 0x4000) == 0)
                    {
                        if (Res == 0x66)
                        {
                            USART_RX_STA |= 0x4000;
                            Flag_3 = 1;
                        }
                        else
                        {
                            USART_RX_STA = 0;
                        }
                    }
                    else
                    {
                        if (Flag_3)
                        {
                            D1_len = Res;
                            if (D1_len > 64)
                                D1_len = (D1_len - 64) * 64;
                            Flag_3 = 0;
                        }
                        else
                        {
                            USART_RX_BUF[USART_RX_STA & 0X0FFF] = Res;
                            USART_RX_STA++;
                            if ((USART_RX_STA & 0x0FFF) == D1_len)
                                USART_RX_STA |= 0x8000;
                        }
                    }
                }
            }
            else if (len < 0) {
                ESP_LOGE(TAG, "recv failed: errno %d", errno);
            }
            else if (len == 0) {
                ESP_LOGI(TAG, "Connection closed");
            }

            // 检查串口数据
            size_t uart_len;
            ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM_0, &uart_len));
            if (uart_len > 0) {
                // 分配缓冲区
                uint8_t* uart_data = (uint8_t*)malloc(uart_len);
                uint8_t* send_buf = (uint8_t*)malloc(uart_len + 2); // +2 用于头部信息
                
                if (uart_data && send_buf) {
                    // 读取UART数据
                    int read_len = uart_read_bytes(UART_NUM_0, uart_data, uart_len, pdMS_TO_TICKS(20));
                    
                    if (read_len > 0) {
                        // 构建发送数据包
                        send_buf[0] = 0x67;          // 协议头
                        send_buf[1] = read_len;      // 数据长度
                        memcpy(send_buf + 2, uart_data, read_len); // 数据内容
                        
                        // 发送数据
                        vTaskDelay(pdMS_TO_TICKS(3));  // 延时3ms
                        send(sock, send_buf, read_len + 2, 0);
                        vTaskDelay(pdMS_TO_TICKS(3));  // 延时3ms
                    }
                    
                    // 释放内存
                    free(uart_data);
                    free(send_buf);
                }
            }

            if (USART2_RX_STA & 0x8000)
            {
                if (D2_len == 7)
                {
                    // 计算波特率
                    baud = (USART2_RX_BUF[0] - '0') * 1000000 + 
                        (USART2_RX_BUF[1] - '0') * 100000 + 
                        (USART2_RX_BUF[2] - '0') * 10000 + 
                        (USART2_RX_BUF[3] - '0') * 1000 + 
                        (USART2_RX_BUF[4] - '0') * 100 + 
                        (USART2_RX_BUF[5] - '0') * 10 + 
                        (USART2_RX_BUF[6] - '0') * 1;

                    // 配置UART参数
                    uart_config_t uart_config = {
                        .baud_rate = baud,
                        .data_bits = UART_DATA_8_BITS,
                        .parity = UART_PARITY_DISABLE,
                        .stop_bits = UART_STOP_BITS_1,
                        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
                        .rx_flow_ctrl_thresh = 122,
                        .source_clk = UART_SCLK_DEFAULT
                    };
                    
                    // 重新配置UART
                    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));
                }
                else if (D2_len == 1)
                {
                    num2++;
                    if (USART2_RX_BUF[0] == 2)
                    {
                        // GPIO控制
                        gpio_set_level(GPIO_NUM_14, 0);
                        gpio_set_level(GPIO_NUM_12, 0);
                        vTaskDelay(pdMS_TO_TICKS(300));
                        gpio_set_level(GPIO_NUM_14, 1);
                        gpio_set_level(GPIO_NUM_12, 1);
                        num2 = 0;
                        // 清空串口缓冲区
                        uint8_t temp_buf[256];
                        size_t length = 0;
                        ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM_0, &length));
                        if (length > 0) {
                            uart_read_bytes(UART_NUM_0, temp_buf, length, pdMS_TO_TICKS(20));
                        }
                    }
                }
                USART2_RX_STA = 0;
            }
            if (USART_RX_STA & 0x8000)
            {
                for (t = 0; t < 64; t++)
                    USB_Request[0][t] = 0;
                len = D1_len;
                USART_RX_STA = 0;
                if (len > 64)
                {
                    for (num = 0; num < len / 64; num++)
                    {
                        for (t = 0; t < 64; t++)
                            USB_Request[0][t] = USART_RX_BUF[t + num * 64];
                        DAP_ExecuteCommand(USB_Request[0], USB_Response[0]);
                    }
                    uint8_t* send_buffer = (uint8_t*)malloc(66);
                    if (send_buffer != NULL) {
                        // 填充头部
                        send_buffer[0] = 0x66;
                        send_buffer[1] = 0x40;
                        
                        // // 填充数据
                        // for (int t = 0; t < 64; t++) {
                        //     send_buffer[t + 2] = USB_Response[0][t];
                        // }
                        
                        // 也可以使用memcpy替代循环
                        memcpy(send_buffer + 2, USB_Response[0], 64);
                        
                        // 通过socket发送
                        int sent = send(sock, send_buffer, 66, 0);
                        if (sent < 0) {
                            ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                        }
                        
                        // 释放缓冲区
                        free(send_buffer);
                    } else {
                        ESP_LOGE(TAG, "Failed to allocate memory for send buffer");
                    }
                }
                else
                {
                    // 复制数据到USB请求缓冲区
                    for (t = 0; t < D1_len; t++) {
                        USB_Request[0][t] = USART_RX_BUF[t];
                    }

                    // 执行DAP命令
                    DAP_ExecuteCommand(USB_Request[0], USB_Response[0]);

                    // 计算需要发送的数据长度
                    for (t = 63; t > 0; t--) {
                        if (Response_RX_BUF[t] != USB_Response[0][t])
                            break;
                    }
                    num = t + 1;

                    // 分配发送缓冲区 (1字节0x66 + 1字节长度 + num字节数据)
                    uint8_t* send_buffer = (uint8_t*)malloc(num + 2);
                    if (send_buffer != NULL) {
                        // 填充头部
                        send_buffer[0] = 0x66;
                        send_buffer[1] = num;
                        
                        // 复制响应数据
                        memcpy(send_buffer + 2, USB_Response[0], num);
                        
                        // 发送数据
                        int sent = send(sock, send_buffer, num + 2, 0);
                        if (sent < 0) {
                            ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                        }
                        
                        // 释放缓冲区
                        free(send_buffer);
                    } else {
                        ESP_LOGE(TAG, "Failed to allocate memory for send buffer");
                    }
                }
                for (t = 0; t < 64; t++)
                    Response_RX_BUF[t] = USB_Response[0][t];
            }
            if (USART3_RX_STA & 0x8000)
            {
                len = D3_len;
                if (len == 1) {
                    if (USART3_RX_BUF[0] == 0x7f) {
                        if (num2 >= 2) {
                            // GPIO控制
                            gpio_set_level(GPIO_NUM_14, 0);
                            vTaskDelay(pdMS_TO_TICKS(100));
                            gpio_set_level(GPIO_NUM_14, 1);
                            vTaskDelay(pdMS_TO_TICKS(10));
                            
                            // 清空串口缓冲区
                            uint8_t temp_buf[256];
                            size_t length = 0;
                            ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM_0, &length));
                            if (length > 0) {
                                uart_read_bytes(UART_NUM_0, temp_buf, length, pdMS_TO_TICKS(20));
                            }
                        }
                    }
                    // 发送单字节数据
                    uart_write_bytes(UART_NUM_0, &USART3_RX_BUF[0], 1);
                    uart_wait_tx_done(UART_NUM_0, pdMS_TO_TICKS(100));
                } 
                else if (len == 2) {
                    if (USART3_RX_BUF[0] == 0x30) {
                        if (USART3_RX_BUF[1] == 0x20) {
                            if (num2 >= 2) {
                                // GPIO控制
                                gpio_set_level(GPIO_NUM_14, 0);
                                gpio_set_level(GPIO_NUM_12, 0);
                                vTaskDelay(pdMS_TO_TICKS(500));
                                gpio_set_level(GPIO_NUM_14, 1);
                                gpio_set_level(GPIO_NUM_12, 1);
                                vTaskDelay(pdMS_TO_TICKS(100));
                                
                                // 清空串口缓冲区
                                uint8_t temp_buf[256];
                                size_t length = 0;
                                ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM_0, &length));
                                if (length > 0) {
                                    uart_read_bytes(UART_NUM_0, temp_buf, length, pdMS_TO_TICKS(20));
                                }
                                vTaskDelay(pdMS_TO_TICKS(300));
                            }
                        }
                    }
                    // 发送两字节数据
                    uart_write_bytes(UART_NUM_0, USART3_RX_BUF, 2);
                    uart_wait_tx_done(UART_NUM_0, pdMS_TO_TICKS(100));
                }
                else
                {
                    // 发送多字节数据
                    uart_write_bytes(UART_NUM_0, USART3_RX_BUF, len);
                    // 等待发送完成
                    uart_wait_tx_done(UART_NUM_0, pdMS_TO_TICKS(100)); // 100ms超时
                }
                num2 = 0; // 清空接收到的次数
                USART3_RX_STA = 0;
            }

            // 检查WiFi连接状态
            if (esp_wifi_sta_get_ap_info(&ap_info) != ESP_OK) {
                gpio_set_level(GPIO_NUM_2, 1);
                for(t=0; t<64; t++)
                    Response_RX_BUF[t] = 0;
                D1_len = 0;
                break;
            }
        }

        close(sock);
    }
}

extern "C" void app_main(void)
{
    // Initialize components
    gpio_init();
    uart_init();
    ESP_ERROR_CHECK(nvs_flash_init());
    wifi_init();

    // Create main task
    xTaskCreate(tcp_client_task, "tcp_client", 4096, NULL, 5, NULL);
}