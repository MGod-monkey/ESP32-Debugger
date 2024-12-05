/*
 * @Author: wpqds666@163.com MGodmonkey
 * @Date: 2024-11-15 23:24:33
 * @LastEditors: wpqds666@163.com MGodmonkey
 * @LastEditTime: 2024-12-05 10:02:59
 * @FilePath: \ESP32-DAPLink-master\main\main.cpp
 * @Description: 
 * 
 * Copyright (c) 2024 by MGodmonkey, All Rights Reserved. 
 */
#include "main.h"

static const char *TAG = "main";
TaskHandle_t kDAPTaskHandle = NULL;
extern TaskHandle_t kDAPTaskHandle;
extern int kRestartDAPHandle;

#define MODE_SWITCH_GPIO GPIO_NUM_10
#define WIRELESS_MODE 0
#define WIRED_MODE 1
#define LONG_PRESS_TIME_MS 2000  // 2秒长按阈值

static volatile bool g_current_mode = WIRELESS_MODE;  // 默认无线模式
static volatile bool g_mode_switching = false;
// extern httpd_handle_t http_server;

extern "C" void tcp_server_task(void *pvParameters);
extern "C" void DAP_Thread(void *pvParameters);

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

extern "C" uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen)
{
    return 0;
}

extern "C" void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer, uint16_t bufsize)
{
    static uint8_t s_tx_buf[CFG_TUD_HID_EP_BUFSIZE];

    DAP_ProcessCommand(buffer, s_tx_buf);
    tud_hid_report(0, s_tx_buf, sizeof(s_tx_buf));
}


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

    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << MODE_SWITCH_GPIO);
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);
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

// 信号灯变化任务
void signalLED_change_task(void *pvParameters) {
    while (1) {
        wifi_ap_record_t ap_info;
        esp_err_t status = esp_wifi_sta_get_ap_info(&ap_info);
        if (status == ESP_OK) {
            int8_t rssi = ap_info.rssi;
            int delay_time = 1000; // 默认1秒
            log_printf(TAG, LOG_DEBUG,  "rssi: %d", rssi);

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

// 按键状态枚举
typedef enum {
    BUTTON_IDLE,         // 空闲状态
    BUTTON_PRESSED,      // 按下状态，正在计时
    BUTTON_TRIGGERED     // 已触发模式切换
} button_state_t;

// // 模式切换任务
// static void mode_switch_task(void *pvParameters)
// {
//     button_state_t button_state = BUTTON_IDLE;
//     TickType_t press_start_time = 0;
//     const TickType_t mode_switch_threshold = pdMS_TO_TICKS(LONG_PRESS_TIME_MS);
    
//     while (1) {
//         int level = gpio_get_level(MODE_SWITCH_GPIO);
//         TickType_t current_time = xTaskGetTickCount();
        
//         switch (button_state) {
//             case BUTTON_IDLE:
//                 if (level == 0) {  // 按键按下
//                     press_start_time = current_time;
//                     button_state = BUTTON_PRESSED;
//                 }
//                 break;
                
//             case BUTTON_PRESSED:
//                 if (level == 0) {  // 按键持续按下
//                     if ((current_time - press_start_time) >= mode_switch_threshold) {
//                         // 达到触发时间，执行模式切换
//                         button_state = BUTTON_TRIGGERED;
                        
//                         // 蓝灯闪烁两次表示模式切换
//                         for (int i = 0; i < 2; i++) {
//                             led_strip_set_pixel(led_strip, 0, 0, 0, 255);
//                             led_strip_refresh(led_strip);
//                             vTaskDelay(pdMS_TO_TICKS(200));
//                             led_strip_set_pixel(led_strip, 0, 0, 0, 0);
//                             led_strip_refresh(led_strip);
//                             vTaskDelay(pdMS_TO_TICKS(200));
//                         }
                        
//                         // 切换模式
//                         g_current_mode = !g_current_mode;
                        
//                         // 根据新模式执行初始化
//                         if (g_current_mode == WIRELESS_MODE) {
//                             // 切换到无线模式
//                             ESP_ERROR_CHECK(nvs_flash_init());
//                             wifi_init();
//                             #if (SINGLE_MODE == 1)
//                             // Specify the usbip server task
//                                 #if (USE_TCP_NETCONN == 1)
//                                     xTaskCreate(tcp_netconn_task, "tcp_server", 4096, NULL, 14, NULL);
//                                 #else // BSD style
//                                     xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 14, NULL);
//                                 #endif
//                             #else
//                                 xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 5, NULL);
//                             #endif

//                             #if (USE_MDNS == 1)
//                                 mdns_setup();
//                             #endif
//                             // TODO: 停止USB相关任务
//                         } else {
//                             // 切换到有线模式
//                             // TODO: 停止WiFi相关任务
//                             // TODO: 初始化USB
//                         }
//                     }
//                 } else {  // 按键释放
//                     button_state = BUTTON_IDLE;
//                 }
//                 break;
                
//             case BUTTON_TRIGGERED:
//                 if (level == 1) {  // 等待按键释放
//                     button_state = BUTTON_IDLE;
//                 }
//                 break;
//         }
        
//         vTaskDelay(pdMS_TO_TICKS(10));  // 10ms采样间隔
//     }
// }

extern "C" void app_main(void)
{
    bool ret = false;
    DAP_Setup();
//     // 无线模式
//     ESP_ERROR_CHECK(nvs_flash_init());

//     #if (USE_UART_BRIDGE == 1)
//         uart_bridge_init();
//     #endif
//     wifi_init();

// #if (SINGLE_MODE == 1)
//     // Specify the usbip server task
//     #if (USE_TCP_NETCONN == 1)
//         xTaskCreate(tcp_netconn_task, "tcp_server", 4096, NULL, 14, NULL);
//     #else // BSD style
//         xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 14, NULL);
//     #endif
// #else
//     xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 5, NULL);
// #endif
//     xTaskCreate(signalLED_change_task, "signalLED_change_task", 2048, NULL, 5, NULL);

//     #if (USE_MDNS == 1)
//         mdns_setup();
//     #endif

    // 有线模式
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

    log_printf(TAG, LOG_INFO,  "USB initialization");

    ret = msc_dick_mount(CONFIG_TINYUSB_MSC_MOUNT_PATH);
    tusb_cfg.configuration_descriptor = get_configuration_descriptor(ret);
    tusb_cfg.string_descriptor = get_string_descriptor(ret);
    tusb_cfg.string_descriptor_count = get_string_descriptor_count();
    tusb_cfg.device_descriptor = get_device_descriptor();

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    ESP_ERROR_CHECK(tusb_cdc_acm_init(&acm_cfg));

    programmer_init();
    cdc_uart_init(UART_NUM_1, GPIO_NUM_13, GPIO_NUM_14, 115200);
    cdc_uart_register_rx_handler(CDC_UART_USB_HANDLER, usb_cdc_send_to_host, (void *)TINYUSB_CDC_ACM_0);
    // cdc_uart_register_rx_handler(CDC_UART_WEB_HANDLER, web_send_to_clients, &http_server);
    log_printf(TAG, LOG_INFO,  "USB initialization DONE");

    // DAP handle task
    xTaskCreate(DAP_Thread, "DAP_Task", 2048, NULL, 10, &kDAPTaskHandle);
    // // Initialize components
    // gpio_init();
    // // 初始化信号灯
    // xTaskCreate(signalLED_change_task, "signalLED_change_task", 2048, NULL, 5, NULL);
    // // 创建模式切换任务
    // xTaskCreate(mode_switch_task, "mode_switch", 2048, NULL, 5, NULL);
    
    // // 根据默认模式初始化
    // if (g_current_mode == WIRELESS_MODE) {
    //     // 无线模式初始化
    //     ESP_ERROR_CHECK(nvs_flash_init());
    //     wifi_init();
    // } else {
    //     // 有线模式初始化
    //     // USB相关初始化
    // }
    // ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));

    // uart_init();
}