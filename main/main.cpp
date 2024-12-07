/*
 * @Author: wpqds666@163.com MGodmonkey
 * @Date: 2024-11-15 23:24:33
 * @LastEditors: wpqds666@163.com MGodmonkey
 * @LastEditTime: 2024-12-07 18:07:35
 * @FilePath: \ESP32-DAPLink-master\main\main.cpp
 * @Description: 
 * 
 * Copyright (c) 2024 by MGodmonkey, All Rights Reserved.
 */
#include "main.h"

static const char *TAG = "main";
TaskHandle_t kDAPTaskHandle = NULL;
TaskHandle_t kWifiTcpServerTaskhandle = NULL;
extern int kRestartDAPHandle;
extern bool tcpserver_run;

#define MODE_SWITCH_GPIO GPIO_NUM_10
#define WIRELESS_MODE 0
#define WIRED_MODE 1
#define LONG_PRESS_TIME_MS 2000  // 2秒长按阈值

static volatile bool current_mode = WIRELESS_MODE;  // 默认有线模式
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

void uart_init(void) {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    
    // 配置 UART 参数
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));
    
    // 设置 UART 引脚
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, GPIO_NUM_13, GPIO_NUM_14, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    // 安装 UART 驱动
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 256, 256, 0, NULL, 0));
}

// GPIO初试化
static void gpio_init(void)
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL<<GPIO_NUM_2) | (1ULL<<GPIO_NUM_12);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << MODE_SWITCH_GPIO);
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf);
    gpio_set_level(GPIO_NUM_2, 1);
    // gpio_set_level(GPIO_NUM_14, 1);
    gpio_set_level(GPIO_NUM_12, 1);
    gpio_set_level(MODE_SWITCH_GPIO, 1);
}

static volatile bool mode_switching = false;
static volatile bool wifi_running = false;
static volatile bool usb_running = false;

// 按键状态枚举
typedef enum {
    BUTTON_IDLE,         // 空闲状态
    BUTTON_PRESSED,      // 按下状态，正在计时
    BUTTON_TRIGGERED     // 已触发模式切换
} button_state_t;

static bool usb_initialized = false;

/**
 * @brief WiFi服务控制函数
 * @param enable: true启动WiFi服务，false停止WiFi服务
 * @return esp_err_t: ESP_OK表示成功，其他值表示错误
 */
esp_err_t wifi_service_control(bool enable) {
    if (enable) {
        if (!wifi_running) {
            wifi_init();
            // xTaskCreate(signalLED_change_task, "signalLED_change_task", 2048, NULL, 5, NULL);
            // xTaskCreate(wifi_monitor_task, "wifi_monitor", 2048, NULL, 5, NULL);
            wifi_running = true;
        }
    } else {
        if (wifi_running) {
            tcpserver_run = false;
            if (kWifiTcpServerTaskhandle != NULL) {
                xTaskNotifyGive(kWifiTcpServerTaskhandle);
            }
            // 等待一段时间确保TCP任务处理完成
            vTaskDelay(pdMS_TO_TICKS(100));
            // 停止WiFi
            ESP_ERROR_CHECK(esp_wifi_disconnect());
            ESP_ERROR_CHECK(esp_wifi_stop());
            ESP_ERROR_CHECK(esp_wifi_deinit());
            // xEventGroupClearBits(wifi_event_group, BIT0);
            wifi_running = false;
        }
    }
    return ESP_OK;
}

/**
 * @brief USB服务控制函数
 * @param enable: true启动USB服务，false停止USB服务
 * @return esp_err_t: ESP_OK表示成功，其他值表示错误
 */
#ifdef CONFIG_TINYUSB_MSC_ENABLED
esp_err_t usb_service_control(bool enable) {

    esp_err_t ret;
    if (enable) {
        if (!usb_running) {           
            log_printf(TAG, LOG_INFO, "USB initialization");
            
            if (!usb_initialized)
            {    
                tusb_cfg.configuration_descriptor = get_configuration_descriptor(true);
                tusb_cfg.string_descriptor = get_string_descriptor(true);
                tusb_cfg.string_descriptor_count = get_string_descriptor_count();
                tusb_cfg.device_descriptor = get_device_descriptor();
                usb_initialized = true;
            }
            bool mount_ret = msc_disk_mount(CONFIG_TINYUSB_MSC_MOUNT_PATH);           
            if (!mount_ret) {
                log_printf(TAG, LOG_ERROR, "Failed to mount MSC disk");
                return ESP_FAIL;
            }

            programmer_init();

           // 安装 TinyUSB 驱动
            ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
            // 初始化 CDC ACM
            ESP_ERROR_CHECK(tusb_cdc_acm_init(&acm_cfg));

            // // 初始化 UART
            // cdc_uart_init(UART_NUM_1, GPIO_NUM_13, GPIO_NUM_14, 115200);
            cdc_uart_register_rx_handler(CDC_UART_USB_HANDLER, usb_cdc_send_to_host, (void *)TINYUSB_CDC_ACM_0);

            usb_running = true;
            log_printf(TAG, LOG_INFO, "USB initialized successfully");
        }
    } else {
        if (usb_running) {
            // 停止 CDC
            tusb_stop_task();
            programmer_stop_task();
            cdc_uart_register_rx_handler(CDC_UART_USB_HANDLER, NULL, NULL);

            // // 卸载 MSC 磁盘
            bool unmount_ret = msc_disk_unmount();
            if (!unmount_ret) {
                log_printf(TAG, LOG_ERROR, "Failed to unmount MSC disk");
            } else {
                log_printf(TAG, LOG_INFO, "MSC disk unmounted successfully");
            }

            // 卸载 CDC ACM
            if (tusb_cdc_acm_deinit(TINYUSB_CDC_ACM_0) != ESP_OK) {
                log_printf(TAG, LOG_ERROR, "Failed to deinitialize CDC ACM");
            }

            // // 删除 UART 驱动
            // if (uart_driver_delete(UART_NUM_1) != ESP_OK) {
            //     log_printf(TAG, LOG_ERROR, "Failed to delete UART driver");
            // } else {
            //     log_printf(TAG, LOG_INFO, "UART driver deleted successfully");
            // }

            // 卸载 TinyUSB 驱动
            if (tinyusb_driver_uninstall() != ESP_OK) {
                log_printf(TAG, LOG_ERROR, "Failed to uninstall TinyUSB driver");
            } else {
                log_printf(TAG, LOG_INFO, "TinyUSB driver uninstalled successfully");
            }

            // 确保所有任务都已停止
            vTaskDelay(pdMS_TO_TICKS(100)); // 等待一段时间确保卸载完成

            usb_running = false;
            log_printf(TAG, LOG_INFO, "USB deinitialized successfully");
        }
    }
    return ESP_OK;
}
#else

esp_err_t usb_service_control(bool enable) {
    bool ret;
    if (enable) {
        if (!usb_running) {           
            log_printf(TAG, LOG_INFO, "USB initialization");

            // 重新挂载 MSC 磁盘
            bool mount_ret = msc_disk_mount(CONFIG_TINYUSB_MSC_MOUNT_PATH);           
            // 配置 USB 设备
            tusb_cfg.configuration_descriptor = get_configuration_descriptor(true);
            tusb_cfg.string_descriptor = get_string_descriptor(true);
            tusb_cfg.string_descriptor_count = get_string_descriptor_count();
            tusb_cfg.device_descriptor = get_device_descriptor();
            
            // 安装 TinyUSB 驱动
            ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
            // 初始化 CDC ACM
            ret = tusb_cdc_acm_init(&acm_cfg);
            if (ret != ESP_OK) {
                log_printf(TAG, LOG_ERROR, "Failed to initialize CDC ACM");
                tinyusb_driver_uninstall();
                msc_disk_unmount(); // 反初始化失败时卸载磁盘
                return ret;
            }

            // 初始化 UART
            cdc_uart_init(UART_NUM_1, GPIO_NUM_13, GPIO_NUM_14, 115200);
            cdc_uart_register_rx_handler(CDC_UART_USB_HANDLER, usb_cdc_send_to_host, (void *)TINYUSB_CDC_ACM_0);

            usb_running = true;
            log_printf(TAG, LOG_INFO, "USB initialized successfully");
        }
    } else {
        if (usb_running) {
           // 停止 CDC
            cdc_uart_register_rx_handler(CDC_UART_USB_HANDLER, NULL, NULL);
            
            // 卸载 CDC
            if (tusb_cdc_acm_deinit(TINYUSB_CDC_ACM_0) != ESP_OK) {
                log_printf(TAG, LOG_ERROR, "Failed to deinitialize CDC ACM");
            }
            vTaskDelay(pdMS_TO_TICKS(100)); // 等待 CDC 卸载

            // 卸载 TinyUSB 驱动
            if (tinyusb_driver_uninstall() != ESP_OK) {
                log_printf(TAG, LOG_ERROR, "Failed to uninstall TinyUSB driver");
            }
            vTaskDelay(pdMS_TO_TICKS(100)); // 等待 TinyUSB 驱动卸载

            // 删除 UART 驱动
            if (uart_driver_delete(UART_NUM_1) != ESP_OK) {
                log_printf(TAG, LOG_ERROR, "Failed to delete UART driver");
            } else {
                log_printf(TAG, LOG_INFO, "UART driver deleted successfully");
            }

            usb_running = false;
            log_printf(TAG, LOG_INFO, "USB deinitialization successful");
        }
    }
    return ESP_OK;
}

#endif

// 模式切换任务
static void mode_switch_task(void *pvParameters)
{
    button_state_t button_state = BUTTON_IDLE;
    TickType_t press_start_time = 0;
    const TickType_t mode_switch_threshold = pdMS_TO_TICKS(LONG_PRESS_TIME_MS);
    
    while (1) {
        int level = gpio_get_level(MODE_SWITCH_GPIO);
        TickType_t current_time = xTaskGetTickCount();
        
        switch (button_state) {
            case BUTTON_IDLE:
                if (level == 0) {  // 按键按下
                    press_start_time = current_time;
                    button_state = BUTTON_PRESSED;
                }
                break;
                
            case BUTTON_PRESSED:
                if (level == 0) {  // 按键持续按下
                    if ((current_time - press_start_time) >= mode_switch_threshold) {
                        // 达到触发时间，执行模式切换
                        button_state = BUTTON_TRIGGERED;
                        log_printf(TAG, LOG_INFO, "Mode switch triggered");
                        
                        // 蓝灯闪烁两次表示模式切换
                        for (int i = 0; i < 2; i++) {
                            // 蓝灯闪烁2次，每次延时200ms
                            wifi_rssi_led_blink(200, 2, 0, 0, 255);
                        }
                        
                        // 切换模式
                        current_mode = !current_mode;
                        
                        // 根据新模式执行初始化
                        if (current_mode == WIRELESS_MODE) {
                            log_printf(TAG, LOG_INFO, "Switching to Wireless Mode");
                            usb_service_control(false);
                            wifi_service_control(true);
                        } else {
                            log_printf(TAG, LOG_INFO, "Switching to Wired Mode");
                            wifi_service_control(false);
                            usb_service_control(true);
                        }
                    }
                } else {  // 按键释放
                    button_state = BUTTON_IDLE;
                }
                break;
                
            case BUTTON_TRIGGERED:
                if (level == 1) {  // 等待按键释放
                    button_state = BUTTON_IDLE;
                }
                break;
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));  // 10ms采样间隔
    }
}

extern "C" void app_main(void)
{
    gpio_init();
    uart_init();
    DAP_Setup();
    wifi_rssi_led_init();
    // xTaskCreate(signalLED_change_task, "signalLED_change_task", 2048, NULL, 5, NULL);
    
    // 增加模式切换任务的栈大小
    xTaskCreate(mode_switch_task, "mode_switch", 4096, NULL, 5, NULL);

    // 根据默认模式初始化
    if (current_mode == WIRELESS_MODE) {
        wifi_service_control(true);
        log_printf(TAG, LOG_INFO,  "wifi init");
    } else {
        usb_service_control(true);
        log_printf(TAG, LOG_INFO,  "usb init");
    }

    xTaskCreate(DAP_Thread, "DAP_Task", 2048, NULL, 10, &kDAPTaskHandle);
}
