/*
 * @Author: wpqds666@163.com MGodmonkey
 * @Date: 2024-11-15 23:24:33
 * @LastEditors: wpqds666@163.com MGodmonkey
 * @LastEditTime: 2024-12-23 16:01:21
 * @FilePath: \ESP32-Debugger\main\main.cpp
 * @Description: 
 * 
 * Copyright (c) 2024 by MGodmonkey, All Rights Reserved.
 */
#include "main.h"

static const char *TAG = "main";
TaskHandle_t kDAPTaskHandle = NULL;
TaskHandle_t ws2812_task_handle = NULL;
TaskHandle_t kWifiTcpServerTaskhandle = NULL;
// extern int kRestartDAPHandle;

static volatile bool current_mode = DEFAULT_DEBUG_MODE;
static RTC_NOINIT_ATTR uint8_t laster_mode;
static RTC_NOINIT_ATTR uint8_t is_first_boot;
// extern httpd_handle_t http_server;

extern "C" void tcp_server_task(void *pvParameters);
extern "C" void DAP_Thread(void *pvParameters);

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

// 有线调试配置
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

// 无线调试串口初试化
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
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, GPIO_UART_TX, GPIO_UART_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, 1024, 1024, 0, NULL, 0));
}

// 按钮初试化
static void key_init(void)
{
    gpio_config_t io_conf = {};

    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << MODE_SWITCH_GPIO);
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf);
    gpio_set_level(MODE_SWITCH_GPIO, 1);
}

// 按键状态枚举
typedef enum {
    BUTTON_IDLE,         // 空闲状态
    BUTTON_PRESSED,      // 按下状态，正在计时
    BUTTON_TRIGGERED     // 已触发模式切换
} button_state_t;


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

                        if (current_mode == WIRELESS_MODE)
                        {
                            ws2812_task_handle = NULL;
                            ws2812_set_color(LED_COLOR_BLACK);
                            wifi_stop();
                        }

                        // 保存新模式
                        laster_mode = !current_mode;
                        
                        // 橙灯闪烁两次表示模式切换
                        ws2812_blink(500, 2, LED_COLOR_ORANGE);
                        
                        // 执行软复位
                        esp_restart();
                        
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

    // 检查是否首次启动
    if (is_first_boot != 0x55) {
        is_first_boot = 0x55;
        laster_mode = DEFAULT_DEBUG_MODE;
    }
    
    current_mode = laster_mode;
    ws2812_init();
    ws2812_breath(LED_COLOR_ORANGE, 2000);
    key_init();
    DAP_Setup();
    
    // 增加模式切换任务的栈大小
    xTaskCreate(mode_switch_task, "mode_switch", 4096, NULL, 5, NULL);

    // 根据默认模式初始化
    if (current_mode == WIRELESS_MODE)
    {
        cdc_uart_init(UART_PORT, GPIO_UART_TX, GPIO_UART_RX, 115200, false);
        wifi_init();

        xTaskCreate(ws2812_task, "ws2812_task", 4096, NULL, 5, &ws2812_task_handle);
        xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 14, &kWifiTcpServerTaskhandle);
    }
    else 
    {
        bool mount_ret = msc_disk_mount(CONFIG_TINYUSB_MSC_MOUNT_PATH);

        if (!mount_ret) {
            log_printf(TAG, LOG_ERROR, "Failed to mount MSC disk");
            ws2812_set_color(LED_COLOR_RED);
        }
        tusb_cfg.configuration_descriptor = get_configuration_descriptor(mount_ret);
        tusb_cfg.string_descriptor = get_string_descriptor(mount_ret);
        tusb_cfg.string_descriptor_count = get_string_descriptor_count();
        tusb_cfg.device_descriptor = get_device_descriptor();

        // 安装 TinyUSB 驱动
        ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
        // 初始化 CDC ACM
        ESP_ERROR_CHECK(tusb_cdc_acm_init(&acm_cfg));

        programmer_init();
        
        // 初始化 CDC UART
        cdc_uart_init(UART_PORT, GPIO_UART_TX, GPIO_UART_RX, 115200, true);
        cdc_uart_register_rx_handler(CDC_UART_USB_HANDLER, usb_cdc_send_to_host, (void *)TINYUSB_CDC_ACM_0);

        ws2812_set_color(LED_COLOR_PURPLE);

        log_printf(TAG, LOG_INFO, "USB initialized successfully");
    }

    xTaskCreate(DAP_Thread, "DAP_Task", 2048, NULL, 10, &kDAPTaskHandle);
}
