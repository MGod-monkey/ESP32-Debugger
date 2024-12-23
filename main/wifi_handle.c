#include "main.h"
#include "sdkconfig.h"
#include <sys/param.h>
#include "wifi_configuration.h"
#include "uart_bridge.h"
#include "web_server.h"

#include "gpio_op.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "lwip/ip4_addr.h"
#include "led.h"
#include "tcp_server.h"

// #if defined CONFIG_IDF_TARGET_ESP32S3
//     #define PIN_LED_WIFI_STATUS 4
// #else
//     #error unknown hardware
// #endif

static const char *TAG = "wifi";
static EventGroupHandle_t wifi_event_group;
extern TaskHandle_t kWifiTcpServerTaskhandle;
httpd_handle_t http_server = NULL;
static esp_netif_t *sta_netif = NULL;
static int ssid_index = 0;
extern bool tcpserver_run;

const int IPV4_GOTIP_BIT = BIT0;
#ifdef CONFIG_EXAMPLE_IPV6
const int IPV6_GOTIP_BIT = BIT1;
#endif

void mdns_setup(void) {
    // initialize mDNS
    int ret;
    ret = mdns_init();
    if (ret != ESP_OK) {
        log_printf(TAG, LOG_WARN,  "mDNS initialize failed:%d", ret);
        return;
    }

    // set mDNS hostname
    ret = mdns_hostname_set(MDNS_HOSTNAME);
    if (ret != ESP_OK) {
        log_printf(TAG, LOG_WARN,  "mDNS set hostname failed:%d", ret);
        return;
    }
    log_printf(TAG, LOG_INFO,  "mDNS hostname set to: [%s]", MDNS_HOSTNAME);

    // set default mDNS instance name
    ret = mdns_instance_name_set(MDNS_INSTANCE);
    if (ret != ESP_OK) {
        log_printf(TAG, LOG_WARN,  "mDNS set instance name failed:%d", ret);
        return;
    }
    log_printf(TAG, LOG_INFO,  "mDNS instance name set to: [%s]", MDNS_INSTANCE);
}

static void ssid_change(void)
{
    if (ssid_index > WIFI_LIST_SIZE - 1) {
        ssid_index = 0;
    }

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "",
            .password = "",
            .scan_method = WIFI_FAST_SCAN,
            .bssid_set = 0,
            .channel = 0,
            .listen_interval = 0,
            .sort_method = WIFI_CONNECT_AP_BY_SIGNAL,
            .threshold.rssi = -127,
            .threshold.authmode = WIFI_AUTH_OPEN,
        },
    };

    // 检查WiFi状态
    wifi_mode_t mode;
    if (esp_wifi_get_mode(&mode) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get WiFi mode");
        return;
    }

    strcpy((char *)wifi_config.sta.ssid, wifi_list[ssid_index].ssid);
    strcpy((char *)wifi_config.sta.password, wifi_list[ssid_index].password);
    ssid_index++;
    esp_err_t err = esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Set WiFi config failed: %s", esp_err_to_name(err));
        return;
    }
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                            int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
            case WIFI_EVENT_STA_START:
                esp_wifi_connect();
                break;
            case WIFI_EVENT_STA_CONNECTED:
                break;
            case WIFI_EVENT_STA_DISCONNECTED:
                web_server_init(&http_server);
                tcpserver_run = false;
                if (kWifiTcpServerTaskhandle != NULL) {
                    xTaskNotifyGive(kWifiTcpServerTaskhandle);
                }
                ESP_LOGI(TAG, "Disconnect reason : %d", ((wifi_event_sta_disconnected_t*)event_data)->reason);
                ssid_change();
                esp_wifi_connect();
                xEventGroupClearBits(wifi_event_group, IPV4_GOTIP_BIT);
            #if (USE_UART_BRIDGE == 1)
                uart_bridge_close();
            #endif
                break;
            default:
                break;
        }
    } else if (event_base == IP_EVENT) {
        switch (event_id) {
            case IP_EVENT_STA_GOT_IP:
                web_server_init(&http_server);
                tcpserver_run = true;  // 允许TCP任务运行
                if (kWifiTcpServerTaskhandle != NULL) {
                    xTaskNotifyGive(kWifiTcpServerTaskhandle);
                    ESP_LOGI(TAG, "TCP server task notified");  // 添加日志
                } else {
                    ESP_LOGE(TAG, "TCP server handle is NULL"); // 添加日志
                }
                ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
                ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
                xEventGroupSetBits(wifi_event_group, IPV4_GOTIP_BIT);
                break;
            #ifdef CONFIG_EXAMPLE_IPV6
                case IP_EVENT_GOT_IP6:
                    ip_event_got_ip6_t* event6 = (ip_event_got_ip6_t*) event_data;
                    ESP_LOGI(TAG, "Got IPv6: " IPV6STR, IPV62STR(event6->ip6_info.ip));
                    xEventGroupSetBits(wifi_event_group, IPV6_GOTIP_BIT);
                    break;
            #endif
        }
    }
}

// static void wait_for_ip(void)
// {
// #ifdef CONFIG_EXAMPLE_IPV6
//     uint32_t bits = IPV4_GOTIP_BIT | IPV6_GOTIP_BIT;
// #else
//     uint32_t bits = IPV4_GOTIP_BIT;
// #endif
//     ESP_LOGI(TAG, "Waiting for AP connection...");
//     xEventGroupWaitBits(wifi_event_group, bits, false, true, portMAX_DELAY);
//     ESP_LOGI(TAG, "Connected to AP");
// }

// static void disconnect_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
// {
//     web_server_stop((httpd_handle_t *)arg);
// }

// static void connect_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
// {
//     web_server_init((httpd_handle_t *)arg);
// }

void wifi_init(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    sta_netif = esp_netif_create_default_wifi_sta();
    
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                    ESP_EVENT_ANY_ID,
                                                    &wifi_event_handler,
                                                    NULL,
                                                    NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                    IP_EVENT_STA_GOT_IP,
                                                    &wifi_event_handler,
                                                    NULL,
                                                    NULL));

    mdns_setup();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ssid_change();
    ESP_ERROR_CHECK(esp_wifi_start());   
}

void wifi_stop(void)
{
    tcpserver_run = false;
    if (kWifiTcpServerTaskhandle != NULL) {
        xTaskNotifyGive(kWifiTcpServerTaskhandle);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    esp_wifi_disconnect();
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler));
    esp_wifi_stop();
    esp_wifi_deinit();
}