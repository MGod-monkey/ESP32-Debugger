#include "sdkconfig.h"

#include <string.h>
#include <stdint.h>
#include <sys/param.h>

#include "wifi_configuration.h"
#include "uart_bridge.h"

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

#if defined CONFIG_IDF_TARGET_ESP32S3
    #define PIN_LED_WIFI_STATUS 4
#else
    #error unknown hardware
#endif

static const char *TAG = "wifi";
static EventGroupHandle_t wifi_event_group;
static esp_netif_t *sta_netif = NULL;
static int ssid_index = 0;

const int IPV4_GOTIP_BIT = BIT0;
#ifdef CONFIG_EXAMPLE_IPV6
const int IPV6_GOTIP_BIT = BIT1;
#endif

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

    strcpy((char *)wifi_config.sta.ssid, wifi_list[ssid_index].ssid);
    strcpy((char *)wifi_config.sta.password, wifi_list[ssid_index].password);
    ssid_index++;
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
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
                GPIO_SET_LEVEL_LOW(PIN_LED_WIFI_STATUS);
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
                GPIO_SET_LEVEL_HIGH(PIN_LED_WIFI_STATUS);
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

static void wait_for_ip(void)
{
#ifdef CONFIG_EXAMPLE_IPV6
    uint32_t bits = IPV4_GOTIP_BIT | IPV6_GOTIP_BIT;
#else
    uint32_t bits = IPV4_GOTIP_BIT;
#endif
    ESP_LOGI(TAG, "Waiting for AP connection...");
    xEventGroupWaitBits(wifi_event_group, bits, false, true, portMAX_DELAY);
    ESP_LOGI(TAG, "Connected to AP");
}

void wifi_init(void)
{
    GPIO_FUNCTION_SET(PIN_LED_WIFI_STATUS);
    GPIO_SET_DIRECTION_NORMAL_OUT(PIN_LED_WIFI_STATUS);

    // 初始化底层TCP/IP栈
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    sta_netif = esp_netif_create_default_wifi_sta();

#if (USE_STATIC_IP == 1)
    esp_netif_dhcpc_stop(sta_netif);
    esp_netif_ip_info_t ip_info = {0};
    
    // // 方法1：使用esp_netif_str_to_ip4
    // esp_netif_str_to_ip4(DAP_IP_ADDRESS, &ip_info.ip);
    // esp_netif_str_to_ip4(DAP_IP_GATEWAY, &ip_info.gw);
    // esp_netif_str_to_ip4(DAP_IP_NETMASK, &ip_info.netmask);
    
    // 或者方法2：直接设置IP地址（推荐）
    #define MY_IP4_ADDR(...) IP4_ADDR(__VA_ARGS__)
    IP4_ADDR(&ip_info.ip, 192, 168, 137, 123);
    IP4_ADDR(&ip_info.gw, 192, 168, 137, 1);
    IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0);
    #undef MY_IP4_ADDR
    
    esp_netif_set_ip_info(sta_netif, &ip_info);
#endif

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

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    
    ssid_change();
    ESP_ERROR_CHECK(esp_wifi_start());

    wait_for_ip();
}