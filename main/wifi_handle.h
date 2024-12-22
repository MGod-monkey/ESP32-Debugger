#ifndef WIFI_HANDLE_H
#define WIFI_HANDLE_H

#ifdef __cplusplus
extern "C" {
#endif

// extern httpd_handle_t http_server;
extern TaskHandle_t kWifiTcpServerTaskhandle;

void wifi_init(void);
void wifi_stop(void);
void mdns_setup(void);
// void wifi_monitor_task(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif // WIFI_HANDLE_H