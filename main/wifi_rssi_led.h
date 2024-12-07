#ifndef WIFI_RSSI_LED_H
#define WIFI_RSSI_LED_H

#include "hal_gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

void wifi_rssi_led_init(void);
void wifi_rssi_led_blink(uint32_t delay_ms, uint8_t blink_count, uint8_t r, uint8_t g, uint8_t b);
void signalLED_change_task(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif // WIFI_RSSI_LED_H