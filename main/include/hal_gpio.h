#ifndef HAL_GPIO_H

// #define GPIO_POWER_PIN          (GPIO_NUM_2)
#define GPIO_LED_RUNNING_STATUS (GPIO_NUM_6)
#define GPIO_LED_WIFI_STATUS    (GPIO_NUM_48)

#define GPIO_DAP_SWD_MOSI       (GPIO_NUM_11)
#define GPIO_DAP_SWD_CLK        (GPIO_NUM_12)
#define GPIO_DAP_TDO            (GPIO_NUM_9)
#define GPIO_DAP_TDI            (GPIO_NUM_10)
#define GPIO_DAP_JTAG_nTRST     (GPIO_NUM_14)
#define GPIO_DAP_JTAG_nRESET    (GPIO_NUM_13)

#define UART_PORT               (UART_NUM_1)            
#define GPIO_UART_TX            (GPIO_NUM_17)
#define GPIO_UART_RX            (GPIO_NUM_18)

#define MODE_SWITCH_GPIO        (GPIO_NUM_15)
#endif // !HAL_GPIO_H