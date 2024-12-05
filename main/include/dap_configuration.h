#ifndef __DAP_CONFIGURATION_H__
#define __DAP_CONFIGURATION_H__

/**
 * @brief Enable this option, no need to physically connect MOSI and MISO
 *
 */
#define USE_SPI_SIO 1


/**
 * @brief Specify to enable USB 3.0
 *
 */
#define USE_USB_3_0 0


// For USB 3.0, it must be 1024 byte.
#if (USE_USB_3_0 == 1)
    #define USB_ENDPOINT_SIZE 1024U
#else
    #define USB_ENDPOINT_SIZE 512U
#endif

/**
 * @brief 接收端模式
 * SINGLE_MODE=1: 单接收端模式     
 *      有线模式：PC->数据线->接收端->SWD/JTAG
 *      无线模式：PC->elaphureLink->TCP/IP->接收端->SWD/JTAG
 *      无线串口模式：PC->WebSocket->TCP/IP->接收端->串口
 * SINGLE_MODE=0: 发送端+接收端模式
 *     有线模式：PC->数据线->发送端->数据线->接收端->SWD/JTAG
 *     无线模式：PC->发送端->TCP/IP->接收端->SWD/JTAG
 *     无线串口模式：PC->发送端->TCP/IP->接收端->串口
 */

#define SINGLE_MODE 1

#endif
