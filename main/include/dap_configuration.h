#ifndef __DAP_CONFIGURATION_H__
#define __DAP_CONFIGURATION_H__

/**
 * @brief Specify the use of WINUSB
 *
 */
#define USE_WINUSB 1

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

/// Maximum Package Size for Command and Response data.
/// This configuration settings is used to optimize the communication performance with the
/// debugger and depends on the USB peripheral. Typical vales are 64 for Full-speed USB HID or WinUSB,
/// 1024 for High-speed USB HID and 512 for High-speed USB WinUSB.

#if (USE_WINUSB == 1)
    // #define DAP_PACKET_SIZE 512U // 512 for WinUSB.
    #define DAP_PACKET_SIZE 64U // 64 for WinUSB.
#else
    #define DAP_PACKET_SIZE 255U // 255 for USB HID
#endif

/**
 * @brief 接收端模式
 * SINGLE_MODE=1: 单接收端模式     
 *      有线模式：PC->数据线->接收端->SWD/JTAG
 *      无线模式：PC->elaphureLink->tcp->接收端->SWD/JTAG
 *      无线串口模式：PC->tcp->接收端->串口
 * SINGLE_MODE=0: 发送端+接收端模式
 *     有线模式：PC->USB->发送端->数据线->接收端->SWD/JTAG
 *     无线模式：PC->发送端->tcp->接收端->SWD/JTAG
 *     无线串口模式：PC->发送端->tcp->接收端->串口
 */

#define SINGLE_MODE 0

#endif
