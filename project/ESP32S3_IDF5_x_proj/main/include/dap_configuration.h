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
    #define DAP_PACKET_SIZE 64U // 64 for WinUSB.
#else
    #define DAP_PACKET_SIZE 255U // 255 for USB HID
#endif

/**
 * @brief Enable this option to force a software reset when resetting the device
 *
 * Some debugger software (e.g. Keil) does not perform a software reset when
 * resetting the target. When this option enabled, a soft reset is attempted
 * when DAP_ResetTarget() is performed. This is done by writing to the
 * SYSRESETREQ field of the AIRCR register in the Cortex-M architecture.
 *
 * This should work for ARMv6-m, ARMv7-m and ARMv8-m architecture. However,
 * there is no guarantee that the reset operation will be executed correctly.
 *
 * Only available for SWD.
 *
 */
#define USE_FORCE_SYSRESETREQ_AFTER_FLASH 1

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

/**
 * @brief 启动时默认的调试模式
 * 
*/
#define WIRED_MODE      1
#define WIRELESS_MODE   0

#define DEFAULT_DEBUG_MODE WIRED_MODE

/**
 * @brief 模式切换按钮阈值（ms）
 * 
*/
#define LONG_PRESS_TIME_MS 2000

#endif
