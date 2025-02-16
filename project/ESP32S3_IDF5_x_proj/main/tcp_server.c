/**
 * @file tcp_server.c
 * @brief Handle main tcp tasks
 * @version 0.1
 * @date 2020-01-22
 *
 * @copyright Copyright (c) 2020
 *
 */
#include <sys/param.h>
#include "main/usbip_server.h"
#include "main/DAP_handle.h"

#include "components/elaphureLink/elaphureLink_protocol.h"

#include "main.h"

extern TaskHandle_t kDAPTaskHandle;
extern int kRestartDAPHandle;
extern TaskHandle_t kWifiTcpServerTaskhandle;

bool tcpserver_run = false;
enum state_t kState = ACCEPTING;
int kSock = -1;

static const char *TAG = "tcp_server";

#if SINGLE_MODE == 1
    void tcp_server_task(void *pvParameters)
    {
        uint8_t tcp_rx_buffer[1500] = {0};
        char addr_str[128];
        enum usbip_server_state_t usbip_state = WAIT_DEVLIST;
        uint8_t *data;
        int addr_family; 
        int ip_protocol;
        int header;
        int ret, sz;

        int on = 1;
        while (1) {
            // 等待通知
            ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
            while (tcpserver_run)
            {
            #ifdef CONFIG_EXAMPLE_IPV4
                struct sockaddr_in destAddr;
                destAddr.sin_addr.s_addr = htonl(INADDR_ANY);
                destAddr.sin_family = AF_INET;
                destAddr.sin_port = htons(WIFI_PORT);
                addr_family = AF_INET;
                ip_protocol = IPPROTO_IP;
                inet_ntoa_r(destAddr.sin_addr, addr_str, sizeof(addr_str) - 1);
            #else // IPV6
                struct sockaddr_in6 destAddr;
                bzero(&destAddr.sin6_addr.un, sizeof(destAddr.sin6_addr.un));
                destAddr.sin6_family = AF_INET6;
                destAddr.sin6_port = htons(WIFI_PORT);
                addr_family = AF_INET6;
                ip_protocol = IPPROTO_IPV6;
                inet6_ntoa_r(destAddr.sin6_addr, addr_str, sizeof(addr_str) - 1);
            #endif

                int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
                if (listen_sock < 0)
                {
                    os_printf("Unable to create socket: errno %d\r\n", errno);
                    break;
                }
                os_printf("Socket created\r\n");

                setsockopt(listen_sock, SOL_SOCKET, SO_KEEPALIVE, (void *)&on, sizeof(on));
                setsockopt(listen_sock, IPPROTO_TCP, TCP_NODELAY, (void *)&on, sizeof(on));

                int err = bind(listen_sock, (struct sockaddr *)&destAddr, sizeof(destAddr));
                if (err != 0)
                {
                    os_printf("Socket unable to bind: errno %d\r\n", errno);
                    break;
                }
                os_printf("Socket binded\r\n");

                err = listen(listen_sock, 1);
                if (err != 0)
                {
                    os_printf("Error occured during listen: errno %d\r\n", errno);
                    break;
                }
                os_printf("Socket listening\r\n");

                #ifdef CONFIG_EXAMPLE_IPV6
                    struct sockaddr_in6 sourceAddr; // Large enough for both IPv4 or IPv6
                #else
                    struct sockaddr_in sourceAddr;
                #endif
                uint32_t addrLen = sizeof(sourceAddr);
                while (tcpserver_run)
                {
                    kSock = accept(listen_sock, (struct sockaddr *)&sourceAddr, &addrLen);
                    if (kSock < 0)
                    {
                        os_printf("Unable to accept connection: errno %d\r\n", errno);
                        break;
                    }
                    setsockopt(kSock, SOL_SOCKET, SO_KEEPALIVE, (void *)&on, sizeof(on));
                    setsockopt(kSock, IPPROTO_TCP, TCP_NODELAY, (void *)&on, sizeof(on));
                    os_printf("Socket accepted\r\n");

                    // Read header
                    sz = 4;
                    data = &tcp_rx_buffer[0];
                    do {
                        ret = recv(kSock, data, sz, 0);
                        if (ret <= 0)
                            goto cleanup;
                        sz -= ret;
                        data += ret;
                    } while (sz > 0);

                    header = *((int *)(tcp_rx_buffer));
                    header = ntohl(header);

                    if (header == EL_LINK_IDENTIFIER) {
                        el_dap_work(tcp_rx_buffer, sizeof(tcp_rx_buffer));
                    } else if ((header & 0xFFFF) == 0x8003 ||
                            (header & 0xFFFF) == 0x8005) { // usbip OP_REQ_DEVLIST/OP_REQ_IMPORT
                        if ((header & 0xFFFF) == 0x8005)
                            usbip_state = WAIT_DEVLIST;
                        else
                            usbip_state = WAIT_IMPORT;
                        usbip_worker(tcp_rx_buffer, sizeof(tcp_rx_buffer), &usbip_state);
                    } else if (header == 0x47455420) { // string "GET "
                        #ifdef CONFIG_USE_WEBSOCKET_DAP
                            websocket_worker(kSock, tcp_rx_buffer, sizeof(tcp_rx_buffer));
                        #endif
                    } else {
                        os_printf("Unknown protocol\n");
                    }

                cleanup:
                    if (kSock != -1)
                    {
                        os_printf("Shutting down socket and restarting...\r\n");
                        //shutdown(kSock, 0);
                        close(kSock);

                        // Restart DAP Handle
                        el_process_buffer_free();

                        kRestartDAPHandle = RESET_HANDLE;
                        if (kDAPTaskHandle)
                            xTaskNotifyGive(kDAPTaskHandle);

                        //shutdown(listen_sock, 0);
                        //close(listen_sock);
                        //vTaskDelay(5);
                    }
                }
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        // vTaskDelete(NULL);
    }

#else
    #define SERVER_IP "192.168.8.1"
    #define SERVER_PORT 8080

    // 缓冲区定义
    static uint8_t USART_RX_BUF[1280] = {0};
    static uint8_t USART2_RX_BUF[64] = {0};
    static uint8_t USART3_RX_BUF[512] = {0};
    static uint8_t Response_RX_BUF[64] = {0};
    static uint8_t datatemp[8] = {0,0,0,0,9,6,0,0};
    static uint8_t data[8] = {0,0,0,0,0,0,0,0};

    // 状态变量
    static uint16_t USART_RX_STA = 0;
    static uint16_t USART2_RX_STA = 0;
    static uint16_t USART3_RX_STA = 0;
    static long baud = 0;
    static uint8_t Res;
    static uint8_t a;

    // 定义DAP相关变量
    static uint8_t USB_Request[1][64];
    static uint8_t USB_Response[1][64];

    static uint16_t Flag_3=0,Flag_4=0,Flag_5=0,D1_len=0,D2_len=0,D3_len=0,len=0,t=0,num = 0;
    static uint8_t num2 = 0, num3 = 0;
    static int sock = -1;

    void tcp_server_task(void *pvParameters)
    {
        uint8_t tcp_rx_buffer[1500];
        char addr_str[128];
        int addr_family = AF_INET;
        int ip_protocol = IPPROTO_IP;
        int on = 1;
        uint8_t Res;
        
        while (1) {
            // 等待通知
            ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
            
            while (tcpserver_run) {
                // 检查WiFi连接
                wifi_ap_record_t ap_info;
                esp_err_t status = esp_wifi_sta_get_ap_info(&ap_info);
                if (status != ESP_OK) {
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    continue;
                } 

            // 配置目标服务器地址
            struct sockaddr_in destAddr;
            destAddr.sin_addr.s_addr = inet_addr(SERVER_IP); // 服务器IP
            destAddr.sin_family = AF_INET;
            destAddr.sin_port = htons(SERVER_PORT);  // 服务器端口

            // 创建socket
            int sock = socket(addr_family, SOCK_STREAM, ip_protocol);
            if (sock < 0) {
                log_printf(TAG, LOG_ERROR, "Unable to create socket: errno %d", errno);
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }

            // 设置socket选项
            setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, (void *)&on, sizeof(on));
            setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, (void *)&on, sizeof(on));

            // 连接服务器
            if (connect(sock, (struct sockaddr *)&destAddr, sizeof(destAddr)) != 0) {
                log_printf(TAG, LOG_ERROR, "Socket connect failed errno=%d", errno);
                close(sock);
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }

            // 连接成功
            vTaskDelay(pdMS_TO_TICKS(100));

            // 数据接收循环
            while (1) {
                fd_set readfds;
                struct timeval tv = {
                    .tv_sec = 0,
                    .tv_usec = 1000  // 1ms超时
                };

                FD_ZERO(&readfds);
                FD_SET(sock, &readfds);

                int activity = select(sock + 1, &readfds, NULL, NULL, &tv);

                if (activity > 0 && FD_ISSET(sock, &readfds)) {
                uint8_t rx_byte;
                int len = recv(sock, &rx_byte, 1, 0);  // 单字节读取
                if (len > 0) {
                    Res = rx_byte;
                    if (((USART_RX_STA & 0x8000) || (USART2_RX_STA & 0x8000) || (USART3_RX_STA & 0x8000)) == 0)
                    {
                        if (((USART_RX_STA & 0x4000) || (USART2_RX_STA & 0x4000) || (USART3_RX_STA & 0x4000)) == 0)
                        {
                            if (Res == 0x66)
                            {
                                USART_RX_STA |= 0x4000;
                                Flag_3 = 1;
                            }
                            else if (Res == 0x67)
                            {
                                USART3_RX_STA |= 0x4000;
                                Flag_4 = 1;
                            }
                            else if (Res == 0x68)
                            {
                                USART2_RX_STA |= 0x4000;
                                Flag_5 = 1;
                            }
                            else
                            {
                                USART_RX_STA = 0;
                                USART2_RX_STA = 0;
                                USART3_RX_STA = 0;
                            }
                        }
                        else if (USART2_RX_STA & 0x4000)
                        {
                            if (Flag_5)
                            {
                                D2_len = Res;
                                Flag_5 = 0;
                            }
                            else
                            {
                                USART2_RX_BUF[USART2_RX_STA & 0X0FFF] = Res;
                                USART2_RX_STA++;
                                if ((USART2_RX_STA & 0x0FFF) == D2_len)
                                    USART2_RX_STA |= 0x8000;
                            }
                        }
                        else if ((((USART_RX_STA & 0x4000)) && (~(USART3_RX_STA & 0x4000))))
                        {
                            if (Flag_3)
                            {
                                D1_len = Res;
                                if (D1_len > 64)
                                    D1_len = (D1_len - 64) * 64;
                                Flag_3 = 0;
                            }
                            else
                            {
                                USART_RX_BUF[USART_RX_STA & 0X0FFF] = Res;
                                USART_RX_STA++;
                                if ((USART_RX_STA & 0x0FFF) == D1_len)
                                    USART_RX_STA |= 0x8000; //
                            }
                        }
                        else if ((((~USART_RX_STA & 0x4000)) && ((USART3_RX_STA & 0x4000))))
                        {
                            if (Flag_4)
                            {
                                D3_len = Res;
                                Flag_4 = 0;
                            }
                            else
                            {
                                USART3_RX_BUF[USART3_RX_STA & 0X0FFF] = Res;
                                USART3_RX_STA++;
                                if ((USART3_RX_STA & 0x0FFF) == D3_len)
                                    USART3_RX_STA |= 0x8000; //
                            }
                        }
                    }
                    else if ((((USART_RX_STA & 0x8000)) && (~(USART3_RX_STA & 0x8000))))
                    {
                        if ((USART3_RX_STA & 0x4000) == 0)
                        {
                            if (Res == 0x67)
                            {
                                USART3_RX_STA |= 0x4000; //
                                Flag_4 = 1;
                            }
                            else
                            {
                                USART3_RX_STA = 0;
                            }
                        }
                        else
                        {
                            if (Flag_4)
                            {
                                D3_len = Res;
                                Flag_4 = 0;
                            }
                            else
                            {
                                USART3_RX_BUF[USART3_RX_STA & 0X0FFF] = Res;
                                USART3_RX_STA++;
                                if ((USART3_RX_STA & 0x0FFF) == D3_len)
                                    USART3_RX_STA |= 0x8000;
                            }
                        }
                    }
                    else if (((~(USART_RX_STA & 0x8000)) && ((USART3_RX_STA & 0x8000))))
                    {
                        if ((USART_RX_STA & 0x4000) == 0)
                        {
                            if (Res == 0x66)
                            {
                                USART_RX_STA |= 0x4000;
                                Flag_3 = 1;
                            }
                            else
                            {
                                USART_RX_STA = 0;
                            }
                        }
                        else
                        {
                            if (Flag_3)
                            {
                                D1_len = Res;
                                if (D1_len > 64)
                                    D1_len = (D1_len - 64) * 64;
                                Flag_3 = 0;
                            }
                            else
                            {
                                USART_RX_BUF[USART_RX_STA & 0X0FFF] = Res;
                                USART_RX_STA++;
                                if ((USART_RX_STA & 0x0FFF) == D1_len)
                                    USART_RX_STA |= 0x8000;
                            }
                        }
                    }
                }
                else if (len < 0) {
                    // log_printf(TAG, LOG_ERROR, "recv failed: errno %d", errno);
                    break;
                    // vTaskDelay(pdMS_TO_TICKS(1000));
                }
                else if (len == 0) {
                    log_printf(TAG, LOG_INFO, "Connection closed");
                }
                }

                // 检查串口数据
                size_t uart_len;
                ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_PORT, &uart_len));
                if (uart_len > 0) {
                    uint8_t* uart_data = (uint8_t*)malloc(uart_len);
                    uint8_t* send_buf = (uint8_t*)malloc(uart_len + 2);
                    
                    if (uart_data && send_buf) {
                        int read_len = uart_read_bytes(UART_PORT, uart_data, uart_len, pdMS_TO_TICKS(20));
                        if (read_len > 0) {
                            send_buf[0] = 0x67;
                            send_buf[1] = read_len;
                            memcpy(send_buf + 2, uart_data, read_len);
                            
                            send(sock, send_buf, read_len + 2, 0);
                        }
                        free(uart_data);
                        free(send_buf);
                    }
                }

                if (USART2_RX_STA & 0x8000)
                {
                    if (D2_len == 7)
                    {
                    // 计算波特率
                    baud = (USART2_RX_BUF[0] - '0') * 1000000 + 
                        (USART2_RX_BUF[1] - '0') * 100000 + 
                        (USART2_RX_BUF[2] - '0') * 10000 + 
                        (USART2_RX_BUF[3] - '0') * 1000 + 
                        (USART2_RX_BUF[4] - '0') * 100 + 
                        (USART2_RX_BUF[5] - '0') * 10 + 
                        (USART2_RX_BUF[6] - '0') * 1;

                    // 验证波特率范围
                    if (baud < 110 || baud > 4500000) {
                        baud = 115200; // 使用默认波特率
                    }

                    // 配置UART参数
                    uart_config_t uart_config = {
                        .baud_rate = baud,
                        .data_bits = UART_DATA_8_BITS,
                        .parity = UART_PARITY_DISABLE,
                        .stop_bits = UART_STOP_BITS_1,
                        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
                        .rx_flow_ctrl_thresh = 122,
                        .source_clk = UART_SCLK_DEFAULT
                    };
                    
                    // 使用错误处理宏替代ESP_ERROR_CHECK
                    esp_err_t err = uart_param_config(UART_PORT, &uart_config);
                    if (err != ESP_OK) {
                        ESP_LOGE(TAG, "Failed to configure UART parameters: %d", err);
                        // 恢复默认配置
                        uart_config.baud_rate = 115200;
                        uart_param_config(UART_PORT, &uart_config);
                    }
                    }
                    else if (D2_len == 1)
                    {
                        num2++;
                        if (USART2_RX_BUF[0] == 2)
                        {
                            // GPIO控制
                            gpio_set_level(GPIO_NUM_14, 0);
                            gpio_set_level(GPIO_NUM_12, 0);
                            vTaskDelay(pdMS_TO_TICKS(300));
                            gpio_set_level(GPIO_NUM_14, 1);
                            gpio_set_level(GPIO_NUM_12, 1);
                            num2 = 0;
                            // 清空串口缓冲区
                            uint8_t temp_buf[256];
                            size_t length = 0;
                            ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_PORT, &length));
                            if (length > 0) {
                                uart_read_bytes(UART_PORT, temp_buf, length, pdMS_TO_TICKS(20));
                            }
                        }
                    }
                    USART2_RX_STA = 0;
                }
                if (USART_RX_STA & 0x8000)
                {
                    for (t = 0; t < 64; t++)
                        USB_Request[0][t] = 0;
                    len = D1_len;
                    USART_RX_STA = 0;
                    if (len > 64)
                    {
                        for (num = 0; num < len / 64; num++)
                        {
                            for (t = 0; t < 64; t++)
                                USB_Request[0][t] = USART_RX_BUF[t + num * 64];
                            DAP_ExecuteCommand(USB_Request[0], USB_Response[0]);
                        }
                        uint8_t* send_buffer = (uint8_t*)malloc(66);
                        if (send_buffer != NULL) {
                            // 填充头部
                            send_buffer[0] = 0x66;
                            send_buffer[1] = 0x40;
                            
                            // // 填充数据
                            // for (int t = 0; t < 64; t++) {
                            //     send_buffer[t + 2] = USB_Response[0][t];
                            // }
                            
                            // 也可以使用memcpy替代循环
                            memcpy(send_buffer + 2, USB_Response[0], 64);
                            
                            // 通过socket发送
                            int sent = send(sock, send_buffer, 66, 0);
                            if (sent < 0) {
                                log_printf(TAG, LOG_ERROR,  "Error occurred during sending: errno %d", errno);
                            }
                            
                            // 释放缓冲区
                            free(send_buffer);
                        } else {
                            log_printf(TAG, LOG_ERROR,  "Failed to allocate memory for send buffer");
                        }
                    }
                    else
                    {
                        // 复制数据到USB请求缓冲区
                        for (t = 0; t < D1_len; t++) {
                            USB_Request[0][t] = USART_RX_BUF[t];
                        }

                        // 执行DAP命令
                        DAP_ExecuteCommand(USB_Request[0], USB_Response[0]);

                        // 计算需要发送的数据长度
                        for (t = 63; t > 0; t--) {
                            if (Response_RX_BUF[t] != USB_Response[0][t])
                                break;
                        }
                        num = t + 1;

                        // 分配发送缓冲区 (1字节0x66 + 1字节长度 + num字节数据)
                        uint8_t* send_buffer = (uint8_t*)malloc(num + 2);
                        if (send_buffer != NULL) {
                            // 填充头部
                            send_buffer[0] = 0x66;
                            send_buffer[1] = num;
                            
                            // 复制响应数据
                            memcpy(send_buffer + 2, USB_Response[0], num);
                            
                            // 发送数据
                            int sent = send(sock, send_buffer, num + 2, 0);
                            if (sent < 0) {
                                log_printf(TAG, LOG_ERROR,  "Error occurred during sending: errno %d", errno);
                            }
                            
                            // 释放缓冲区
                            free(send_buffer);
                        } else {
                            log_printf(TAG, LOG_ERROR,  "Failed to allocate memory for send buffer");
                        }
                    }
                    for (t = 0; t < 64; t++)
                        Response_RX_BUF[t] = USB_Response[0][t];
                }
                if (USART3_RX_STA & 0x8000)
                {
                    len = D3_len;
                    if (len == 1) {
                        if (USART3_RX_BUF[0] == 0x7f) {
                            if (num2 >= 2) {
                                // GPIO控制
                                gpio_set_level(GPIO_NUM_14, 0);
                                vTaskDelay(pdMS_TO_TICKS(100));
                                gpio_set_level(GPIO_NUM_14, 1);
                                vTaskDelay(pdMS_TO_TICKS(10));
                                
                                // 清空串口缓冲区
                                uint8_t temp_buf[256];
                                size_t length = 0;
                                ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_PORT, &length));
                                if (length > 0) {
                                    uart_read_bytes(UART_PORT, temp_buf, length, pdMS_TO_TICKS(20));
                                }
                            }
                        }
                        // 发送单字节数据
                        uart_write_bytes(UART_PORT, &USART3_RX_BUF[0], 1);
                        uart_wait_tx_done(UART_PORT, pdMS_TO_TICKS(100));
                    } 
                    else if (len == 2) {
                        if (USART3_RX_BUF[0] == 0x30) {
                            if (USART3_RX_BUF[1] == 0x20) {
                                if (num2 >= 2) {
                                    // GPIO控制
                                    gpio_set_level(GPIO_NUM_14, 0);
                                    gpio_set_level(GPIO_NUM_12, 0);
                                    vTaskDelay(pdMS_TO_TICKS(500));
                                    gpio_set_level(GPIO_NUM_14, 1);
                                    gpio_set_level(GPIO_NUM_12, 1);
                                    vTaskDelay(pdMS_TO_TICKS(100));
                                    
                                    // 清空串口缓冲区
                                    uint8_t temp_buf[256];
                                    size_t length = 0;
                                    ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_PORT, &length));
                                    if (length > 0) {
                                        uart_read_bytes(UART_PORT, temp_buf, length, pdMS_TO_TICKS(20));
                                    }
                                    vTaskDelay(pdMS_TO_TICKS(300));
                                }
                            }
                        }
                        // 发送两字节数据
                        uart_write_bytes(UART_PORT, USART3_RX_BUF, 2);
                        uart_wait_tx_done(UART_PORT, pdMS_TO_TICKS(100));
                    }
                    else
                    {
                        // 发送多字节数据
                        uart_write_bytes(UART_PORT, USART3_RX_BUF, len);
                        // 等待发送完成
                        uart_wait_tx_done(UART_PORT, pdMS_TO_TICKS(100)); // 100ms超时
                    }
                    num2 = 0; // 清空接收到的次数
                    USART3_RX_STA = 0;
                }

                // 检查WiFi连接状态
                if (esp_wifi_sta_get_ap_info(&ap_info) != ESP_OK) {
                    for(t=0; t<64; t++)
                        Response_RX_BUF[t] = 0;
                    D1_len = 0;
                    break;
                }
            }
            // 检查是否应该继续运行
            if (!tcpserver_run) {
                if (sock >= 0) {
                    close(sock);
                }
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
#endif