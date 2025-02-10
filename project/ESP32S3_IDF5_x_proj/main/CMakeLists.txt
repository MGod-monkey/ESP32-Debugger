idf_component_register(SRCS
                        "led.c" 
                        "main.cpp" 
                        "cdc_uart.c"
                        "msc_disk.c"
                        "usb_cdc_handler.c"
                        "usb_desc.c"
                        "prog.cpp"
                        "programmer.cpp"
                        "tcp_server.c"
                        "wifi_handle.c"
                        "usbip_server.c"
                        "DAP_handle.c"
                        "uart_bridge.c"
                        "prog_data.cpp"
                        "prog_idle.cpp"
                        "prog_online.cpp"
                        "prog_offline.cpp"
                        INCLUDE_DIRS 
                        "./"
                        "./include"
                        EMBED_FILES
                        "../html/root.html"
                        "../html/favicon.ico"
                        "../html/program.html"
                        "../html/webserial.html")

# 添加以下行来启用 C++ 支持
set(COMPONENT_REQUIRES "${COMPONENT_REQUIRES}" esp_common)
set(COMPONENT_PRIV_REQUIRES "")

# 设置 C++ 标准版本（如 11、14 或 17）
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)