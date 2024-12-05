set(COMPONENT_ADD_INCLUDEDIRS "inc/")
set(COMPONENT_SRCS 
            "src/swd_iface.cpp"
            "src/target_swd.cpp"
            "src/target_flash.cpp"
            "src/flash_accessor.cpp"
			"src/bin_program.cpp"
            "src/hex_parser.c"
			"src/hex_program.cpp"
            "src/algo_extractor.cpp"
            "src/file_programmer.cpp"
            "src/stream_programmer.cpp"
			)
set(COMPONENT_REQUIRES fatfs DAP)
# 添加以下行来启用 C++ 支持
set(COMPONENT_REQUIRES "${COMPONENT_REQUIRES}" esp_common)
set(COMPONENT_PRIV_REQUIRES "")

# 设置 C++ 标准版本（如 11、14 或 17）
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

register_component()