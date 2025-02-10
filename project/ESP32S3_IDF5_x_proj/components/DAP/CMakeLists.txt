set(COMPONENT_ADD_INCLUDEDIRS "include/" "config/" "cmsis-core")
set(COMPONENT_SRCS 
			"Source/DAP.c "
			"Source/DAP_vendor.c"
			"Source/JTAG_DP.c"
			"Source/SW_DP.c"
			"Source/swd_host.c"
			"Source/error.c"
			"Source/SWO.c"
			"Source/UART.c"
			"Source/spi_op.c"
			"Source/spi_switch.c"
			"Source/dap_utility.c"
			)
set(COMPONENT_REQUIRES driver main)
register_component()