utfr_declare_lib(UTFR_UART
    SOURCES    uart.c
    INCLUDES   ${CMAKE_CURRENT_LIST_DIR}
    BOARD_DEPS stm32_hal_interface
)
