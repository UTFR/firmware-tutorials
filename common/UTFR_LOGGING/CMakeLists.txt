utfr_declare_lib(UTFR_LOGGING
    SOURCES    logger.c
    INCLUDES   ${CMAKE_CURRENT_LIST_DIR} ${COMMON_DIR}
    BOARD_DEPS FreeRTOS stm32_hal_interface UTFR_UART
)
