utfr_declare_lib(UTFR_SPI_DRIVER
    SOURCES            spi_driver.c
    INCLUDES           ${CMAKE_CURRENT_LIST_DIR}
    PRIVATE_BOARD_DEPS stm32_hal FreeRTOS UTFR_LOGGING UTFR_DIGITAL
)
