utfr_declare_lib(UTFR_DIGITAL
    SOURCES          pin_driver.c
    PRIVATE_INCLUDES ${CMAKE_CURRENT_LIST_DIR}
    DEPS             UTFR_UTILS
    BOARD_DEPS       stm32_hal_interface
)
