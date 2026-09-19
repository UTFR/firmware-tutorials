set(BOARD_NAME    LECTURE2_CONCURRENCY)
set(BOARD_KIND    APP)
set(BOARD_MCU     stm32g474)
set(BOARD_CONFIG_DIR ${CMAKE_CURRENT_LIST_DIR}/include/hal)
# Deliberately no RTOS here -- this lecture teaches the concept of a race
# condition before FreeRTOS is introduced next lecture (see
# controllers/lecture3_freertos). The "two concurrent flows of control" are
# realized as a bare-metal mainline-vs-timer-ISR race instead.
set(BOARD_RTOS    OFF)
set(BOARD_MODULES cortex dma flash gpio pwr rcc tim uart)
set(BOARD_LIBS    UTFR_BOOT_UTILS UTFR_UART UTFR_DIGITAL)
