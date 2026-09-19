# Keyed by chip, not by application board: the bootloader depends on flash
# geometry and the HAL family, not on what the board does. Boards sharing a
# chip share a bootloader binary. An H7 one is a sibling of this file.
set(BOARD_NAME    BOOTLOADER_STM32G474)
set(BOARD_KIND    BOOTLOADER)
set(BOARD_MCU     stm32g474)
set(BOARD_CONFIG_DIR ${CMAKE_CURRENT_LIST_DIR}/include/hal)
set(BOARD_RTOS    OFF)
set(BOARD_MODULES cortex dma fdcan flash gpio pwr rcc uart)
set(BOARD_LIBS    UTFR_BOOT_UTILS UTFR_UART)
