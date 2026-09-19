utfr_declare_lib(UTFR_BOOT_UTILS
    SOURCES    boot.c crc32.c flash.c image.c shared_ram.c syscalls.c vector.c
    INCLUDES   ${CMAKE_CURRENT_LIST_DIR} ${COMMON_DIR}
    DEPS       UTFR_UTILS
    BOARD_DEPS stm32_hal_interface
    DEFINES    @KIND@_IMAGE
)
