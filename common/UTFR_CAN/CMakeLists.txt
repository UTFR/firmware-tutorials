utfr_declare_lib(UTFR_CAN
    SOURCES    can.c
    INCLUDES   ${CMAKE_CURRENT_LIST_DIR} ${CMAKE_SOURCE_DIR}/common ${AUTOGEN_DIR}
    DEPS       can_generated UTFR_UTILS
    BOARD_DEPS FreeRTOS
)
