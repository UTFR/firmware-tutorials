cmake_minimum_required(VERSION 3.22)
enable_language(C ASM)

# Read through global properties with absolute paths: this is called from the
# root scope, where a directory-scoped variable is invisible and a relative
# source would resolve against the caller.
set_property(GLOBAL PROPERTY UTFR_FREERTOS_DIR ${CMAKE_CURRENT_LIST_DIR})
set_property(GLOBAL PROPERTY UTFR_FREERTOS_SRC
    ${CMAKE_CURRENT_LIST_DIR}/src/croutine.c
    ${CMAKE_CURRENT_LIST_DIR}/src/event_groups.c
    ${CMAKE_CURRENT_LIST_DIR}/src/list.c
    ${CMAKE_CURRENT_LIST_DIR}/src/queue.c
    ${CMAKE_CURRENT_LIST_DIR}/src/stream_buffer.c
    ${CMAKE_CURRENT_LIST_DIR}/src/tasks.c
    ${CMAKE_CURRENT_LIST_DIR}/src/timers.c
    ${CMAKE_CURRENT_LIST_DIR}/src/CMSIS_RTOS_V2/cmsis_os2.c
    ${CMAKE_CURRENT_LIST_DIR}/src/portable/MemMang/heap_4.c
)
# Port is arch-specific, chosen per board in compile_freertos() below --
# ARM_CM7/r0p1 (Cortex-M7, includes a silicon errata workaround ARM_CM4F
# doesn't have) vs ARM_CM4F (Cortex-M4F). Both vendored at kernel V10.3.1,
# matching the rest of this tree.

# The only FreeRTOS setting that differs between boards.
set_property(GLOBAL PROPERTY UTFR_FREERTOS_HEAP_KIB-ACM     100)
set_property(GLOBAL PROPERTY UTFR_FREERTOS_HEAP_KIB-RC      100)
set_property(GLOBAL PROPERTY UTFR_FREERTOS_HEAP_KIB-WDAQ_RX 100)
set_property(GLOBAL PROPERTY UTFR_FREERTOS_HEAP_KIB-WDAQ_TX  84)
# FC needs more of its RAM for the logger queue and task stacks; at the larger
# value the scheduler fails to start.
set_property(GLOBAL PROPERTY UTFR_FREERTOS_HEAP_KIB-FC       56)
# H7dev now also runs the logger (logger_init() creates a 128-entry queue of
# ~136-byte messages, ~17 KiB alone) plus a LoggerTask and a foo_task on top
# of blink_task -- 8 KiB (sized back when this board really was just one
# blink task) is smaller than that one queue allocation, so logger_init()'s
# xQueueCreate() fails immediately and vApplicationMallocFailedHook() halts
# in assert_handler() before the scheduler ever gets to run anything. H7dev
# has 512 KiB of AXI SRAM total, so headroom here costs nothing.
set_property(GLOBAL PROPERTY UTFR_FREERTOS_HEAP_KIB-H7dev    64)

set_property(GLOBAL PROPERTY UTFR_FREERTOS_HEAP_KIB-LECTURE3_FREERTOS 24)
set_property(GLOBAL PROPERTY UTFR_FREERTOS_HEAP_KIB-INTRO_PROJECT     32)

function(compile_freertos board_name)
    get_property(dir  GLOBAL PROPERTY UTFR_FREERTOS_DIR)
    get_property(srcs GLOBAL PROPERTY UTFR_FREERTOS_SRC)
    get_property(heap GLOBAL PROPERTY UTFR_FREERTOS_HEAP_KIB-${board_name})

    if(NOT heap)
        message(FATAL_ERROR
            "FreeRTOS: no heap size declared for ${board_name}. Add a "
            "UTFR_FREERTOS_HEAP_KIB-${board_name} property in "
            "common/FreeRTOS/CMakeLists.txt rather than inheriting a default.")
    endif()

    set(mcu ${BOARD_MCU-${board_name}})
    utfr_mcu_get(${mcu} ARCH arch)
    if(arch STREQUAL "cortex-m7f")
        set(port_dir ${dir}/src/portable/GCC/ARM_CM7/r0p1)
    else()
        set(port_dir ${dir}/src/portable/GCC/ARM_CM4F)
    endif()

    add_library(FreeRTOS-${board_name} OBJECT)
    target_sources(FreeRTOS-${board_name} PRIVATE ${srcs} ${port_dir}/port.c)
    target_link_libraries(FreeRTOS-${board_name} PUBLIC stm32_hal_interface-${board_name})
    target_include_directories(FreeRTOS-${board_name} PUBLIC
        ${dir}/src/include
        ${dir}/src/CMSIS_RTOS_V2
        ${port_dir}
        ${CMAKE_SOURCE_DIR}/config/freertos
    )
    # PUBLIC: board sources that include FreeRTOSConfig.h must agree with the
    # value FreeRTOS itself was compiled against.
    target_compile_definitions(FreeRTOS-${board_name} PUBLIC
        configTOTAL_HEAP_KIB=${heap})
endfunction()

if((CMAKE_C_STANDARD EQUAL 90) OR (CMAKE_C_STANDARD EQUAL 99))
    message(ERROR "Generated code requires C11 or higher")
endif()
