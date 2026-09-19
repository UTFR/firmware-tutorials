cmake_minimum_required(VERSION 3.22)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(BOARD_NAME LECTURE3_FREERTOS)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(LECTURE3_FREERTOS_SRC
    src/main.c
    src/tim.c
    src/foo_task.c
    src/mutex.c
    src/timer_notification.c
)

set(LECTURE3_FREERTOS_INCLUDE_DIRS
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${COMMON_DIR}
)

set(LECTURE3_FREERTOS_LINK_LIBS
    stm32_hal-${BOARD_NAME}
    FreeRTOS-${BOARD_NAME}
    git_generated
    UTFR_UTILS
    UTFR_LOGGING-${BOARD_NAME}
    UTFR_DIGITAL-${BOARD_NAME}
    UTFR_BOOT_UTILS-${BOARD_NAME}
    UTFR_UART-${BOARD_NAME}
)

set(LINKER_SCRIPT_PATH ${CMAKE_CURRENT_LIST_DIR}/../app.ld)
set(CMAKE_C_LINK_FLAGS "${ORIGINAL_CMAKE_C_LINK_FLAGS} -T \"${LINKER_SCRIPT_PATH}\"")
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,-Map=${BOARD_NAME}.map -Wl,--gc-sections")

add_executable(${BOARD_NAME})

set_target_properties(${BOARD_NAME} PROPERTIES ADDITIONAL_CLEAN_FILES ${BOARD_NAME}.map)

target_sources(${BOARD_NAME} PRIVATE ${LECTURE3_FREERTOS_SRC})
target_include_directories(${BOARD_NAME} PRIVATE ${LECTURE3_FREERTOS_INCLUDE_DIRS})
target_link_libraries(${BOARD_NAME} PRIVATE ${LECTURE3_FREERTOS_LINK_LIBS})
add_dependencies(${BOARD_NAME} linker_script_preprocess)

# --- Reference variant ---------------------------------------------------
# src/queue.c redefines motor_control_thread()/torque_calculator_thread() as
# an alternate (queue-based) take on the same problem src/mutex.c solves --
# can't link both into one executable. Compiled here as an OBJECT library to
# prove it builds against the real FreeRTOS headers; swap it into
# LECTURE3_FREERTOS_SRC above (in place of src/mutex.c) to try it for real.
add_library(lecture3_queue_variant OBJECT src/queue.c)
target_include_directories(lecture3_queue_variant PRIVATE ${LECTURE3_FREERTOS_INCLUDE_DIRS})
target_link_libraries(lecture3_queue_variant PRIVATE FreeRTOS-${BOARD_NAME})
