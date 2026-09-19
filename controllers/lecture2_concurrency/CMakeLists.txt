cmake_minimum_required(VERSION 3.22)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(BOARD_NAME LECTURE2_CONCURRENCY)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(LECTURE2_CONCURRENCY_SRC
    src/main.c
    src/race_condition_real_life.c
)

set(LECTURE2_CONCURRENCY_INCLUDE_DIRS
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${COMMON_DIR}
)

set(LECTURE2_CONCURRENCY_LINK_LIBS
    stm32_hal-${BOARD_NAME}
    git_generated
    UTFR_UTILS
    UTFR_DIGITAL-${BOARD_NAME}
    UTFR_BOOT_UTILS-${BOARD_NAME}
    UTFR_UART-${BOARD_NAME}
)

set(LINKER_SCRIPT_PATH ${CMAKE_CURRENT_LIST_DIR}/../app.ld)
set(CMAKE_C_LINK_FLAGS "${ORIGINAL_CMAKE_C_LINK_FLAGS} -T \"${LINKER_SCRIPT_PATH}\"")
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,-Map=${BOARD_NAME}.map -Wl,--gc-sections")

add_executable(${BOARD_NAME})

set_target_properties(${BOARD_NAME} PROPERTIES ADDITIONAL_CLEAN_FILES ${BOARD_NAME}.map)

target_sources(${BOARD_NAME} PRIVATE ${LECTURE2_CONCURRENCY_SRC})
target_include_directories(${BOARD_NAME} PRIVATE ${LECTURE2_CONCURRENCY_INCLUDE_DIRS})
target_link_libraries(${BOARD_NAME} PRIVATE ${LECTURE2_CONCURRENCY_LINK_LIBS})
add_dependencies(${BOARD_NAME} linker_script_preprocess)

# --- Reference variants -------------------------------------------------
# The rest of the original lecture handout (mutex.cpp, queue.cpp,
# race_condition_ex.cpp) can't all link into one executable -- they redefine
# the same motor_control_thread()/torque_calculator_thread() symbols as
# alternate takes on the same problem. Each is still compiled here (proving
# it builds against the real headers) as an OBJECT library that is never
# linked into LECTURE2_CONCURRENCY. Swap one into LECTURE2_CONCURRENCY_SRC
# above (in place of race_condition_real_life.c) to try it for real.

add_library(lecture2_race_condition_ex_variant OBJECT src/race_condition_ex.c)

add_library(lecture2_mutex_variant OBJECT src/mutex.c src/bare_metal_mutex.c)
target_include_directories(lecture2_mutex_variant PRIVATE ${LECTURE2_CONCURRENCY_INCLUDE_DIRS})
target_link_libraries(lecture2_mutex_variant PRIVATE stm32_hal_interface-${BOARD_NAME})

add_library(lecture2_queue_variant OBJECT src/queue.c src/bare_metal_queue.c)
target_include_directories(lecture2_queue_variant PRIVATE ${LECTURE2_CONCURRENCY_INCLUDE_DIRS})
target_link_libraries(lecture2_queue_variant PRIVATE stm32_hal_interface-${BOARD_NAME})
