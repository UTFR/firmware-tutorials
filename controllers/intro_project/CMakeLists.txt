cmake_minimum_required(VERSION 3.22)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(BOARD_NAME INTRO_PROJECT)

include(${CMAKE_SOURCE_DIR}/cmake/can.cmake)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(INTRO_PROJECT_SRC
    src/main.c
    src/tim.c
    src/bsp.c
    src/intro_project.c
)

set(INTRO_PROJECT_INCLUDE_DIRS
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${COMMON_DIR}
)

set(INTRO_PROJECT_LINK_LIBS
    stm32_hal-${BOARD_NAME}
    FreeRTOS-${BOARD_NAME}
    git_generated
    UTFR_UTILS
    UTFR_LOGGING-${BOARD_NAME}
    UTFR_DIGITAL-${BOARD_NAME}
    UTFR_BOOT_UTILS-${BOARD_NAME}
    UTFR_UART-${BOARD_NAME}
    UTFR_CAN-${BOARD_NAME}
    can_generated
    can_generated_headers
)

set(LINKER_SCRIPT_PATH ${CMAKE_CURRENT_LIST_DIR}/../app.ld)
set(CMAKE_C_LINK_FLAGS "${ORIGINAL_CMAKE_C_LINK_FLAGS} -T \"${LINKER_SCRIPT_PATH}\"")
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,-Map=${BOARD_NAME}.map -Wl,--gc-sections")

add_executable(${BOARD_NAME})

set_target_properties(${BOARD_NAME} PROPERTIES ADDITIONAL_CLEAN_FILES ${BOARD_NAME}.map)

target_sources(${BOARD_NAME} PRIVATE ${INTRO_PROJECT_SRC})
target_include_directories(${BOARD_NAME} PRIVATE ${INTRO_PROJECT_INCLUDE_DIRS})
target_link_libraries(${BOARD_NAME} PRIVATE ${INTRO_PROJECT_LINK_LIBS})
add_dependencies(${BOARD_NAME} linker_script_preprocess)
