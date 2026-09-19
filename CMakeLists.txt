cmake_minimum_required(VERSION 3.22)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(CMAKE_PROJECT_NAME firmware-tutorials)
set(AUTOGEN_DIR ${CMAKE_BINARY_DIR}/generated)
set(COMMON_DIR ${CMAKE_SOURCE_DIR}/common)
set(CAN_DBC_FILES
    ${CMAKE_SOURCE_DIR}/dbc/front.dbc
    ${CMAKE_SOURCE_DIR}/dbc/rear.dbc
    ${CMAKE_SOURCE_DIR}/dbc/inverter.dbc
)
set(CAN_YAML_FILES
    ${CMAKE_SOURCE_DIR}/dbc/front.yaml
    ${CMAKE_SOURCE_DIR}/dbc/rear.yaml
    ${CMAKE_SOURCE_DIR}/dbc/inverter.yaml
    ${CMAKE_SOURCE_DIR}/dbc/types.yaml
)

add_compile_definitions(DO_UTFR_ASSERTS)

include(${CMAKE_SOURCE_DIR}/cmake/common.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/python.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/gcc-arm-none-eabi.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/arch.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/hal.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/board.cmake)

# A board declares its chip, its HAL modules and the libraries it wants
set(BOARD_MANIFESTS
    ${CMAKE_SOURCE_DIR}/bootloader/board_stm32g474.cmake
    ${CMAKE_SOURCE_DIR}/controllers/lecture2_concurrency/board.cmake
    ${CMAKE_SOURCE_DIR}/controllers/lecture3_freertos/board.cmake
    ${CMAKE_SOURCE_DIR}/controllers/intro_project/board.cmake
)
foreach(manifest ${BOARD_MANIFESTS})
    include(${manifest})
    list(APPEND ALL_BOARDS ${BOARD_NAME})
    if(BOARD_KIND STREQUAL BOOTLOADER)
        list(APPEND BOOTLOADER_BOARDS ${BOARD_NAME})
    else()
        list(APPEND BOARDS ${BOARD_NAME})
    endif()
    set(HAL_MODULES-${BOARD_NAME} ${BOARD_MODULES})
    set(BOARD_LIBS-${BOARD_NAME}  ${BOARD_LIBS})
    set(BOARD_MCU-${BOARD_NAME}   ${BOARD_MCU})
    utfr_load_mcu(${BOARD_MCU})
    list(APPEND ALL_MCUS ${BOARD_MCU})
    set(BOARD_RTOS-${BOARD_NAME}  ${BOARD_RTOS})
    set(HAL_CONFIG_DIR-${BOARD_NAME} ${BOARD_CONFIG_DIR})
    set(BOARD_KIND-${BOARD_NAME}  ${BOARD_KIND})
endforeach()
include(${CMAKE_SOURCE_DIR}/cmake/git.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/STM32_Programmer_CLI.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/linker.cmake)

project(${CMAKE_PROJECT_NAME})
enable_language(C ASM)
set(ORIGINAL_CMAKE_C_LINK_FLAGS ${CMAKE_C_LINK_FLAGS})

add_subdirectory(common/FreeRTOS)
add_subdirectory(common/UTFR_UTILS)
add_subdirectory(common/UTFR_CAN)
add_subdirectory(common/UTFR_UART)
add_subdirectory(common/UTFR_SPI_DRIVER)
add_subdirectory(common/UTFR_BOOT_UTILS)
add_subdirectory(common/UTFR_LOGGING)
add_subdirectory(common/UTFR_DIGITAL)

foreach(board ${ALL_BOARDS})
    utfr_add_board(${board})
endforeach()

add_subdirectory(bootloader)
add_subdirectory(controllers/lecture2_concurrency)
add_subdirectory(controllers/lecture3_freertos)
add_subdirectory(controllers/intro_project)
