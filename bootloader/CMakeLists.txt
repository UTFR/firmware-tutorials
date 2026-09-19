cmake_minimum_required(VERSION 3.22)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()
set(LINKER_SCRIPT_PATH ${CMAKE_CURRENT_LIST_DIR}/bootloader.ld)
set(CMAKE_C_LINK_FLAGS "${ORIGINAL_CMAKE_C_LINK_FLAGS} -T \"${LINKER_SCRIPT_PATH}\" -Wl,--gc-sections")

add_subdirectory(lib/cQueue)

# src/can.c and src/dfu.c implement CAN-based DFU, which is G4-only for now
# (see the UTFR_MCU_FAMILY_H7 guards in src/main.c) -- can.c hardcodes
# G474's 3-instance FDCAN config and doesn't compile against H7's FDCAN1/2.
# H7 bootloader boards get validate-and-jump only, so those two are left out
# of their source list entirely rather than compiled and unused.
set(BOOTLOADER_SRC
	src/can.c
	src/dfu.c
	src/main.c
)
set(BOOTLOADER_SRC_MINIMAL
	src/main.c
)

# One binary per chip. Boards sharing a chip share it; the source is identical,
# only the HAL family and flash geometry underneath differ.
foreach(bl ${BOOTLOADER_BOARDS})
	add_executable(${bl})

	utfr_mcu_get(${BOARD_MCU-${bl}} FAMILY bl_family)
	if(bl_family STREQUAL "stm32h7")
		target_sources(${bl} PRIVATE ${BOOTLOADER_SRC_MINIMAL})
	else()
		target_sources(${bl} PRIVATE ${BOOTLOADER_SRC})
	endif()
	target_include_directories(${bl} PRIVATE
		${CMAKE_CURRENT_LIST_DIR}/include
		${CMAKE_CURRENT_LIST_DIR}/include/hal
		${CMAKE_CURRENT_LIST_DIR}/lib
		${COMMON_DIR}
	)
	target_link_libraries(${bl} PRIVATE
		cQueue
		git_generated
		stm32_hal-${bl}
		UTFR_BOOT_UTILS-${bl}
		UTFR_UART-${bl}
		UTFR_UTILS
		can_generated_headers
	)
	target_link_options(${bl} PRIVATE -Wl,-Map=${bl}.map)
	set_target_properties(${bl} PROPERTIES ADDITIONAL_CLEAN_FILES ${bl}.map)
	add_dependencies(${bl} linker_script_preprocess)
endforeach()
