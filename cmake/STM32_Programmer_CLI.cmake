add_custom_target(
    flash
    DEPENDS
		${PROJECT_PYTHON_DEPS}
		${BOARDS}
		${BOOTLOADER_BOARDS}
	COMMAND ${PROJECT_PYTHON} ${CMAKE_SOURCE_DIR}/scripts/flash_monitor.py flash
	USES_TERMINAL
)

add_custom_target(
    monitor
    DEPENDS
		${PROJECT_PYTHON_DEPS}
		${BOARDS}
		${BOOTLOADER_BOARDS}
	COMMAND ${PROJECT_PYTHON} ${CMAKE_SOURCE_DIR}/scripts/flash_monitor.py monitor
	USES_TERMINAL
)
