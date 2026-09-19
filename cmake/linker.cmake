# One map per chip: sizes and erase granularity differ, and a board linking the
# wrong one fails silently rather than loudly.

list(REMOVE_DUPLICATES ALL_MCUS)
set(MEMORY_MAP_IN ${CMAKE_CURRENT_SOURCE_DIR}/memory_map.ld)
set(all_maps "")

foreach(mcu ${ALL_MCUS})
    utfr_mcu_get(${mcu} FLASH_SIZE      map_flash)
    utfr_mcu_get(${mcu} RAM_SIZE        map_ram)
    utfr_mcu_get(${mcu} RAM_START       map_ram_start)
    utfr_mcu_get(${mcu} FLASH_PAGE_SIZE map_page)

    set(map_dir ${CMAKE_BINARY_DIR}/generated/ld/${mcu})
    set(map_out ${map_dir}/memory_map_preprocessed.ld)

    add_custom_command(
        OUTPUT  ${map_out}
        COMMAND ${CMAKE_C_COMPILER}
                -E -P -x c
                -DFLASH_SIZE=${map_flash}
                -DRAM_SIZE=${map_ram}
                -DRAM_START=${map_ram_start}
                -DPAGE_SIZE=${map_page}
                -I${COMMON_DIR}
                ${MEMORY_MAP_IN}
                -o ${map_out}
        DEPENDS ${MEMORY_MAP_IN} ${COMMON_DIR}/UTFR_BOOT_UTILS/memory_map_config.h
        COMMENT "Preprocessing linker script for ${mcu}"
    )

    add_custom_target(linker_script_preprocess-${mcu} DEPENDS ${map_out})
    set_property(GLOBAL PROPERTY UTFR_LD_DIR-${mcu} ${map_dir})
    list(APPEND all_maps ${map_out})
endforeach()

# Aggregate, so a board can depend on the map without naming its chip.
add_custom_target(linker_script_preprocess DEPENDS ${all_maps})

# UTFR_PARAMS is built once rather than per board, so it takes the first map.
# That is only meaningful while every board shares a chip; make it per board
# before that stops being true.
list(GET all_maps 0 MEMORY_MAP_OUT)
