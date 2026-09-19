# A board names a chip; cmake/mcu/<chip>.cmake says what that chip is. Nothing
# here names a family.
#
# HAL_MODULES-<board> drives both the module #define in the generated conf
# header and which HAL sources compile, so the two cannot disagree. Modules are
# logical, not files: one name may pull a base and an _ex half sharing a single
# #ifdef guard. The name also survives a change of family; a filename does not.
set(HAL_KNOWN_MODULES
    adc bdma comp cordic cortex crc cryp dac dma dts eth exti fdcan flash fmac
    gpio hrtim hsem i2c i2s irda iwdg lptim mdma nand nor octospi opamp pcd pwr
    qspi ramecc rcc rng rtc sai sd sdram sdmmc smartcard smbus spi sram tim uart
    usart wwdg
)

# Reads cmake/mcu/<mcu>.cmake once per chip. Results go in global properties
# because the manifest loop runs at root scope.
function(utfr_load_mcu mcu)
    get_property(loaded GLOBAL PROPERTY UTFR_MCU_LOADED-${mcu})
    if(loaded)
        return()
    endif()

    set(descriptor ${CMAKE_SOURCE_DIR}/cmake/mcu/${mcu}.cmake)
    if(NOT EXISTS ${descriptor})
        message(FATAL_ERROR "No descriptor for MCU '${mcu}'. Add cmake/mcu/${mcu}.cmake.")
    endif()
    include(${descriptor})

    foreach(f ARCH FAMILY HAL_DIR HAL_CONF_IN CMSIS_DEVICE DEFINES
              FLASH_SIZE RAM_SIZE RAM_START FLASH_PAGE_SIZE OPENOCD_TARGET)
        if(NOT DEFINED MCU_${f})
            message(FATAL_ERROR "${descriptor} does not set MCU_${f}.")
        endif()
        set_property(GLOBAL PROPERTY UTFR_MCU_${f}-${mcu} "${MCU_${f}}")
    endforeach()

    set_property(GLOBAL PROPERTY UTFR_MCU_LOADED-${mcu} TRUE)
endfunction()

# Read one descriptor field back.
macro(utfr_mcu_get mcu field out)
    get_property(${out} GLOBAL PROPERTY UTFR_MCU_${field}-${mcu})
endmacro()

function(utfr_hal_configure board out_sources)
    set(modules ${HAL_MODULES-${board}})
    if(NOT modules)
        message(FATAL_ERROR "HAL: no HAL_MODULES-${board} declared.")
    endif()

    set(mcu ${BOARD_MCU-${board}})
    utfr_mcu_get(${mcu} FAMILY      family)
    utfr_mcu_get(${mcu} HAL_DIR     hal_dir)
    utfr_mcu_get(${mcu} HAL_CONF_IN conf_in)

    set(src_dir ${hal_dir}/src)
    set(defines "")
    set(sources ${src_dir}/${family}xx_hal.c)

    foreach(m ${modules})
        if(NOT m IN_LIST HAL_KNOWN_MODULES)
            message(FATAL_ERROR "HAL: unknown module '${m}' in HAL_MODULES-${board}.")
        endif()

        string(TOUPPER ${m} M)
        string(APPEND defines "#define HAL_${M}_MODULE_ENABLED\n")

        list(APPEND sources ${src_dir}/${family}xx_hal_${m}.c)
        if(EXISTS ${src_dir}/${family}xx_hal_${m}_ex.c)
            list(APPEND sources ${src_dir}/${family}xx_hal_${m}_ex.c)
        endif()
    endforeach()

    # RAM-resident erase/program, needed to rewrite flash while running from it.
    if(flash IN_LIST modules AND EXISTS ${src_dir}/${family}xx_hal_flash_ramfunc.c)
        list(APPEND sources ${src_dir}/${family}xx_hal_flash_ramfunc.c)
    endif()

    set(UTFR_HAL_MODULE_DEFINES "${defines}")
    set(conf_dir ${CMAKE_BINARY_DIR}/generated/hal/${board})
    configure_file(${conf_in} ${conf_dir}/${family}xx_hal_conf.h @ONLY)

    set(HAL_CONF_DIR-${board} ${conf_dir} PARENT_SCOPE)
    set(${out_sources} ${sources} PARENT_SCOPE)
endfunction()

function(compile_hal board_name)
    utfr_hal_configure(${board_name} board_hal_sources)

    set(mcu ${BOARD_MCU-${board_name}})
    utfr_mcu_get(${mcu} HAL_DIR      hal_dir)
    utfr_mcu_get(${mcu} CMSIS_DEVICE cmsis_device)
    utfr_mcu_get(${mcu} DEFINES      mcu_defines)
    utfr_mcu_get(${mcu} ARCH         arch)
    utfr_mcu_get(${mcu} FLASH_SIZE      mcu_flash_size)
    utfr_mcu_get(${mcu} RAM_SIZE        mcu_ram_size)
    utfr_mcu_get(${mcu} RAM_START       mcu_ram_start)
    utfr_mcu_get(${mcu} FLASH_PAGE_SIZE mcu_page_size)
    get_property(ld_dir GLOBAL PROPERTY UTFR_LD_DIR-${mcu})

    add_library(stm32_hal_interface-${board_name} INTERFACE)
    target_include_directories(stm32_hal_interface-${board_name} INTERFACE
        ${hal_dir}/include
        ${hal_dir}/include/legacy
        ${cmsis_device}
        ${CMAKE_SOURCE_DIR}/common/CMSIS/include
        ${CMAKE_SOURCE_DIR}/common        # for utfr_hal.h, the family-neutral HAL umbrella
        ${HAL_CONF_DIR-${board_name}}
        ${HAL_CONFIG_DIR-${board_name}}
    )
    target_link_libraries(stm32_hal_interface-${board_name} INTERFACE utfr_arch_${arch})
    target_link_options(stm32_hal_interface-${board_name} INTERFACE -L${ld_dir})
    target_compile_definitions(stm32_hal_interface-${board_name} INTERFACE
        ${mcu_defines} $<$<CONFIG:Debug>:DEBUG>
        UTFR_MCU_FLASH_SIZE=${mcu_flash_size}
        RAM_SIZE=${mcu_ram_size}
        RAM_START=${mcu_ram_start}
        PAGE_SIZE=${mcu_page_size})

    add_library(stm32_hal-${board_name} OBJECT)
    target_sources(stm32_hal-${board_name} PRIVATE ${board_hal_sources})
    target_link_libraries(stm32_hal-${board_name} PUBLIC stm32_hal_interface-${board_name})
    add_dependencies(stm32_hal-${board_name} linker_script_preprocess-${mcu})
endfunction()
