# Core flags, selected by the chip's declared core and reaching board code
# through the HAL interface. -mcpu is the CPU core, -mfpu the FPU, -mfloat-abi
# the calling convention -- none of them identifies the chip, which the part
# defines and the linker script express instead.
#
# Additive: the globals remain the default for code compiled once for every
# board. Target options come later on the command line, so a board on another
# core overrides rather than conflicts. Anything compiled once cannot serve two
# cores, so such libraries must become per-board first.

function(utfr_declare_arch arch)
    if(TARGET utfr_arch_${arch})
        return()
    endif()
    add_library(utfr_arch_${arch} INTERFACE)
    target_compile_options(utfr_arch_${arch} INTERFACE ${ARGN})
    target_link_options(utfr_arch_${arch} INTERFACE ${ARGN})
endfunction()

utfr_declare_arch(cortex-m4f -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard)
utfr_declare_arch(cortex-m7f -mcpu=cortex-m7 -mfpu=fpv5-d16    -mfloat-abi=hard)
utfr_declare_arch(cortex-m0  -mcpu=cortex-m0 -mfloat-abi=soft)
