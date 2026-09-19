INCLUDE memory_map_preprocessed.ld

SECTIONS {
    .image_header : {
        KEEP(*(.image_header))
    } >APP

    . = ALIGN(512);

    .vector_table : {
        . = ALIGN(512);
        KEEP(*(.vector_table))
        . = ALIGN(4);
    } >APP

    .text : {
        . = ALIGN(4);
        *(.text)           /* .text sections (code) */
        *(.text*)          /* .text* sections (code) */
        *(.glue_7)         /* glue arm to thumb code */
        *(.glue_7t)        /* glue thumb to arm code */
        *(.eh_frame)       /* exception frames */
        KEEP (*(.init))    /* c++ constructor stuff */
        KEEP (*(.fini))    /* c++ destructor stuff */
        . = ALIGN(4);
        _etext = .;
    } >APP

    .rodata : {
        . = ALIGN(4);
        *(.rodata)
        *(.rodata*)
        . = ALIGN(4);
    } >APP

    .ARM.extab : {
        . = ALIGN(4);
        *(.ARM.extab* .gnu.linkonce.armextab.*)
        . = ALIGN(4);
    } >APP

    .ARM : {
        . = ALIGN(4);
        __exidx_start = .;
        *(.ARM.exidx*)
        __exidx_end = .;
        . = ALIGN(4);
    } >APP

    .preinit_array : {
        . = ALIGN(4);
        PROVIDE_HIDDEN (__preinit_array_start = .);
        KEEP (*(.preinit_array*))
        PROVIDE_HIDDEN (__preinit_array_end = .);
        . = ALIGN(4);
    } >APP

    .init_array : {
        . = ALIGN(4);
        PROVIDE_HIDDEN (__init_array_start = .);
        KEEP (*(SORT(.init_array.*)))
        KEEP (*(.init_array*))
        PROVIDE_HIDDEN (__init_array_end = .);
        . = ALIGN(4);
    } >APP

    .fini_array : {
        . = ALIGN(4);
        PROVIDE_HIDDEN (__fini_array_start = .);
        KEEP (*(SORT(.fini_array.*)))
        KEEP (*(.fini_array*))
        PROVIDE_HIDDEN (__fini_array_end = .);
        . = ALIGN(4);
    } >APP

    /* expose address to copy from data segment to sram in reset_handler */
    _sidata = LOADADDR(.data);

    .data : {
        . = ALIGN(4);
        _sdata = .;
        *(.data)
        *(.data*)
        *(.RamFunc)
        *(.RamFunc*)

        . = ALIGN(4);
        _edata = .;
    } >RAM AT> APP /* VMA in RAM but LMA in APP */

    . = ALIGN(4);
    .bss : {
        /* expose address to zero initialize bss segment */
        _sbss = .;
        __bss_start__ = _sbss; /* TODO: do not need this */
        *(.bss)
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        /* expose address to zero initialize bss segment */
        _ebss = .;
        __bss_end__ = _ebss; /* TODO: do not need this */
    } >RAM

    /* used to check that there is enough RAM left */
    ._user_heap_stack : {
        . = ALIGN(8);
        PROVIDE ( end = . );
        PROVIDE ( _end = . );
        . = . + _min_heap_size;
        . = . + _min_stack_size;
        . = ALIGN(8);
    } >RAM

    /* remove information from the standard libraries */
    /DISCARD/ : {
        libc.a ( * )
        libm.a ( * )
        libgcc.a ( * )
    }
}
