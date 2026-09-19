#ifndef UTFR_BOOT_UTILS_MEMORY_MAP_CONFIG_H
#define UTFR_BOOT_UTILS_MEMORY_MAP_CONFIG_H

// #define str(s) #s
#define _STRINGIFY(x) #x
#define STRINGIFY(x)  _STRINGIFY(x)

#ifndef PAGE_SIZE
  #define PAGE_SIZE 2048
#endif
#define ALIGNED(ADDR, BOUNDARY) (ADDR % BOUNDARY == 0)

// ------- RAM -------

#ifndef RAM_SIZE
  #define RAM_SIZE (128 * 1024)
#endif

#ifndef RAM_START
  #define RAM_START 0x20000000
#endif

#define SHARED_RAM_SECTION_START    RAM_START
#define SHARED_RAM_SECTION_SIZE     256
#define SHARED_RAM_SECTION_NAME     .shared_ram
#define SHARED_RAM_SECTION_NAME_STR STRINGIFY(SHARED_RAM_SECTION_NAME)

#define RAM_SECTION_START (SHARED_RAM_SECTION_START + SHARED_RAM_SECTION_SIZE)

// ------ FLASH ------

#ifndef FLASH_SIZE
  // Not UTFR_MCU_FLASH_SIZE's own compile-definition name: STM32H7's CMSIS
  // device header (stm32h755xx.h etc.) also #defines a bare FLASH_SIZE (a
  // runtime flash-size-detection expression, unrelated to this repo's
  // board-region concept), so any TU that includes both would silently
  // clobber whichever one wins by include order. UTFR_MCU_FLASH_SIZE (set
  // per-MCU in cmake/hal.cmake) can't collide with a vendor header's name.
  #ifdef UTFR_MCU_FLASH_SIZE
    #define FLASH_SIZE UTFR_MCU_FLASH_SIZE
  #else
    #define FLASH_SIZE (512 * 1024)
  #endif
#endif

#define BOOTLOADER_SECTION_START 0x08000000
#define BOOTLOADER_SECTION_SIZE  (128 * 1024)

#define PARAMS_SECTION_START    (BOOTLOADER_SECTION_START + BOOTLOADER_SECTION_SIZE)
#define PARAMS_SECTION_SIZE     PAGE_SIZE
#define PARAMS_SECTION_NAME     .params
#define PARAMS_SECTION_NAME_STR STRINGIFY(PARAMS_SECTION_NAME)

#define APP_SECTION_START (PARAMS_SECTION_START + PARAMS_SECTION_SIZE)

// DMA buffer placement
#ifndef __ASSEMBLER__
  #if defined(UTFR_MCU_FAMILY_H7)
    #define DMA_BUFFER __attribute__((section(".dma_buffer"), aligned(32)))
  #else
    #define DMA_BUFFER __attribute__((aligned(4)))
  #endif
#endif

#endif
