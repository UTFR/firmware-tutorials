#ifndef UTFR_BOOT_UTILS_MEMORY_MAP_H
#define UTFR_BOOT_UTILS_MEMORY_MAP_H

#include <assert.h>
#include <stdint.h>
#include "memory_map_config.h"

static_assert(ALIGNED(BOOTLOADER_SECTION_START, PAGE_SIZE), "bootloader section not aligned");
static_assert(ALIGNED(PARAMS_SECTION_START, PAGE_SIZE), "params section not aligned");
static_assert(ALIGNED(APP_SECTION_START, PAGE_SIZE), "app section not aligned");

extern void __libc_init_array(void); // NOLINT(bugprone-reserved-identifier)

extern void _estack(void);           // NOLINT(bugprone-reserved-identifier)
extern uint32_t _sidata;             // NOLINT(bugprone-reserved-identifier)
extern uint32_t _sdata;              // NOLINT(bugprone-reserved-identifier)
extern uint32_t _edata;              // NOLINT(bugprone-reserved-identifier)
extern uint32_t _sbss;               // NOLINT(bugprone-reserved-identifier)
extern uint32_t _ebss;               // NOLINT(bugprone-reserved-identifier)

#endif
