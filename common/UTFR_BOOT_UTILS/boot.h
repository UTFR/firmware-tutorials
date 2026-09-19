#ifndef UTFR_BOOT_UTILS_BOOT_H
#define UTFR_BOOT_UTILS_BOOT_H

#include <stdint.h>

#include "utfr_hal.h"

static inline void vtor_init(uint32_t address) {
  SCB->VTOR = address; // NOLINT(clang-analyzer-core.FixedAddressDereference)

  // make sure change has fully taken effect before proceeding
  // if we jumped to another image before the data/instructions in the pipeline catch up then we
  // could be referencing the wrong vector table
  __DSB();
  __ISB();
}

__attribute__((noreturn)) void reset_handler(void);
void sys_clock_config(void);

#endif
