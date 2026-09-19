#ifndef UTFR_UTILS_CYCCNT_H
#define UTFR_UTILS_CYCCNT_H

// Cycle counter

#include <stdbool.h>
#include <stdint.h>

#include "utfr_hal.h"

// Enable DWT->CYCCNT. Idempotent.
static inline void cyccnt_init(void) {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
#if defined(UTFR_MCU_FAMILY_H7)
  // Cortex-M7 gates DWT register writes behind the Lock Access Register
  DWT->LAR = 0xC5ACCE55UL;
#endif
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

// Free-running 32-bit CPU-cycle count.
static inline uint32_t cyccnt_read(void) { return DWT->CYCCNT; }

// True once cyccnt_init() has actually enabled the counter. Lets a caller warn
static inline bool cyccnt_running(void) {
  return (DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U;
}

#endif // UTFR_UTILS_CYCCNT_H
