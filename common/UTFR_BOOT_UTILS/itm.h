#ifndef UTFR_BOOT_UTILS_ITM_H
#define UTFR_BOOT_UTILS_ITM_H

#include "utfr_hal.h"
#include <stdbool.h>

#define ITM_TIMEOUT_CYCLES 1000

static inline bool debugger_attached(void) {
  return (CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) != 0;
}

static inline bool debugger_enabled(void) {
  return (CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) != 0;
}

// Global enable for all DWT and ITM features
static inline void debugger_enable(void) { CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; }

/*
From Armv7-M Architecture Reference Manual
C1-709: ITM_TCR.ITMENA is a global enable bit for the ITM. A power-on reset clears this bit to 0, disabling the ITM.
*/
static inline bool itm_enabled(void) { return (ITM->TCR & ITM_TCR_ITMENA_Msk) != 0; }

/*
From Armv7-M Architecture Reference Manual
C1-709: The ITM_TERs provide an enable bit for each stimulus port.
*/
static inline bool itm_stimulus_port_enabled(void) { return (ITM->TER & 1U) != 0; }

static inline void itm_write(const char *buf, int len) {
  if (!debugger_enabled()) return;
  if (!itm_enabled()) return;
  if (!itm_stimulus_port_enabled()) return;

  for (int i = 0; i < len; i++) {
    uint32_t timeout = ITM_TIMEOUT_CYCLES;
    while (ITM->PORT[0].u32 == 0) {
      if (--timeout == 0) return; // timeout (SWO prolly not connected)
    }
    ITM->PORT[0].u8 = (uint8_t)buf[i];
  }
}

#endif
