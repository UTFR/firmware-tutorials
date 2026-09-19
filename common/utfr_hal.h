#ifndef UTFR_HAL_H
#define UTFR_HAL_H

// Family-neutral HAL umbrella.

#if defined(UTFR_MCU_FAMILY_H7) || defined(STM32H723xx) || defined(STM32H7)
  #include "stm32h7xx_hal.h"
#elif defined(UTFR_MCU_FAMILY_G4) || defined(STM32G474xx) || defined(STM32G4)
  #include "stm32g4xx_hal.h"
#else
  #error "utfr_hal.h: no MCU family defined (expected UTFR_MCU_FAMILY_G4 or UTFR_MCU_FAMILY_H7)"
#endif

#endif // UTFR_HAL_H
