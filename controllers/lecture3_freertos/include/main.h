#ifndef LECTURE3_FREERTOS_MAIN_H
#define LECTURE3_FREERTOS_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"

  void error_handler(void);

  typedef enum {
    PIN_STATUS_LED, // PC13, toggled by spin_motor()
    PIN_COUNT_,
  } digital_pin_t;

#ifdef __cplusplus
}
#endif

#endif
