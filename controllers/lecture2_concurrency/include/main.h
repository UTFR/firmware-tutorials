#ifndef LECTURE2_CONCURRENCY_MAIN_H
#define LECTURE2_CONCURRENCY_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"

  void error_handler(void);

  typedef enum {
    PIN_STATUS_LED, // PC13, toggled once per torque_calculator_thread() tick
    PIN_COUNT_,
  } digital_pin_t;

#ifdef __cplusplus
}
#endif

#endif
