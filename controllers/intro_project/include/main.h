#ifndef INTRO_PROJECT_MAIN_H
#define INTRO_PROJECT_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include "UTFR_CAN/can.h"

  void error_handler(void);
  void app_main(void);

  extern can_t vcan;

  typedef enum {
    // LCD + BMS share SPI1 (SCK=PA5, MISO=PA6, MOSI=PA7) -- these are the two
    // chip-selects that arbitrate it.
    PIN_LCD_CS, // PB0
    PIN_BMS_CS, // PB1

    // Dash buttons, active low.
    PIN_TS_ON_BUTTON, // PB4
    PIN_RTD_BUTTON,   // PB5

    // Accumulator Isolation Relays + precharge relay.
    PIN_AIR_PLUS,  // PB6
    PIN_AIR_MINUS, // PB7
    PIN_PRECHARGE, // PB8

    // Wheelspeed tooth sensors (17 teeth/rev, active low, EXTI9_5).
    PIN_WHEELSPEED_FL, // PC6
    PIN_WHEELSPEED_FR, // PC7
    PIN_WHEELSPEED_RL, // PC8
    PIN_WHEELSPEED_RR, // PC9

    PIN_COUNT_,
  } digital_pin_t;

#ifdef __cplusplus
}
#endif

#endif
