// Re-realization of the original Teensy/Arduino "race_condition_real_life.cpp"
// handout: the same torque_cmd_t shared between a motor-control flow and a
// torque-calculator flow, with no synchronization at all. On the old Teensy
// build this was two cooperative "threads" that never actually ran (nothing
// in platformio.ini ever compiled lecture2-concurrency/). Here it is real:
// motor_control_thread() runs in main()'s bare loop, torque_calculator_thread()
// runs from a TIM6 interrupt every CONTROL_LOOP_PERIOD_MS (see src/main.c) --
// a genuine mainline-vs-ISR race on a multi-field, non-atomic struct.
#include "torque_control.h"
#include "UTFR_DIGITAL/pin_driver.h"
#include "main.h"

#include <stdlib.h>

static torque_cmd_t torque_cmd;

void motor_control_thread(void) { spin_motor(torque_cmd); }

void torque_calculator_thread(void) {
  // random torque between 0 and 230.0 Nm
  torque_cmd.torque = (rand() % 2300) * 0.1;
  // random direction (forward or reverse)
  torque_cmd.dir = rand() % 2;
}

// Standing in for "command the inverter": there's no motor on this board, so
// toggle a GPIO once per command instead -- still a real, observable HAL call.
void spin_motor(torque_cmd_t torque_command) {
  (void)torque_command;
  digital_pin_toggle(PIN_STATUS_LED);
}
