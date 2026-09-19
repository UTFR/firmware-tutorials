// Direct port of the original "mutex.cpp" handout, now backed by a real
// (if minimal) mutex -- see src/bare_metal_mutex.c. Not linked into the
// default LECTURE2_CONCURRENCY executable (it would symbol-clash with
// race_condition_real_life.c's motor_control_thread/torque_calculator_thread);
// built as a standalone object library to prove it compiles against the real
// headers. Swap it into LECTURE2_CONCURRENCY_SRC in CMakeLists.txt to try it.
#include "bare_metal_mutex.h"
#include "torque_control.h"
#include "UTFR_DIGITAL/pin_driver.h"
#include "main.h"

#include <stdlib.h>

static torque_cmd_t torque_cmd;
static mutex_t torque_cmd_mutex;

void initialize_control_loop(void) { torque_cmd_mutex = mutex_create(); }

void motor_control_thread(void) {
  // Take ownership over torque command
  while (!mutex_take(torque_cmd_mutex)) {}

  spin_motor(torque_cmd);

  // Give up ownership over torque command
  mutex_give(torque_cmd_mutex);
}

void torque_calculator_thread(void) {
  // Take ownership over torque command
  while (!mutex_take(torque_cmd_mutex)) {}

  torque_cmd.torque = (rand() % 2300) * 0.1;
  torque_cmd.dir = rand() % 2;

  // Give up ownership over torque command
  mutex_give(torque_cmd_mutex);
}

void spin_motor(torque_cmd_t torque_command) {
  (void)torque_command;
  digital_pin_toggle(PIN_STATUS_LED);
}
