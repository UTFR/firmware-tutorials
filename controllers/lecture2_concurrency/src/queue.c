// Direct port of the original "queue.cpp" handout, now backed by a real
// (if minimal) queue -- see src/bare_metal_queue.c. Not linked into the
// default LECTURE2_CONCURRENCY executable (symbol clash with
// race_condition_real_life.c); built as a standalone object library to prove
// it compiles against the real headers.
//
// The original handout left a note here: "I now have a question about the
// call to `delay`. Remind me if i forget to ask it" -- kept for continuity,
// though this port has no delay() call left to ask about (see main.c for
// where the periodicity now comes from).
#include "bare_metal_queue.h"
#include "torque_control.h"
#include "UTFR_DIGITAL/pin_driver.h"
#include "main.h"

#include <stdlib.h>

static queue_t torque_cmd_queue;

void initialize_control_loop(void) { torque_cmd_queue = queue_create(128, sizeof(torque_cmd_t)); }

void motor_control_thread(void) {
  torque_cmd_t cmd;

  while (!queue_receive(torque_cmd_queue, &cmd)) {}

  spin_motor(cmd);
}

void torque_calculator_thread(void) {
  torque_cmd_t cmd;

  cmd.torque = (rand() % 2300) * 0.1;
  cmd.dir = rand() % 2;

  while (!queue_send(torque_cmd_queue, &cmd)) {}
}

void spin_motor(torque_cmd_t torque_command) {
  (void)torque_command;
  digital_pin_toggle(PIN_STATUS_LED);
}
