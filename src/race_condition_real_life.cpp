#include <Arduino.h>

typedef struct {
  double torque;
  int dir;
} torque_cmd_t;

extern void spin_motor(torque_cmd_t torque_command);

#define NUM_ITERS 1000
#define CONTROL_LOOP_PERIOD_MS 100

static torque_cmd_t torque_cmd;

void motor_control_thread(void) {
  for(;;) {
    delay(CONTROL_LOOP_PERIOD_MS);

    spin_motor(torque_cmd);
  }
}

void torque_calculator_thread(void) {
  for(;;) {
    delay(CONTROL_LOOP_PERIOD_MS);

    // random torque beteen 0 and 230.0 Nm
    torque_cmd.torque = random(0, 2300) * 0.1;
    // random direction (forward or reverse)
    torque_cmd.dir = random(0, 1);
  }
}
