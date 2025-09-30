#include <Arduino.h>
#include <stdbool.h>

typedef struct {
  double torque;
  int dir;
} torque_cmd_t;

typedef int mutex_t;

extern mutex_t mutex_create(void);

// What would happen without volatile?
extern volatile bool mutex_take(mutex_t mutex);
extern volatile bool mutex_give(mutex_t mutex);
extern void spin_motor(torque_cmd_t torque_command);

#define NUM_ITERS 1000
#define CONTROL_LOOP_PERIOD_MS 100

static torque_cmd_t torque_cmd;
static mutex_t torque_cmd_mutex;

void initialize_control_loop(void) { torque_cmd_mutex = mutex_create(); }

void motor_control_thread(void) {
  for(;;) {
    delay(CONTROL_LOOP_PERIOD_MS);

    // Take ownership over torque command
    while(!mutex_take(torque_cmd_mutex)) {}

    spin_motor(torque_cmd);

    // Give up ownership over torque command
    mutex_give(torque_cmd_mutex);
  }
}

void torque_calculator_thread(void) {
  for(;;) {
    delay(CONTROL_LOOP_PERIOD_MS);

    // Take ownership over torque command
    while(!mutex_take(torque_cmd_mutex)) {}

    torque_cmd.torque = random(0, 2300) * 0.1;
    torque_cmd.dir = random(0, 1);

    // Give up ownership over torque command
    mutex_give(torque_cmd_mutex);
  }
}
