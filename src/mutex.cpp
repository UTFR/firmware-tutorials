#include "portable/portmacro.h"
#include <arduino_freertos.h>
#include <semphr.h>
#include <queue.h>

#define NUM_ITERS 1000
#define CONTROL_LOOP_PERIOD_MS 100

typedef struct {
  double torque;
  int dir;
} torque_cmd_t;

extern void spin_motor(torque_cmd_t torque_command);

static torque_cmd_t torque_cmd;
static SemaphoreHandle_t torque_cmd_mutex;

void initialize_control_loop(void) {
  torque_cmd_mutex = xSemaphoreCreateMutex();
}

void motor_control_thread(void) {
  for(;;) {
    xSemaphoreTake(torque_cmd_mutex, portMAX_DELAY);
    spin_motor(torque_cmd);
    xSemaphoreGive(torque_cmd_mutex);
  }
}

void torque_calculator_thread(void) {
  for(;;) {
    xSemaphoreTake(torque_cmd_mutex, portMAX_DELAY);

    torque_cmd.torque = random(0, 2300) * 0.1;
    torque_cmd.dir = random(0, 1);

    // Give up ownership over torque command
    xSemaphoreGive(torque_cmd_mutex);
  }
}

