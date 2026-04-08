#include "portable/portmacro.h"
#include <arduino_freertos.h>
#include <semphr.h>
#include <queue.h>
#include <stdbool.h>

typedef struct {
  double torque;
  int dir;
} torque_cmd_t;

extern void spin_motor(torque_cmd_t torque_command);

#define NUM_ITERS 1000
#define CONTROL_LOOP_PERIOD_MS 100

static QueueHandle_t torque_cmd_queue;

void initialize_control_loop(void) {
  // torque_cmd_queue = queue_create(128, sizeof(torque_cmd_t));
  torque_cmd_queue = xQueueCreate(128, sizeof(torque_cmd_t));
}

void motor_control_thread(void) {
  torque_cmd_t cmd;

  for(;;) {
    xQueueReceive(torque_cmd_queue, &cmd, portMAX_DELAY);
    spin_motor(cmd);
  }
}

void torque_calculator_thread(void) {
  torque_cmd_t cmd;

  for(;;) {
    delay(CONTROL_LOOP_PERIOD_MS);

    cmd.torque = random(0, 2300) * 0.1;
    cmd.dir = random(0, 1);

    xQueueSend(torque_cmd_queue, &cmd, 0);
  }
}
