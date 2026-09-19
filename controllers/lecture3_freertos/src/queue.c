// Direct C port of the original "queue.cpp" handout, same treatment as
// src/mutex.c. Not linked into the default LECTURE3_FREERTOS executable (it
// would symbol-clash with mutex.c's motor_control_thread/
// torque_calculator_thread); compiled as a standalone object library to
// prove it builds against the real FreeRTOS headers. Swap it into
// LECTURE3_FREERTOS_SRC in CMakeLists.txt (in place of mutex.c) to try it.
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "UTFR_DIGITAL/pin_driver.h"
#include "main.h"

#include <stdbool.h>
#include <stdlib.h>

#define CONTROL_LOOP_PERIOD_MS 100

typedef struct {
  double torque;
  int dir;
} torque_cmd_t;

static void spin_motor(torque_cmd_t torque_command);

static QueueHandle_t torque_cmd_queue;

void initialize_control_loop(void) { torque_cmd_queue = xQueueCreate(128, sizeof(torque_cmd_t)); }

void motor_control_thread(void *pvParameters) {
  (void)pvParameters;
  torque_cmd_t cmd;

  for (;;) {
    xQueueReceive(torque_cmd_queue, &cmd, portMAX_DELAY);
    spin_motor(cmd);
  }
}

void torque_calculator_thread(void *pvParameters) {
  (void)pvParameters;
  torque_cmd_t cmd;

  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(CONTROL_LOOP_PERIOD_MS));

    cmd.torque = (rand() % 2300) * 0.1;
    cmd.dir = rand() % 2;

    xQueueSend(torque_cmd_queue, &cmd, 0);
  }
}

static void spin_motor(torque_cmd_t torque_command) {
  (void)torque_command;
  digital_pin_toggle(PIN_STATUS_LED);
}
