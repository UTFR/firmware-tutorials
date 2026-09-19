// Direct C port of the original "mutex.cpp" handout: same structure, same
// torque_cmd_t, same xSemaphoreCreateMutex()/Take()/Give() calls -- only the
// include swapped from the Teensy <arduino_freertos.h> wrapper to the real
// FreeRTOS headers (this board links against the same vendored FreeRTOS
// kernel the real firmware repo uses), and random()/delay() swapped for
// rand()/vTaskDelay() since there's no Arduino runtime here. This is the
// "active" demo linked into LECTURE3_FREERTOS -- see src/queue.c for the
// alternate (queue-based) take on the same problem.
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "lecture3_tasks.h"
#include "UTFR_DIGITAL/pin_driver.h"
#include "main.h"

#include <stdlib.h>

#define CONTROL_LOOP_PERIOD_MS 100

typedef struct {
  double torque;
  int dir;
} torque_cmd_t;

static void spin_motor(torque_cmd_t torque_command);

static torque_cmd_t torque_cmd;
static SemaphoreHandle_t torque_cmd_mutex;

void initialize_control_loop(void) { torque_cmd_mutex = xSemaphoreCreateMutex(); }

void motor_control_thread(void *pvParameters) {
  (void)pvParameters;
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(CONTROL_LOOP_PERIOD_MS));

    // Take ownership over torque command
    xSemaphoreTake(torque_cmd_mutex, portMAX_DELAY);

    spin_motor(torque_cmd);

    // Give up ownership over torque command
    xSemaphoreGive(torque_cmd_mutex);
  }
}

void torque_calculator_thread(void *pvParameters) {
  (void)pvParameters;
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(CONTROL_LOOP_PERIOD_MS));

    // Take ownership over torque command
    xSemaphoreTake(torque_cmd_mutex, portMAX_DELAY);

    torque_cmd.torque = (rand() % 2300) * 0.1;
    torque_cmd.dir = rand() % 2;

    // Give up ownership over torque command
    xSemaphoreGive(torque_cmd_mutex);
  }
}

static void spin_motor(torque_cmd_t torque_command) {
  (void)torque_command;
  digital_pin_toggle(PIN_STATUS_LED);
}
