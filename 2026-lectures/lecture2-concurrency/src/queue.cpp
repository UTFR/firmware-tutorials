#include <Arduino.h>
#include <stdbool.h>

typedef struct {
  double torque;
  int dir;
} torque_cmd_t;

typedef int mutex_t;
typedef int queue_t;

extern mutex_t mutex_create(void);
extern bool mutex_take(mutex_t mutex);
extern bool mutex_give(mutex_t mutex);

extern queue_t queue_create(int num_elements, size_t element_size);
extern bool queue_send(queue_t queue, void *data);
extern bool queue_receive(queue_t queue, void *data);

extern void spin_motor(torque_cmd_t torque_command);

#define NUM_ITERS 1000
#define CONTROL_LOOP_PERIOD_MS 100

static queue_t torque_cmd_queue;

void initialize_control_loop(void) {
  torque_cmd_queue = queue_create(128, sizeof(torque_cmd_t));
}

// I now have a question about the call to `delay`
// Remind me if i forget to ask it

void motor_control_thread(void) {
  torque_cmd_t cmd;

  for(;;) {
    delay(CONTROL_LOOP_PERIOD_MS);

    while(!queue_receive(torque_cmd_queue, &cmd)) {}

    spin_motor(cmd);
  }
}

void torque_calculator_thread(void) {
  torque_cmd_t cmd;

  for(;;) {
    delay(CONTROL_LOOP_PERIOD_MS);

    cmd.torque = random(0, 2300) * 0.1;
    cmd.dir = random(0, 1);

    while(!queue_send(torque_cmd_queue, &cmd)) {}
  }
}
