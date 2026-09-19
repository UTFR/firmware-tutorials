#ifndef LECTURE2_CONCURRENCY_TORQUE_CONTROL_H
#define LECTURE2_CONCURRENCY_TORQUE_CONTROL_H

// Shared by main.c and the "active" demo (src/race_condition_real_life.c).
// The other variants (src/mutex.c, src/queue.c, src/bare_metal_*.c) are
// self-contained on purpose, the same way the original lecture handouts were
// -- swap one of them into controllers/lecture2_concurrency/CMakeLists.txt's
// LECTURE2_CONCURRENCY_SRC in place of race_condition_real_life.c to try it.

typedef struct {
  double torque;
  int dir;
} torque_cmd_t;

#define CONTROL_LOOP_PERIOD_MS 100

void initialize_control_loop(void);
void motor_control_thread(void);
void torque_calculator_thread(void);
void spin_motor(torque_cmd_t torque_command);

#endif
