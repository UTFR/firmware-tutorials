#ifndef LECTURE3_FREERTOS_TASKS_H
#define LECTURE3_FREERTOS_TASKS_H

// Shared by main.c and the "active" demo (src/mutex.c). src/queue.c is a
// self-contained reference variant, same idiom as lecture2_concurrency.
void initialize_control_loop(void);
void motor_control_thread(void *pvParameters);
void torque_calculator_thread(void *pvParameters);

// src/timer_notification.c
void create_timer(void);
void create_bar_task(void);

#endif
