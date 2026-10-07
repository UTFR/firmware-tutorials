#include "portable/portmacro.h"
#include <arduino_freertos.h>
#include <timers.h>
#include <climits>

void timer_callback(TimerHandle_t timer);

TaskHandle_t bar_task_handle;

void create_timer(void) {
  TimerHandle_t foo_timer =
    xTimerCreate("foo", 10, pdTRUE, NULL, timer_callback);
  xTimerStart(foo_timer, 0);
}

void timer_callback(TimerHandle_t timer) { xTaskNotifyGive(bar_task_handle); }

void bar_task(void *) {
  for(;;) {
    xTaskNotifyWait(ULONG_MAX, ULONG_MAX, NULL, portMAX_DELAY);

    // run some 10ms periodic code here
  }
}
