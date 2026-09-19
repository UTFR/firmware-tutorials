// Direct C port of the original "timer_notification.cpp" handout: same
// xTimerCreate()/xTaskNotifyGive()/xTaskNotifyWait() calls, headers swapped
// from the Teensy wrapper to the real FreeRTOS headers, <climits> swapped for
// <limits.h> (this is C, not C++).
//
// The original handout never actually created bar_task (create_timer() only
// armed the timer; nothing ever called xTaskCreate(bar_task, ...)), so under
// the old Teensy build this code was never truly exercised end to end.
// create_bar_task() below is the small addition that makes it real: it's
// called from main.c alongside create_timer().
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "lecture3_tasks.h"

#include <limits.h>

#define BAR_STACK_DEPTH 128
#define BAR_PRIORITY    (tskIDLE_PRIORITY + 1)

static void timer_callback(TimerHandle_t timer);
static void bar_task(void *p);

static TaskHandle_t bar_task_handle;

void create_timer(void) {
  TimerHandle_t foo_timer = xTimerCreate("foo", pdMS_TO_TICKS(10), pdTRUE, NULL, timer_callback);
  xTimerStart(foo_timer, 0);
}

void create_bar_task(void) {
  xTaskCreate(bar_task, "bar", BAR_STACK_DEPTH, NULL, BAR_PRIORITY, &bar_task_handle);
}

static void timer_callback(TimerHandle_t timer) {
  (void)timer;
  xTaskNotifyGive(bar_task_handle);
}

static void bar_task(void *p) {
  (void)p;
  for (;;) {
    xTaskNotifyWait(ULONG_MAX, ULONG_MAX, NULL, portMAX_DELAY);

    // run some 10ms periodic code here
  }
}
