// Direct C port of the original "foo_task.cpp" handout -- only the include
// swapped from the Teensy <arduino_freertos.h> wrapper to the real FreeRTOS
// headers. Everything else, including the (deliberately silly) two identical
// tasks, is unchanged.
#include "FreeRTOS.h"
#include "task.h"
#include "foo_task.h"

static void foo_task(void *p);

#define FOO_STACK_DEPTH 128
#define FOO_PRIORITY    (tskIDLE_PRIORITY + 1)

TaskHandle_t foo;

void create_foo_task(void) {
  xTaskCreate(foo_task, "foo", FOO_STACK_DEPTH, NULL, FOO_PRIORITY, &foo);
  xTaskCreate(foo_task, "foo", FOO_STACK_DEPTH, NULL, FOO_PRIORITY, NULL);
}

static void foo_task(void *p) {
  (void)p;
  for (;;) {}
}
