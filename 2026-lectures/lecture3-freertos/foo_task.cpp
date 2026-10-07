#include <arduino_freertos.h>
#include "foo_task.h"

static void foo_task(void *p);

#define FOO_STACK_DEPTH 1024
#define FOO_PRIORITY tskIDLE_PRIORITY + 1

TaskHandle_t foo;

void create_foo_task(void) {
  xTaskCreate(foo_task, "foo", FOO_STACK_DEPTH, NULL, FOO_PRIORITY, &foo);
  xTaskCreate(foo_task, "foo", FOO_STACK_DEPTH, NULL, FOO_PRIORITY, NULL);
}

static void foo_task(void *p) {
  for(;;) {}
}
