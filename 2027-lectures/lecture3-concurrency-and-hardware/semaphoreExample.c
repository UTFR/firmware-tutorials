#include <FreeRTOS.h>
#include <semphr.h>

extern void prepare_a(void);
extern void prepare_b(void);

static SemaphoreHandle_t a_ready;
static SemaphoreHandle_t b_ready;

void task_a(void *arg) {
  for(;;) {
    prepare_a();
    xSemaphoreGive(a_ready);
    xSemaphoreTake(b_ready, portMAX_DELAY);
    // both A and B are at this point now
  }
}
void task_b(void *arg) { // mirror image
  for(;;) {
    prepare_b();
    xSemaphoreGive(b_ready);
    xSemaphoreTake(a_ready, portMAX_DELAY);
  }
}
