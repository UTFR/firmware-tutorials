#include <FreeRTOS.h>
#include <task.h>

extern int  button_is_pressed(void);           // reads the GPIO pin
extern void enable_button_interrupt(void (*isr)(void)); // fires isr on press
extern void handle_press(void);


void polling_task(void *arg) {
  for(;;) {
    if (button_is_pressed()) {
      handle_press();
    }
  }
}

static TaskHandle_t button_task_handle;

// Runs in interrupt context: keep it short, no blocking calls,
void button_isr(void) {
  BaseType_t woke_higher_prio = pdFALSE;
  vTaskNotifyGiveFromISR(button_task_handle, &woke_higher_prio);
  portYIELD_FROM_ISR(woke_higher_prio); // switch to the task right away if needed
}

void interrupt_task(void *arg) {
  button_task_handle = xTaskGetCurrentTaskHandle();
  enable_button_interrupt(button_isr);
  for(;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // sleeps (0% CPU) until the ISR fires
    handle_press();
  }
}
