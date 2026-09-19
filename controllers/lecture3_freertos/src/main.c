// Provided bring-up for the LECTURE3_FREERTOS board -- the real HAL/boot/
// FreeRTOS boilerplate every RTOS board needs, so the lecture content
// (src/mutex.c, src/queue.c, src/foo_task.c, src/timer_notification.c) can
// stay focused on the FreeRTOS primitives themselves. This is the direct C
// translation of the original Arduino setup()/loop() split -- real FreeRTOS
// firmware just calls vTaskStartScheduler() from main() and never returns.
#include "main.h"
#include "foo_task.h"
#include "lecture3_tasks.h"
#include "UTFR_BOOT_UTILS/boot.h"
#include "UTFR_BOOT_UTILS/image.h"
#include "UTFR_BOOT_UTILS/vector.h"
#include "UTFR_DIGITAL/pin_driver.h"
#include "UTFR_LOGGING/logger.h"
#include "UTFR_UART/uart.h"
#include "git/git.h"
#include "FreeRTOS.h"
#include "task.h"

#define MOTOR_TASK_STACK_DEPTH  256
#define MOTOR_TASK_PRIORITY     (tskIDLE_PRIORITY + 2)
#define TORQUE_TASK_STACK_DEPTH 256
#define TORQUE_TASK_PRIORITY    (tskIDLE_PRIORITY + 2)

static void pins_init(void);

__attribute__((section(".image_header"))) image_header_t app_header
  = {.magic = IMAGE_MAGIC,
     .image_kind = IMAGE_KIND_APP,
     .version_major = 0,
     .version_minor = 1,
     .version_patch = 0,
     .vector_addr = (uint32_t)&vector_table,
     .git_sha = GIT_HASH};

uart_t serial_uart = {.baudrate = UART_BAUDRATE_115200, .rx = USART2_RX_PA3, .tx = USART2_TX_PA2};

digital_config_t pins[] = {
  [PIN_STATUS_LED] = {GPIOC, GPIO_PIN_13, DIGITAL_OUTPUT, DIGITAL_NOPULL, GPIO_PIN_RESET}
};

int main(void) {
  HAL_Init();
  sys_clock_config();

  if (uart_init(&serial_uart) != HAL_OK) { error_handler(); }
  uart_set_io_handle(&serial_uart.handle);
  logger_init(&serial_uart);
  pins_init();

  LOGI("lecture3_freertos: starting tasks");

  initialize_control_loop();
  xTaskCreate(motor_control_thread, "motor_ctl", MOTOR_TASK_STACK_DEPTH, NULL, MOTOR_TASK_PRIORITY,
              NULL);
  xTaskCreate(torque_calculator_thread, "torque_calc", TORQUE_TASK_STACK_DEPTH, NULL,
              TORQUE_TASK_PRIORITY, NULL);
  create_foo_task();
  create_timer();
  create_bar_task();

  vTaskStartScheduler();

  for (;;) {}
}

void vApplicationMallocFailedHook(void) {
  __disable_irq();
  for (;;) {}
}

static void pins_init(void) {
  for (int i = 0; i < PIN_COUNT_; i++) { digital_pin_init((digital_pin_t)i); }
}

void HAL_MspInit(void) {
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_NVIC_SetPriority(PendSV_IRQn, 15, 0);
  HAL_PWREx_DisableUCPDDeadBattery();
}

void error_handler(void) {
  __disable_irq();
  for (;;) {}
}

void nmi_handler(void) {
  for (;;) {}
}
void hard_fault_handler(void) {
  __disable_irq();
  for (;;) {}
}
void memory_manage_fault_handler(void) {
  for (;;) {}
}
void bus_fault_handler(void) {
  for (;;) {}
}
void usage_fault_handler(void) {
  for (;;) {}
}
void debug_monitor_handler(void) {}
