// Provided bring-up for the LECTURE2_CONCURRENCY board -- the real HAL/boot
// boilerplate every board needs, so the lecture content (src/race_condition_*,
// src/mutex.c, src/queue.c) can stay focused on the concurrency bug itself.
//
// There's no RTOS here (see board.cmake), so the "two concurrent flows of
// control racing on shared state" from the original lecture are realized as
// a real mainline-vs-timer-ISR race: torque_calculator_thread() runs from a
// TIM6 interrupt every CONTROL_LOOP_PERIOD_MS, motor_control_thread() runs
// back-to-back in main()'s bare loop.
#include "config.h"
#include "main.h"
#include "torque_control.h"
#include "UTFR_BOOT_UTILS/boot.h"
#include "UTFR_BOOT_UTILS/image.h"
#include "UTFR_BOOT_UTILS/vector.h"
#include "UTFR_DIGITAL/pin_driver.h"
#include "UTFR_UART/uart.h"
#include "git/git.h"

static void pins_init(void);
static void control_timer_init(void);

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

static TIM_HandleTypeDef tim6;

int main(void) {
  HAL_Init();
  sys_clock_config();

  if (uart_init(&serial_uart) != HAL_OK) { error_handler(); }
  uart_set_io_handle(&serial_uart.handle);

  pins_init();
  control_timer_init();

  for (;;) { motor_control_thread(); }
}

static void pins_init(void) {
  for (int i = 0; i < PIN_COUNT_; i++) { digital_pin_init((digital_pin_t)i); }
}

// Periodic interrupt standing in for the "torque calculator thread" -- fires
// every CONTROL_LOOP_PERIOD_MS and races with main()'s motor_control_thread().
static void control_timer_init(void) {
  __HAL_RCC_TIM6_CLK_ENABLE();

  uint32_t timer_clock = HAL_RCC_GetPCLK1Freq();
  uint32_t prescaler = (timer_clock / 1000U) - 1U; // 1 kHz tick

  tim6.Instance = TIM6;
  tim6.Init.Prescaler = prescaler;
  tim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  tim6.Init.Period = CONTROL_LOOP_PERIOD_MS - 1U;
  tim6.Init.ClockDivision = 0;
  if (HAL_TIM_Base_Init(&tim6) != HAL_OK) { error_handler(); }
  if (HAL_TIM_Base_Start_IT(&tim6) != HAL_OK) { error_handler(); }

  HAL_NVIC_SetPriority(TIM6_DAC_IRQn, INTERRUPT_PRIORITY, 0);
  HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

void tim6_dac_irq_handler(void) { HAL_TIM_IRQHandler(&tim6); }

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM6) { torque_calculator_thread(); }
}

void systick_handler(void) { HAL_IncTick(); }

void HAL_MspInit(void) {
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
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
