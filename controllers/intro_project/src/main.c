// Provided bring-up for the INTRO_PROJECT board: real HAL/boot/FreeRTOS
// boilerplate plus every bsp_*_init() call, so intro_projects/<you>/... (see
// src/intro_project.c) can start straight from a real, already-initialized
// board and focus entirely on the concurrency architecture.
#include "main.h"
#include "bsp.h"
#include "UTFR_BOOT_UTILS/boot.h"
#include "UTFR_BOOT_UTILS/image.h"
#include "UTFR_BOOT_UTILS/vector.h"
#include "UTFR_DIGITAL/pin_driver.h"
#include "UTFR_LOGGING/logger.h"
#include "UTFR_UART/uart.h"
#include "git/git.h"
#include "FreeRTOS.h"
#include "task.h"

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
  [PIN_LCD_CS] = {GPIOB, GPIO_PIN_0,            DIGITAL_OUTPUT, DIGITAL_NOPULL,   GPIO_PIN_SET},
  [PIN_BMS_CS] = {GPIOB, GPIO_PIN_1,            DIGITAL_OUTPUT, DIGITAL_NOPULL,   GPIO_PIN_SET},

  [PIN_TS_ON_BUTTON] = {GPIOB, GPIO_PIN_4,             DIGITAL_INPUT, DIGITAL_PULLUP, GPIO_PIN_RESET},
  [PIN_RTD_BUTTON] = {GPIOB, GPIO_PIN_5,             DIGITAL_INPUT, DIGITAL_PULLUP, GPIO_PIN_RESET},

  [PIN_AIR_PLUS] = {GPIOB, GPIO_PIN_6,            DIGITAL_OUTPUT, DIGITAL_NOPULL, GPIO_PIN_RESET},
  [PIN_AIR_MINUS] = {GPIOB, GPIO_PIN_7,            DIGITAL_OUTPUT, DIGITAL_NOPULL, GPIO_PIN_RESET},
  [PIN_PRECHARGE] = {GPIOB, GPIO_PIN_8,            DIGITAL_OUTPUT, DIGITAL_NOPULL, GPIO_PIN_RESET},

  [PIN_WHEELSPEED_FL]
  = {GPIOC, GPIO_PIN_6, DIGITAL_INTERRUPT_FALLING, DIGITAL_PULLUP, GPIO_PIN_RESET},
  [PIN_WHEELSPEED_FR]
  = {GPIOC, GPIO_PIN_7, DIGITAL_INTERRUPT_FALLING, DIGITAL_PULLUP, GPIO_PIN_RESET},
  [PIN_WHEELSPEED_RL]
  = {GPIOC, GPIO_PIN_8, DIGITAL_INTERRUPT_FALLING, DIGITAL_PULLUP, GPIO_PIN_RESET},
  [PIN_WHEELSPEED_RR]
  = {GPIOC, GPIO_PIN_9, DIGITAL_INTERRUPT_FALLING, DIGITAL_PULLUP, GPIO_PIN_RESET},
};

int main(void) {
  HAL_Init();
  sys_clock_config();

  if (uart_init(&serial_uart) != HAL_OK) { error_handler(); }
  uart_set_io_handle(&serial_uart.handle);
  logger_init(&serial_uart);

  pins_init();
  bsp_adc_init();
  bsp_wheelspeed_init();
  bsp_spi_bus_init();
  bsp_can_init();

  LOGI("intro_project: bsp ready");

  app_main();

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
