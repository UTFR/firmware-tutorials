#include <stdio.h>

#include "main.h"
#include "UTFR_BOOT_UTILS/boot.h"
#include "UTFR_BOOT_UTILS/image.h"
#include "UTFR_UART/uart.h"
#include "dfu.h"
#include "shared_ram.h"
#include "utfr_hal.h"
#include "vector.h"
#include "git/git.h"
#include "can.h"

image_header_t image_header __attribute__((used, section(".image_header")))
= {.magic = IMAGE_MAGIC,
   .image_kind = IMAGE_KIND_BOOTLOADER,
   .version_major = 0,
   .version_minor = 1,
   .version_patch = 0,
   .vector_addr = (uint32_t)&vector_table,
   .git_sha = GIT_HASH};

uart_t serial_uart = {.baudrate = UART_BAUDRATE_115200, .rx = USART2_RX_PA3, .tx = USART2_TX_PA2};

#if !defined(UTFR_MCU_FAMILY_H7)
// CAN-based DFU is G4-only for now: bootloader/src/can.c hardcodes G474's
// 3-instance FDCAN config (FDCAN3, ClockDivider, PCLK1 clock source, AF11 --
// none of which exist on H7's FDCAN1/2), and isn't built for any H7
// bootloader board (see bootloader/CMakeLists.txt). H7 bootloaders are
// validate-and-jump only -- see the #else main() body below.
#define RCAN_RX_ACM CAN3_RX_PA8
#define RCAN_TX_ACM CAN3_TX_PB4

#define FCAN_RX_FC CAN2_RX_PB12
#define FCAN_TX_FC CAN2_TX_PB13

#define RCAN_RX_RC CAN2_RX_PB5
#define RCAN_TX_RC CAN2_TX_PB6

#define RCAN_RX_WDAQ CAN2_RX_PB5
#define RCAN_TX_WDAQ CAN2_TX_PB6

can_t can = {.name = "FLASH CAN", .rx = RCAN_RX_RC, .tx = RCAN_TX_RC};

static can_filter_t filters[] = {
  {.kind = CAN_FILTER_KIND_EXACT, .exact = 0x100, .action = CAN_FILTER_ACTION_RXFIFO0},
  {.kind = CAN_FILTER_KIND_EXACT, .exact = 0x101, .action = CAN_FILTER_ACTION_RXFIFO0},
  {.kind = CAN_FILTER_KIND_EXACT, .exact = 0x102, .action = CAN_FILTER_ACTION_RXFIFO0},
  {.kind = CAN_FILTER_KIND_EXACT, .exact = 0x103, .action = CAN_FILTER_ACTION_RXFIFO0},
  {.kind = CAN_FILTER_KIND_EXACT, .exact = 0x104, .action = CAN_FILTER_ACTION_RXFIFO0},
  {.kind = CAN_FILTER_KIND_EXACT, .exact = 0x105, .action = CAN_FILTER_ACTION_RXFIFO0},
  {.kind = CAN_FILTER_KIND_EXACT, .exact = 0x106, .action = CAN_FILTER_ACTION_RXFIFO0},
};
#endif

// GPIOC, GPIO_PIN_11 -- inert status pin on both families (not tied to any
// board-specific LED); harmless to leave wired the same way on H7.
static void status_led_init(void) {
  __HAL_RCC_GPIOC_CLK_ENABLE();
  GPIO_InitTypeDef init = {0};
  init.Mode = GPIO_MODE_OUTPUT_PP;
  init.Pin = GPIO_PIN_11;
  init.Pull = GPIO_NOPULL;
  init.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &init);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);
}

int main(void) {
  HAL_Init();
  sys_clock_config();
  shared_ram_init();
#if !defined(UTFR_MCU_FAMILY_H7)
  // uart_msp_init() is an explicit stub on H7 (see common/UTFR_UART/uart.c)
  // that calls error_handler() -- calling uart_init() here would halt the
  // bootloader before it ever reaches the image-validate-and-jump logic
  // below. io_uart_handle stays NULL on H7, same as controllers/H7dev:
  // printf() still reaches ITM/SWO (see common/UTFR_BOOT_UTILS/syscalls.c),
  // it just never reaches a physical UART.
  uart_init(&serial_uart);
  uart_set_io_handle(&serial_uart.handle);
#endif
  status_led_init();

#if defined(UTFR_MCU_FAMILY_H7)
  // Minimal bootloader: validate the app image and jump, no CAN-based DFU
  // (see the #if guards above -- can.c/dfu.c aren't built for H7 bootloader
  // boards). If the image is invalid there's no DFU fallback to recover
  // into on H7 yet, so this just halts rather than pretending to recover.
  printf("bootloader started (H7, no DFU)...\n\r");
  const image_header_t *header = image_get_and_validate_header(IMAGE_SLOT_APP);
  if (header != NULL) {
    printf("jumping to image...\n");
    image_start(header);
  } else {
    printf("could not get or validate app image -- no DFU fallback on H7, halting\n");
  }
#else
  switch (shared_ram_get_controller()) {
  case 1:
    printf("init CAN for ACM\n\r");
    can.rx = RCAN_RX_ACM;
    can.tx = RCAN_TX_ACM;
    break;
  case 2:
    printf("init CAN for FC\n\r");
    can.rx = FCAN_RX_FC;
    can.tx = FCAN_TX_FC;
    break;
  case 3:
    printf("init CAN for RC\n\r");
    can.rx = RCAN_RX_RC;
    can.tx = RCAN_TX_RC;
    break;
  case 4:
    printf("init CAN for WDAQ\n\r");
    can.rx = RCAN_RX_WDAQ;
    can.tx = RCAN_TX_WDAQ;
    break;
  default: printf("invalid controller %ld\n\r", shared_ram_get_controller()); break;
  }

  can_init(&can, filters, sizeof(filters) / sizeof(filters[0]),
           FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO1_NEW_MESSAGE);

  printf("bootloader started...\n\r");

  image_header_t dfu_header = {0};
  dfu_ctx_t ctx = {
    .can_recv = can_recv,
    .can_send = can_send,
    .header = &dfu_header,
  };

  printf("DFU Flag set: %d\n\r", shared_ram_is_flag_set(SHARED_RAM_FLAG_DFU_REQUESTED));

  if (shared_ram_is_flag_set(SHARED_RAM_FLAG_DFU_REQUESTED)) {
    do_dfu(&ctx);
    HAL_NVIC_SystemReset();
  }

  const image_header_t *header = image_get_and_validate_header(IMAGE_SLOT_APP);
  if (header != NULL) {
    printf("jumping to image...\n");
    image_start(header);
  } else {
    printf("could not get or validate app image...\n");
    do_dfu(&ctx);
  }
#endif

  for (;;) {}

  return 0;
}

void bootloader_image_deinit(void) {
  uart_deinit(&serial_uart);
#if !defined(UTFR_MCU_FAMILY_H7)
  can_deinit(&can);
#endif
  __HAL_RCC_GPIOA_CLK_DISABLE();
  __HAL_RCC_GPIOC_CLK_DISABLE();
  HAL_GPIO_DeInit(GPIOC, GPIO_PIN_11);
  HAL_DeInit();
}

void HAL_MspInit(void) {
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  HAL_NVIC_SetPriority(PendSV_IRQn, 15, 0);
#if !defined(UTFR_MCU_FAMILY_H7)
  // PWR is always-on on H7 (no clock-gate macro), and UCPD dead-battery is
  // a G4-only USB-PD feature.
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWREx_DisableUCPDDeadBattery();
#endif
}

void HAL_MspDeInit(void) {
#if !defined(UTFR_MCU_FAMILY_H7)
  __HAL_RCC_PWR_CLK_DISABLE();
#endif
  __HAL_RCC_SYSCFG_CLK_DISABLE();
}

void error_handler(void) {
  for (;;) { __asm__(""); }
}

void nmi_handler(void) {
  for (;;) {}
}
void hard_fault_handler(void) {
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

void systick_handler(void) { HAL_IncTick(); }
