#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "can.h"
#include "can_types/common.h"
#include "UTFR_LOGGING/logger.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "utfr_hal.h"

#define CAN_TX_TIMEOUT_MS 10

static bool is_apb1_48mhz(void);
static HAL_StatusTypeDef can_init_filters(can_t *can, const can_filter_t *filters, int num_filters);
static HAL_StatusTypeDef can_msp_init(can_t *can);
static void can_msp_deinit(can_t *can);
static void register_can_instance(can_t *can, can_instance_t instance);
static FDCAN_GlobalTypeDef *can_get_instance(can_rx_t rx);
static bool can_tx_has_free_buffer(can_t *can);

static int num_can_initialized = 0;
static can_t *can_lut[NUM_CAN_CONTROLLERS];

void fdcan1_it0_irq_handler(void) { HAL_FDCAN_IRQHandler(can_lut[CAN1]->handle); }
void fdcan1_it1_irq_handler(void) { HAL_FDCAN_IRQHandler(can_lut[CAN1]->handle); }

void fdcan2_it0_irq_handler(void) { HAL_FDCAN_IRQHandler(can_lut[CAN2]->handle); }
void fdcan2_it1_irq_handler(void) { HAL_FDCAN_IRQHandler(can_lut[CAN2]->handle); }

#if !defined(UTFR_MCU_FAMILY_H7)
// H7's dual-core FDCAN parts (H745/755) only have FDCAN1/FDCAN2 -- no FDCAN3.
void fdcan3_it0_irq_handler(void) { HAL_FDCAN_IRQHandler(can_lut[CAN3]->handle); }
void fdcan3_it1_irq_handler(void) { HAL_FDCAN_IRQHandler(can_lut[CAN3]->handle); }
#endif

static const uint32_t PRESCALAR_1000KBPS = 3;
static const uint32_t PRESCALAR_500KBPS = 6;
static const uint32_t PRESCALAR_250KBPS = 12;
static const uint32_t PRESCALAR_125KBPS = 24;

static const uint32_t TIMESEG1_1000KBPS = 13;
static const uint32_t TIMESEG1_500KBPS = 11;
static const uint32_t TIMESEG1_250KBPS = 13;
static const uint32_t TIMESEG1_125KBPS = 13;

static const uint32_t TIMESEG2_1000KBPS = 2;
static const uint32_t TIMESEG2_500KBPS = 4;
static const uint32_t TIMESEG2_250KBPS = 2;
static const uint32_t TIMESEG2_125KBPS = 2;

static void register_can_instance(can_t *can, can_instance_t instance) { can_lut[instance] = can; }

static bool can_tx_has_free_buffer(can_t *can) {
  return HAL_FDCAN_GetTxFifoFreeLevel(can->handle) > 0;
}

void can_transmit(can_instance_t instance, can_msg_t *msg) {
  if ((msg == NULL) || (instance >= NUM_CAN_CONTROLLERS) || (can_lut[instance] == NULL)
      || (can_lut[instance]->handle == NULL)) {
    return;
  }

  FDCAN_TxHeaderTypeDef tx_header = {0};
  tx_header.DataLength = msg->dlc;
  tx_header.Identifier = (int)msg->raw_id;
  tx_header.IdType = FDCAN_STANDARD_ID;
  if (msg->raw_id > MAX_STD_CAN_ID) { tx_header.IdType = FDCAN_EXTENDED_ID; }

  uint32_t deadline = HAL_GetTick() + CAN_TX_TIMEOUT_MS;
  while (!can_tx_has_free_buffer(can_lut[instance])) {
    if (HAL_GetTick() >= deadline) { return; }
  }

  if (HAL_FDCAN_AddMessageToTxFifoQ(can_lut[instance]->handle, &tx_header, msg->data) != HAL_OK) {
    LOGE("%s: could not tx (%d)", can_lut[instance]->name, can_lut[instance]->handle->ErrorCode);
  }
}

// https://community.st.com/t5/stm32-mcus-products/can-auto-bus-off-recovery-not-happening-on-stm32g474re-mcu/td-p/721331
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs) {
  // If Bus-Off error occured
  if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != 0) {
    hfdcan->Instance->CCCR &= ~FDCAN_CCCR_INIT; // Recover from Bus-Off
  }
}

static FDCAN_HandleTypeDef can1, can2;
#if !defined(UTFR_MCU_FAMILY_H7)
static FDCAN_HandleTypeDef can3;
#endif

HAL_StatusTypeDef can_init(can_t *can, const can_filter_t *const filters, const int num_filters,
                           uint32_t interrupts) {
  FDCAN_GlobalTypeDef *instance = can_get_instance(can->rx);

  if (instance == FDCAN1) {
    can->handle = &can1;
    can->handle->Instance = FDCAN1;
    register_can_instance(can, CAN1);
  } else if (instance == FDCAN2) {
    can->handle = &can2;
    can->handle->Instance = FDCAN2;
    register_can_instance(can, CAN2);
  }
#if !defined(UTFR_MCU_FAMILY_H7)
  else {
    can->handle = &can3;
    can->handle->Instance = FDCAN3;
    register_can_instance(can, CAN3);
  }
#endif

#if !defined(UTFR_MCU_FAMILY_H7)
  // H7's FDCAN_InitTypeDef has no ClockDivider field -- the kernel clock
  // reaching the FDCAN prescaler is selected entirely via
  // RCC_PeriphCLKInitTypeDef.FdcanClockSelection below, not a per-instance
  // divider register like G4's.
  can->handle->Init.ClockDivider = FDCAN_CLOCK_DIV1;
#endif
  can->handle->Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  can->handle->Init.Mode = FDCAN_MODE_NORMAL;
  can->handle->Init.AutoRetransmission = ENABLE;
  can->handle->Init.TransmitPause = DISABLE;
  can->handle->Init.ProtocolException = DISABLE;

  assert_param(is_apb1_48mhz());

  uint32_t prescalar = 1;
  uint32_t timeseg1 = 1;
  uint32_t timeseg2 = 1;

  switch (can->baudrate) {
  case CAN_BAUDRATE_1MBPS:
    prescalar = PRESCALAR_1000KBPS;
    timeseg1 = TIMESEG1_1000KBPS;
    timeseg2 = TIMESEG2_1000KBPS;
    break;
  case CAN_BAUDRATE_500KBPS:
    prescalar = PRESCALAR_500KBPS;
    timeseg1 = TIMESEG1_500KBPS;
    timeseg2 = TIMESEG2_500KBPS;
    break;
  case CAN_BAUDRATE_250KBPS:
    prescalar = PRESCALAR_250KBPS;
    timeseg1 = TIMESEG1_250KBPS;
    timeseg2 = TIMESEG2_250KBPS;
    break;
  case CAN_BAUDRATE_125KBPS:
    prescalar = PRESCALAR_125KBPS;
    timeseg1 = TIMESEG1_125KBPS;
    timeseg2 = TIMESEG2_125KBPS;
    break;
  }

  LOGT("Prescalar: %d, Timeseg1: %d, Timeseg2: %d", prescalar, timeseg1, timeseg2);
  LOGT("%d filters", num_filters);

  can->handle->Init.NominalPrescaler = prescalar;
  can->handle->Init.NominalSyncJumpWidth = 1;
  can->handle->Init.NominalTimeSeg1 = timeseg1;
  can->handle->Init.NominalTimeSeg2 = timeseg2;
  can->handle->Init.DataPrescaler = 1;
  can->handle->Init.DataTimeSeg1 = 1;
  can->handle->Init.DataTimeSeg2 = 1;

  can->handle->Init.StdFiltersNbr = num_filters;
  can->handle->Init.ExtFiltersNbr = 1;
  can->handle->Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;

  can_msp_init(can);
  if (HAL_FDCAN_Init(can->handle) != HAL_OK) {
    LOGE("could not init fdcan: 0x%x", can->handle->ErrorCode);
    return HAL_ERROR;
  }

  if (can_init_filters(can, filters, num_filters) != HAL_OK) {
    LOGE("could not init fdcan filters");
    return HAL_ERROR;
  }

  FDCAN_FilterTypeDef filter_config = {0};
  filter_config.FilterID1 = 0x0;
  filter_config.FilterID2 = 0x0;
  filter_config.IdType = FDCAN_EXTENDED_ID;
  filter_config.FilterType = FDCAN_FILTER_MASK;
  filter_config.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
  filter_config.FilterIndex = 0;

  if (HAL_FDCAN_ConfigFilter(can->handle, &filter_config) != HAL_OK) {
    LOGE("could not config fdcan extended filter");
    return HAL_ERROR;
  }

  /* Route non-matching frames to the same FIFO the caller services with IRQs.
   * WDAQ ICAN/PCAN use FIFO1 only — accepting into FIFO0 there would leave
   * frames unread until overflow. */
  uint32_t nonmatching_fifo = FDCAN_ACCEPT_IN_RX_FIFO0;
  if ((interrupts & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != 0U) {
    nonmatching_fifo = FDCAN_ACCEPT_IN_RX_FIFO1;
  }
  HAL_FDCAN_ConfigGlobalFilter(can->handle, nonmatching_fifo, nonmatching_fifo, FDCAN_REJECT_REMOTE,
                               FDCAN_REJECT_REMOTE);

  if (HAL_FDCAN_Start(can->handle) != HAL_OK) {
    LOGE("could not start fdcan");
    return HAL_ERROR;
  }

  if (HAL_FDCAN_ActivateNotification(can->handle, interrupts | FDCAN_IT_BUS_OFF, 0) != HAL_OK) {
    LOGE("could not activate fdcan notifications");
    return HAL_ERROR;
  }

  return HAL_OK;
}

HAL_StatusTypeDef can_deinit(can_t *can) {
  can_msp_deinit(can);
  return HAL_FDCAN_DeInit(can->handle);
}

static HAL_StatusTypeDef can_init_filters(can_t *can, const can_filter_t *filters,
                                          int num_filters) {
  assert_param(num_filters < MAX_NUM_FILTERS);
  FDCAN_FilterTypeDef filter_config = {0};
  for (int i = 0; i < num_filters; i++) {
    filter_config.FilterIndex = i;
    filter_config.IdType = FDCAN_STANDARD_ID;

    switch (filters[i].kind) {
    case CAN_FILTER_KIND_EXACT:
      filter_config.FilterType = FDCAN_FILTER_RANGE;
      filter_config.FilterID1 = filters[i].exact;
      filter_config.FilterID2 = filters[i].exact;
      break;
    case CAN_FILTER_KIND_MASK:
      filter_config.FilterType = FDCAN_FILTER_MASK;
      filter_config.FilterID1 = filters[i].mask;
      filter_config.FilterID2 = filters[i].mask;
      break;
    case CAN_FILTER_KIND_RANGE:
      filter_config.FilterType = FDCAN_FILTER_RANGE;
      filter_config.FilterID1 = filters[i].range.from;
      filter_config.FilterID2 = filters[i].range.to;
      break;
    }

    switch (filters[i].action) {
    case CAN_FILTER_ACTION_REJECT:  filter_config.FilterConfig = FDCAN_FILTER_REJECT; break;
    case CAN_FILTER_ACTION_RXFIFO0: filter_config.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; break;
    case CAN_FILTER_ACTION_RXFIFO1: filter_config.FilterConfig = FDCAN_FILTER_TO_RXFIFO1; break;
    }

    assert_param(filter_config.FilterID1 <= MAX_STD_CAN_ID);
    assert_param(filter_config.FilterID2 <= MAX_STD_CAN_ID);

    if (HAL_FDCAN_ConfigFilter(can->handle, &filter_config) != HAL_OK) {
      LOGE("could not config fdcan filter");
      return HAL_ERROR;
    }
  }
  return HAL_OK;
}

static HAL_StatusTypeDef can_msp_init(can_t *can) {
  GPIO_InitTypeDef gpio_init = {0};
  IRQn_Type irqn0 = FDCAN1_IT0_IRQn;
  IRQn_Type irqn1 = FDCAN1_IT1_IRQn;
  uint16_t rx_pin = 0;
  uint16_t tx_pin = 0;
  GPIO_TypeDef *rx_port = GPIOA;
  GPIO_TypeDef *tx_port = GPIOA;

  if (num_can_initialized++ == 0) { __HAL_RCC_FDCAN_CLK_ENABLE(); }

  if (can->handle->Instance == FDCAN1) {
    irqn0 = FDCAN1_IT0_IRQn;
    irqn1 = FDCAN1_IT1_IRQn;
  } else if (can->handle->Instance == FDCAN2) {
    irqn0 = FDCAN2_IT0_IRQn;
    irqn1 = FDCAN2_IT1_IRQn;
  }
#if !defined(UTFR_MCU_FAMILY_H7)
  else if (can->handle->Instance == FDCAN3) {
    irqn0 = FDCAN3_IT0_IRQn;
    irqn1 = FDCAN3_IT1_IRQn;
  }
#endif

  switch (can->rx) {
  case CAN1_RX_PA11:
    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpio_init.Alternate = GPIO_AF9_FDCAN1;
    rx_pin = GPIO_PIN_11;
    rx_port = GPIOA;
    break;
  case CAN1_RX_PB8:
    __HAL_RCC_GPIOB_CLK_ENABLE();
    gpio_init.Alternate = GPIO_AF9_FDCAN1;
    rx_pin = GPIO_PIN_8;
    rx_port = GPIOB;
    break;
  case CAN2_RX_PB5:
    __HAL_RCC_GPIOB_CLK_ENABLE();
    gpio_init.Alternate = GPIO_AF9_FDCAN2;
    rx_pin = GPIO_PIN_5;
    rx_port = GPIOB;
    break;
  case CAN2_RX_PB12:
    __HAL_RCC_GPIOB_CLK_ENABLE();
    gpio_init.Alternate = GPIO_AF9_FDCAN2;
    rx_pin = GPIO_PIN_12;
    rx_port = GPIOB;
    break;
#if !defined(UTFR_MCU_FAMILY_H7)
  case CAN3_RX_PA8:
    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpio_init.Alternate = GPIO_AF11_FDCAN3;
    rx_pin = GPIO_PIN_8;
    rx_port = GPIOA;
    break;
#endif
  }

  switch (can->tx) {
  case CAN1_TX_PA12:
    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpio_init.Alternate = GPIO_AF9_FDCAN1;
    tx_pin = GPIO_PIN_12;
    tx_port = GPIOA;
    break;
  case CAN1_TX_PB9:
    __HAL_RCC_GPIOB_CLK_ENABLE();
    gpio_init.Alternate = GPIO_AF9_FDCAN1;
    tx_pin = GPIO_PIN_9;
    tx_port = GPIOB;
    break;
  case CAN2_TX_PB6:
    __HAL_RCC_GPIOB_CLK_ENABLE();
    gpio_init.Alternate = GPIO_AF9_FDCAN2;
    tx_pin = GPIO_PIN_6;
    tx_port = GPIOB;
    break;
  case CAN2_TX_PB13:
    __HAL_RCC_GPIOB_CLK_ENABLE();
    gpio_init.Alternate = GPIO_AF9_FDCAN2;
    tx_pin = GPIO_PIN_13;
    tx_port = GPIOB;
    break;
#if !defined(UTFR_MCU_FAMILY_H7)
  case CAN3_TX_PA15:
    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpio_init.Alternate = GPIO_AF11_FDCAN3;
    tx_pin = GPIO_PIN_15;
    tx_port = GPIOA;
    break;
  case CAN3_TX_PB4:
    __HAL_RCC_GPIOB_CLK_ENABLE();
    gpio_init.Alternate = GPIO_AF11_FDCAN3;
    tx_pin = GPIO_PIN_4;
    tx_port = GPIOB;
    break;
#endif
  }

  gpio_init.Mode = GPIO_MODE_AF_PP;
  gpio_init.Pin = rx_pin;
  gpio_init.Pull = GPIO_PULLUP;
  gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(rx_port, &gpio_init);

  gpio_init.Pin = tx_pin;
  gpio_init.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(tx_port, &gpio_init);

  RCC_PeriphCLKInitTypeDef peripheral_clk_init = {0};

  peripheral_clk_init.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
#if defined(UTFR_MCU_FAMILY_H7)
  // H7 has no PCLK1 option for FDCAN's kernel clock (only HSE/PLL/PLL2) --
  // this board has no CAN transceiver to actually talk to regardless (see
  // bootloader/board_stm32h755.cmake), so bit-timing correctness here is
  // moot; PLL matches ST's own examples for this family.
  peripheral_clk_init.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;
#else
  peripheral_clk_init.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
#endif
  if (HAL_RCCEx_PeriphCLKConfig(&peripheral_clk_init) != HAL_OK) { return HAL_ERROR; }

  HAL_NVIC_SetPriority(irqn0, CAN_IRQ_PREEMPT_PRIORITY, CAN_IRQ_SUBPRIORITY);
  HAL_NVIC_EnableIRQ(irqn0);
  HAL_NVIC_SetPriority(irqn1, CAN_IRQ_PREEMPT_PRIORITY, CAN_IRQ_SUBPRIORITY);
  HAL_NVIC_EnableIRQ(irqn1);

  return HAL_OK;
}

static void can_msp_deinit(can_t *can) {
  uint16_t rx_pin = 0;
  uint16_t tx_pin = 0;
  GPIO_TypeDef *rx_port = GPIOA;
  GPIO_TypeDef *tx_port = GPIOA;

  if (--num_can_initialized == 0) { __HAL_RCC_FDCAN_CLK_DISABLE(); }

  switch (can->rx) {
  case CAN1_RX_PA11:
    rx_pin = GPIO_PIN_11;
    rx_port = GPIOA;
    break;
  case CAN1_RX_PB8:
    rx_pin = GPIO_PIN_8;
    rx_port = GPIOB;
    break;
  case CAN2_RX_PB5:
    rx_pin = GPIO_PIN_5;
    rx_port = GPIOB;
    break;
  case CAN2_RX_PB12:
    rx_pin = GPIO_PIN_12;
    rx_port = GPIOB;
    break;
#if !defined(UTFR_MCU_FAMILY_H7)
  case CAN3_RX_PA8:
    rx_pin = GPIO_PIN_8;
    rx_port = GPIOA;
    break;
#endif
  }

  switch (can->tx) {
  case CAN1_TX_PA12:
    tx_pin = GPIO_PIN_12;
    tx_port = GPIOA;
    break;
  case CAN1_TX_PB9:
    tx_pin = GPIO_PIN_9;
    tx_port = GPIOB;
    break;
  case CAN2_TX_PB6:
    tx_pin = GPIO_PIN_6;
    tx_port = GPIOB;
    break;
  case CAN2_TX_PB13:
    tx_pin = GPIO_PIN_13;
    tx_port = GPIOB;
    break;
#if !defined(UTFR_MCU_FAMILY_H7)
  case CAN3_TX_PA15:
    tx_pin = GPIO_PIN_15;
    tx_port = GPIOA;
    break;
  case CAN3_TX_PB4:
    tx_pin = GPIO_PIN_4;
    tx_port = GPIOB;
    break;
#endif
  }

  HAL_GPIO_DeInit(rx_port, rx_pin);
  HAL_GPIO_DeInit(tx_port, tx_pin);

  IRQn_Type irqn0 = FDCAN1_IT0_IRQn;
  IRQn_Type irqn1 = FDCAN1_IT1_IRQn;
  if (can->handle->Instance == FDCAN1) {
  } else if (can->handle->Instance == FDCAN2) {
    irqn0 = FDCAN2_IT0_IRQn;
    irqn1 = FDCAN2_IT1_IRQn;
  }
#if !defined(UTFR_MCU_FAMILY_H7)
  else if (can->handle->Instance == FDCAN3) {
    irqn0 = FDCAN3_IT0_IRQn;
    irqn1 = FDCAN3_IT1_IRQn;
  }
#endif

  HAL_NVIC_DisableIRQ(irqn0);
  HAL_NVIC_DisableIRQ(irqn1);
}

static bool is_apb1_48mhz(void) {
  RCC_ClkInitTypeDef clk_config;
  uint32_t flash_latency;
  HAL_RCC_GetClockConfig(&clk_config, &flash_latency);
  return clk_config.SYSCLKSource == RCC_SYSCLKSOURCE_HSE
         && clk_config.AHBCLKDivider == RCC_SYSCLK_DIV1
         && clk_config.APB1CLKDivider == RCC_HCLK_DIV1;
}

static FDCAN_GlobalTypeDef *can_get_instance(can_rx_t rx) {
  switch (rx) {
  case CAN1_RX_PA11:
  case CAN1_RX_PB8:  return FDCAN1;
  case CAN2_RX_PB5:
  case CAN2_RX_PB12: return FDCAN2;
#if !defined(UTFR_MCU_FAMILY_H7)
  case CAN3_RX_PA8:  return FDCAN3;
#endif
  }
  __builtin_unreachable();
}
