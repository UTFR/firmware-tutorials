#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "can.h"
#include "cQueue.h"
#include "can_types/common.h"
#include "utfr_hal.h"

Queue_t can_rx_queue;
can_msg_t can_rx_queue_data[CAN_RX_QUEUE_SIZE];
extern can_t can;

static bool is_apb1_48mhz(void);
static HAL_StatusTypeDef can_init_filters(can_t *can, const can_filter_t *filters, int num_filters);
static HAL_StatusTypeDef can_msp_init(can_t *can);
static void can_msp_deinit(can_t *can);
static void register_can_instance(can_t *can, can_instance_t instance);
static FDCAN_GlobalTypeDef *can_get_instance(can_rx_t rx);

static int num_can_initialized = 0;
static can_t *can_instances[NUM_CAN_CONTROLLERS];

void fdcan1_it0_irq_handler(void) { HAL_FDCAN_IRQHandler(can_instances[CAN1]->handle); }
void fdcan1_it1_irq_handler(void) { HAL_FDCAN_IRQHandler(can_instances[CAN1]->handle); }

void fdcan2_it0_irq_handler(void) { HAL_FDCAN_IRQHandler(can_instances[CAN2]->handle); }
void fdcan2_it1_irq_handler(void) { HAL_FDCAN_IRQHandler(can_instances[CAN2]->handle); }

void fdcan3_it0_irq_handler(void) { HAL_FDCAN_IRQHandler(can_instances[CAN3]->handle); }
void fdcan3_it1_irq_handler(void) { HAL_FDCAN_IRQHandler(can_instances[CAN3]->handle); }

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hcan, uint32_t its) {
  UNUSED(hcan);
  UNUSED(its);

  FDCAN_RxHeaderTypeDef rx_header;

  can_msg_t msg = {0};
  if (HAL_FDCAN_GetRxMessage(can.handle, FDCAN_RX_FIFO0, &rx_header, msg.data) != HAL_OK) {
    printf("err receiving CAN msg\n");
    return;
  }

  msg.id = rx_header.Identifier;
  msg.dlc = rx_header.DataLength;
  q_push(&can_rx_queue, &msg);
}

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

static void register_can_instance(can_t *can, can_instance_t instance) {
  can_instances[instance] = can;
}

static FDCAN_HandleTypeDef can1, can2, can3;

HAL_StatusTypeDef can_init(can_t *can, const can_filter_t *const filters, const int num_filters,
                           uint32_t interrupts) {
  q_init_static(&can_rx_queue, sizeof(can_msg_t), CAN_RX_QUEUE_SIZE, FIFO, false, can_rx_queue_data,
                sizeof(can_rx_queue_data));
  FDCAN_GlobalTypeDef *instance = can_get_instance(can->rx);

  if (instance == FDCAN1) {
    can->handle = &can1;
    can->handle->Instance = FDCAN1;
    register_can_instance(can, CAN1);
  } else if (instance == FDCAN2) {
    can->handle = &can2;
    can->handle->Instance = FDCAN2;
    register_can_instance(can, CAN2);
  } else {
    can->handle = &can3;
    can->handle->Instance = FDCAN3;
    register_can_instance(can, CAN3);
  }

  can->handle->Init.ClockDivider = FDCAN_CLOCK_DIV1;
  can->handle->Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  can->handle->Init.Mode = FDCAN_MODE_NORMAL;
  can->handle->Init.AutoRetransmission = ENABLE;
  can->handle->Init.TransmitPause = DISABLE;
  can->handle->Init.ProtocolException = DISABLE;

  assert_param(is_apb1_48mhz());

  can_baudrate_t baudrate = CAN_BAUDRATE_1MBPS;
  uint32_t prescalar = 1;
  uint32_t timeseg1 = 1;
  uint32_t timeseg2 = 1;

  baudrate = CAN_BAUDRATE_1MBPS;

  switch (baudrate) {
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
  if (HAL_FDCAN_Init(can->handle) != HAL_OK) { return HAL_ERROR; }

  if (can_init_filters(can, filters, num_filters) != HAL_OK) { return HAL_ERROR; }

  HAL_FDCAN_ConfigGlobalFilter(can->handle, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE,
                               FDCAN_REJECT_REMOTE);

  if (HAL_FDCAN_Start(can->handle) != HAL_OK) { return HAL_ERROR; }

  HAL_FDCAN_ActivateNotification(can->handle, interrupts, 0);

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

    if (HAL_FDCAN_ConfigFilter(can->handle, &filter_config) != HAL_OK) { return HAL_ERROR; }
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
  } else if (can->handle->Instance == FDCAN3) {
    irqn0 = FDCAN3_IT0_IRQn;
    irqn1 = FDCAN3_IT1_IRQn;
  }

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
  case CAN3_RX_PA8:
    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpio_init.Alternate = GPIO_AF11_FDCAN3;
    rx_pin = GPIO_PIN_8;
    rx_port = GPIOA;
    break;
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
  peripheral_clk_init.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
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
  case CAN3_RX_PA8:
    rx_pin = GPIO_PIN_8;
    rx_port = GPIOA;
    break;
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
  case CAN3_TX_PA15:
    tx_pin = GPIO_PIN_15;
    tx_port = GPIOA;
    break;
  case CAN3_TX_PB4:
    tx_pin = GPIO_PIN_4;
    tx_port = GPIOB;
    break;
  }

  HAL_GPIO_DeInit(rx_port, rx_pin);
  HAL_GPIO_DeInit(tx_port, tx_pin);

  IRQn_Type irqn0 = FDCAN1_IT0_IRQn;
  IRQn_Type irqn1 = FDCAN1_IT1_IRQn;
  if (can->handle->Instance == FDCAN1) {
  } else if (can->handle->Instance == FDCAN2) {
    irqn0 = FDCAN2_IT0_IRQn;
    irqn1 = FDCAN2_IT1_IRQn;
  } else if (can->handle->Instance == FDCAN3) {
    irqn0 = FDCAN3_IT0_IRQn;
    irqn1 = FDCAN3_IT1_IRQn;
  }

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
  case CAN3_RX_PA8:  return FDCAN3;
  }
  __builtin_unreachable();
}

void can_send(uint32_t id, const uint8_t *payload, uint32_t dlc) {
  static FDCAN_TxHeaderTypeDef tx_header;
  tx_header.Identifier = id;
  tx_header.IdType = FDCAN_STANDARD_ID;

  switch (dlc) {
  case 0:  tx_header.DataLength = FDCAN_DLC_BYTES_0; break;
  case 1:  tx_header.DataLength = FDCAN_DLC_BYTES_1; break;
  case 2:  tx_header.DataLength = FDCAN_DLC_BYTES_2; break;
  case 3:  tx_header.DataLength = FDCAN_DLC_BYTES_3; break;
  case 4:  tx_header.DataLength = FDCAN_DLC_BYTES_4; break;
  case 5:  tx_header.DataLength = FDCAN_DLC_BYTES_5; break;
  case 6:  tx_header.DataLength = FDCAN_DLC_BYTES_6; break;
  case 7:  tx_header.DataLength = FDCAN_DLC_BYTES_7; break;
  case 8:  tx_header.DataLength = FDCAN_DLC_BYTES_8; break;
  default: printf("nah\n\r"); break;
  }

  while (HAL_FDCAN_GetTxFifoFreeLevel(can.handle) == 0);

  tx_header.TxFrameType = FDCAN_DATA_FRAME;

  // uint32_t timeout = HAL_GetTick() + 1000;
  // while (HAL_FDCAN_GetTxFifoFreeLevel(can.handle) == 0) {
  //   if (HAL_GetTick() > timeout) {
  //     printf("TX FIFO timeout\n\r");
  //     return;
  //   }
  // }

  if (HAL_FDCAN_AddMessageToTxFifoQ(can.handle, &tx_header, payload) != HAL_OK) {
    printf("could not send\n\r");
  }
}

void can_recv(can_msg_t *msg) { CAN_RECV(&can_rx_queue, msg); }
