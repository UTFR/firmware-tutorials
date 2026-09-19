#ifndef BOOTLOADER_CAN_H
#define BOOTLOADER_CAN_H

#include "utfr_hal.h"
#include <stdint.h>

#define CAN_RX_QUEUE_SIZE 128
#define CAN_BUF_SIZE      8

#define CAN_RECV(__QUEUE__, __MSG__)                                                               \
  do {                                                                                             \
    do {                                                                                           \
      __disable_irq();                                                                             \
      bool got = q_pop(__QUEUE__, __MSG__);                                                        \
      __enable_irq();                                                                              \
      if (got) { break; }                                                                          \
    } while (1);                                                                                   \
  } while (0);

typedef struct {
  uint16_t id;
  uint16_t dlc;
  uint8_t data[CAN_BUF_SIZE];
} can_msg_t;

void can_recv(can_msg_t *msg);
void can_send(uint32_t id, const uint8_t *payload, uint32_t dlc);

#define MAX_NUM_FILTERS          28
#define CAN_IRQ_PREEMPT_PRIORITY 6
#define CAN_IRQ_SUBPRIORITY      0

#define MAX_STD_CAN_ID      0x7FF
#define MAX_EXT_CAN_ID      0x1FFFFFFF
#define NUM_CAN_CONTROLLERS 3

#define CAN_TX_TASK_STACK_DEPTH 256
#define CAN_TX_TASK_PRIORITY    5

#define CAN_MAX_DATA_LENGTH 8

typedef enum {
  CAN1_RX_PA11,
  CAN1_RX_PB8,
  CAN2_RX_PB5,
  CAN2_RX_PB12,
  CAN3_RX_PA8,
} can_rx_t;

typedef enum {
  CAN1_TX_PA12,
  CAN1_TX_PB9,
  CAN2_TX_PB6,
  CAN2_TX_PB13,
  CAN3_TX_PA15,
  CAN3_TX_PB4
} can_tx_t;

typedef uint32_t can_id_t;

typedef struct {
  can_id_t from;
  can_id_t to;
} can_filter_range_t;

typedef can_id_t can_filter_mask_t;
typedef can_id_t can_filter_exact_t;

typedef enum {
  CAN_FILTER_KIND_EXACT,
  CAN_FILTER_KIND_RANGE,
  CAN_FILTER_KIND_MASK,
} can_filter_kind_t;

typedef enum {
  CAN_FILTER_ACTION_RXFIFO0,
  CAN_FILTER_ACTION_RXFIFO1,
  CAN_FILTER_ACTION_REJECT,
} can_filter_action_t;

typedef struct {
  can_filter_kind_t kind;
  union {
    can_filter_range_t range;
    can_filter_mask_t mask;
    can_filter_exact_t exact;
  };
  can_filter_action_t action;
} can_filter_t;

#define CAN_FILTER_ACCEPT_ALL(__ACTION__)                                                          \
  {                                                                                                \
    .kind = CAN_FILTER_KIND_RANGE,                                                                 \
    .range = {.from = 0x000, .to = MAX_STD_CAN_ID},                                                \
    .action = (__ACTION__) \
}

#define CAN_BUS_NAME_MAX_LEN 16

typedef struct {
  char name[CAN_BUS_NAME_MAX_LEN];
  FDCAN_HandleTypeDef *handle;
  can_rx_t rx;
  can_tx_t tx;
} can_t;

typedef enum { CAN1 = 0, CAN2 = 1, CAN3 = 2 } can_instance_t;

HAL_StatusTypeDef can_init(can_t *can, const can_filter_t *filters, int num_filters,
                           uint32_t interrupts);
HAL_StatusTypeDef can_deinit(can_t *can);

#endif
