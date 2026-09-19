#ifndef UTFR_CAN_H
#define UTFR_CAN_H

#include "FreeRTOS.h"
#include "can_types/common.h"
#include "can_types/front.h"
#include "can_types/inverter.h"
#include "can_types/rear.h"
#include "queue.h"
#include "utfr_hal.h"
#include "semphr.h"

#include <stdint.h>

#define MAX_NUM_FILTERS          28
#define CAN_IRQ_PREEMPT_PRIORITY 6
#define CAN_IRQ_SUBPRIORITY      0

#define MAX_STD_CAN_ID      0x7FF
#define MAX_EXT_CAN_ID      0x1FFFFFFF
#define NUM_CAN_CONTROLLERS 3

#define CAN_MAX_DATA_LENGTH 8

typedef enum { CAN1 = 0, CAN2 = 1, CAN3 = 2 } can_instance_t;

typedef enum {
  CAN1_RX_PA11,
  CAN1_RX_PB8,
  CAN2_RX_PB5,
  CAN2_RX_PB12,
#if !defined(UTFR_MCU_FAMILY_H7)
  // H7's dual-core FDCAN parts (H745/755) only have FDCAN1/FDCAN2.
  CAN3_RX_PA8,
#endif
} can_rx_t;

typedef enum {
  CAN1_TX_PA12,
  CAN1_TX_PB9,
  CAN2_TX_PB6,
  CAN2_TX_PB13,
#if !defined(UTFR_MCU_FAMILY_H7)
  CAN3_TX_PA15,
  CAN3_TX_PB4
#endif
} can_tx_t;

typedef uint32_t can_id_t;

_Static_assert(sizeof(rear_msg_id_t) == sizeof(uint32_t), "rear_msg_id_t wrong size");
_Static_assert(sizeof(front_msg_id_t) == sizeof(uint32_t), "front_msg_id_t wrong size");
_Static_assert(sizeof(inverter_msg_id_t) == sizeof(uint32_t), "inverter_msg_id_t wrong size");

typedef struct {
  uint8_t data[CAN_MAX_DATA_LENGTH];
  union {
    front_msg_id_t front_id;
    rear_msg_id_t rear_id;
    inverter_msg_id_t inverter_id;
    uint32_t raw_id;
  };
  uint8_t dlc;
  can_instance_t instance;
} can_msg_t;

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
  can_baudrate_t baudrate;
  QueueHandle_t tx_queue;
} can_t;

typedef struct {
  can_msg_t msg;
  bool immediate;
} can_msg_update_t;

typedef struct {
  can_msg_t msg;
  uint32_t cycle_time_ms;
  TickType_t last_tick;
} can_tx_msg_t;

HAL_StatusTypeDef can_init(can_t *can, const can_filter_t *filters, int num_filters,
                           uint32_t interrupts);
HAL_StatusTypeDef can_deinit(can_t *can);

void can_transmit(can_instance_t instance, can_msg_t *msg);

#endif
