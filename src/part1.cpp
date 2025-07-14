#include "UTFR_CAN_FC_PRIMARY.hpp"
#include "UTFR_CAN_TEENSY.hpp"
#include "UTFR_CAN_TEENSY.tpp"
#include <Arduino.h>
#include <arduino_freertos.h>
#include <cstdint>
#include "queue.h"
#include "semphr.h"

static void pcan_rx_callback(const CAN_message_t &message);

QueueHandle_t can_rx_queue;

SemaphoreHandle_t pack_stats_mutex;
uint8_t temp;
uint16_t energy;

#define PCAN_BAUDRATE 1000000 /*TODO*/

UTFR_CAN_TEENSY<
  CAN2, msg_fc_primary_e, sig_fc_primary_e, RX_QUEUE_SIZE, TX_QUEUE_SIZE>
  primary_can(
    "primary_can", PCAN_BAUDRATE, msg_array_fc_primary, &pcan_rx_callback
  );

void loop() {}

/*
Part 1:

Read pack energy and temperature
Display it on the LCD
*/

void setup() {
  Serial.begin(9600);

  can_rx_queue = xQueueCreate(256, sizeof(CAN_message_t));
  pack_stats_mutex = xSemaphoreCreateMutex();
}

void can_task(void) {
  CAN_message_t msg;
  for(;;) {
    xQueueReceive(can_rx_queue, &msg, portMAX_DELAY);

    switch(msg.id) {
    case 0x123:
      xSemaphoreTake(pack_stats_mutex, 0);
      temp = primary_can.getSignal(
        msg_fc_primary_e::PACK_STATS, sig_fc_primary_e::TEMP
      );
      energy = primary_can.getSignal(
        msg_fc_primary_e::PACK_STATS, sig_fc_primary_e::ENERGY
      );
      xSemaphoreGive(pack_stats_mutex);
      break;
    default:
      break;
    }
  }
}

void lcd_task(void) {
  for(;;) {
    xSemaphoreTake(pack_stats_mutex, portMAX_DELAY);

    // temp, energy

    xSemaphoreGive(pack_stats_mutex);
  }
}

void pcan_rx_callback(const CAN_message_t &message) {
  xQueueSendFromISR(can_rx_queue, &message, 0);
}
