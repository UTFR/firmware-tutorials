/******************************************************************************
 * Types + config header for the UTFR Teensy CAN Library
 * ----------------------------------------------------------------------------
 *
 * This code is responsible for critical car functionality.
 * DO NOT modify it unless you're sure of your changes.
 *
 * This file is common across all Teensy 4.1 CAN nodes on the car.
 *
 *  -> Simpler to maintain/write an AUTOGEN script for it this way.
 *  -> If configured for the node with highest load in system, should work
 *     without issue on every other node.
 *
 *****************************************************************************/

#ifndef _UTFR_CAN_TEENSY_TYPES_H_
#define _UTFR_CAN_TEENSY_TYPES_H_

// ==================== INCLUDES ====================================
#include <FlexCAN_T4.h>

// ==================== USER CONFIG =================================
// #define CAN_debugMode                           // Uncomment this line to
// enable debug prints.

#define TOTAL_MB_COUNT 64
#define RX_MB_COUNT 48 // Change ratio of Rx mailboxes to Tx mailboxes here
#define TX_MB_COUNT                                                            \
  TOTAL_MB_COUNT -                                                             \
      RX_MB_COUNT // TO DO: Determine best ratio through bus sim with PCAN

#define RX_QUEUE_SIZE                                                          \
  RX_SIZE_256 // Defining sizes of circular buffers/queues (pure software
              // queues)
#define TX_QUEUE_SIZE                                                          \
  TX_SIZE_128 // TO DO: Decide these sizes through testing with PCAN

#define MAX_SIGNALS_ONE_MSG                                                    \
  64 // Max number of signals in any single message on the car
// ==================================================================

#define UTFR_TCAN_CLASS                                                        \
  template <CAN_DEV_TABLE _bus, typename msg_e, typename sig_e,                \
            FLEXCAN_RXQUEUE_TABLE _rxSize, FLEXCAN_TXQUEUE_TABLE _txSize>

struct CAN_signal_t // Default values for undefined signal case
{
  const uint8_t startBit = 0;
  uint8_t len = 8;
  const bool littleEndian = true; // true: little endian, false: big endian
  const bool sign = false;        // true: signed, false: unsigned
  const float scale = 1.0;
  const float offset = 0.0;
  const float min = 0.0;
  const float max = 10000.0;
  // const String unit = "C";                // TO DO: units library instead of
  // String for this const String receiver = "NA"; const String comment = " ";
};

struct CAN_msg_t     // Superstruct containing message from FlexCAN_T4.h plus
                     // signals contained
{                    // in that message
  CAN_message_t msg; // Message struct from FlexCAN_T4.h
  const CAN_signal_t
      signals[MAX_SIGNALS_ONE_MSG]; // Signals contained in this message
};

enum CAN_errors_e {
  CAN_OK_ERROR = 0,         // No error, code executed correctly
  CAN_INIT_BUS_SPEED_ERROR, // Invalid bus speed passed to constructor
};

#endif // _UTFR_CAN_TEENSY_TYPES_H_