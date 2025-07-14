/******************************************************************************
 * Class header for the UTFR Teensy CAN Library
 * ----------------------------------------------------------------------------
 *
 * This code is responsible for critical car functionality.
 * DO NOT modify it unless you're sure of your changes.
 *
 *****************************************************************************/

/******************************************************************************
 *                              I N C L U D E S                               *
 *****************************************************************************/
#include "UTFR_CAN_TEENSY_TYPES.hpp"

/******************************************************************************
 *                              D E F I N E S                                 *
 *****************************************************************************/

#ifndef _UTFR_CAN_TEENSY_H_ // AUTOGEN
#define _UTFR_CAN_TEENSY_H_

UTFR_TCAN_CLASS class UTFR_CAN_TEENSY {
  /******************************************************************************
   *          P U B L I C   D A T A   D E C L A R A T I O N S *
   ******************************************************************************/
public:
  FlexCAN_T4<_bus, _rxSize, _txSize>
      _node; // _bus, _rxSize, _txSize passed when generating this class from
             // its template

  String _nodeName;     // Passed in constructor, used primarily in debug prints
  uint32_t _busSpeed;   // Passed in constructor
  CAN_msg_t *_msgArray; // Passed in constructor - Array of messages along with
                        // signal data
  void (*_rxCallback)(const CAN_message_t &); // Passed in constructor

  CAN_errors_e _nodeError =
      CAN_OK_ERROR; // Set to non-zero value when an error occurs

  /******************************************************************************
   *          P U B L I C   F U N C T I O N   D E C L A R A T I O N S *
   ******************************************************************************/

  UTFR_CAN_TEENSY(String nodeName, int busSpeed, CAN_msg_t *msgArray,
                  void (*rxCallback)(const CAN_message_t &)); // Constructor

  void begin(void); // Initialize CAN node
  void print_errors();
  int send(msg_e msgName); // Send message by name
  void setFilters(void);   // set masks and filters according to CAN_filterArray
  void setFiltersAcceptAll(void); // set masks and filters to receive ALL
                                  // messages sent by other nodes
  void setFiltersRejectAll(void); // set masks and filters to receive NO
                                  // messages sent by other nodes
  int64_t getSignalRaw(msg_e msgName,
                       sig_e signalName); // Get signal data by name without
                                          // scale, offset, or unit
  int64_t
  getSignalRawBE(msg_e msgName,
                 sig_e signalName); // Get big endian signal data by name
                                    // without scale, offset, or unit
  float getSignal(msg_e msgName,
                  sig_e signalName); // Get signal data by name with scale,
                                     // offset, and TO DO: unit
  float getSignalBE(msg_e msgName,
                    sig_e signalName); // Get big endian signal data by name
                                       // with scale, offset, and TO DO: unit
  void setSignal(msg_e msgName, sig_e signalName,
                 float signalData); // Set signal data by name
  void setSignalRaw(msg_e msgName, sig_e signalName, int64_t signalData);
  // void sendError(errorNames_e error); // TO DO: Sends error message to
  // datalogger.
  void printMsgData(msg_e msgName); // Prints all data stored in a given
                                    // message (in decimal format)
};

#endif