#ifndef UTFR_CAN_FC_PRIMARY_H
#define UTFR_CAN_FC_PRIMARY_H
#include "UTFR_CAN_TEENSY_TYPES.hpp"

enum class msg_fc_primary_e {
  PACK_STATS,

  COUNT
};

enum class sig_fc_primary_e {
  ENERGY,
  TEMP,

  COUNT
};

extern CAN_msg_t msg_array_fc_primary[(int)msg_fc_primary_e::COUNT];

#endif
