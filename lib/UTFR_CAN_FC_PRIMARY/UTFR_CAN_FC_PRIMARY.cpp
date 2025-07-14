#include <UTFR_CAN_FC_PRIMARY.hpp>

CAN_msg_t msg_array_fc_primary[(int)msg_fc_primary_e::COUNT] =
  {[(int)msg_fc_primary_e::PACK_STATS] = {
     .msg = {.id = 0x123, .timestamp = 0, .idhit = 0, .flags = {.extended = 0}},
     .signals =
       {
         [(int)sig_fc_primary_e::TEMP] =
           {
             .startBit = 0,
             .len = 8,
             .littleEndian = true,
             .sign = true,
             .scale = 1.0,
             .offset = 0.0,
             .min = 0.0,
             .max = 256.0,
           },
         [(int)sig_fc_primary_e::ENERGY] =
           {
             .startBit = 8,
             .len = 16,
             .littleEndian = true,
             .sign = false,
             .scale = 1.0,
             .offset = 0.0,
             .min = 0.0,
             .max = 65535,
           }
       }
   }};
