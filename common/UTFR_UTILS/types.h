#ifndef UTFR_UTILS_TYPES_H
#define UTFR_UTILS_TYPES_H

typedef int utfr_status_t;

#define UTFR_ERR (-1)
#define UTFR_OK  1

// typedef enum {
//   UTFR_ERR_CAN_WD_EXPIRED,
//   NUM_UTFR_ERR,
// } utfr_err_kind_t;

// typedef struct {
//   utfr_err_kind_t kind;
//   union {

//   };
// } utfr_err_t;

typedef enum {
  AMS_FAULT_,
  AMS_FAULT_IMD,
} ams_fault_t;

#endif
