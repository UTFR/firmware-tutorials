#ifndef BOOTLOADER_DFU_H
#define BOOTLOADER_DFU_H

#include "can.h"
#include "UTFR_BOOT_UTILS/image.h"

typedef enum {
  BOOTLOADER_CMD_ACK = 0x100,
  BOOTLOADER_CMD_NACK = 0x101,
  BOOTLOADER_CMD_IMAGE_HEADER = 0x102,
  BOOTLOADER_CMD_FIRMWARE_DATA = 0x103,
  BOOTLOADER_CMD_FIRMWARE_DATA_FINISH = 0x104,
} bootloader_cmd_e;

typedef struct {
  void (*can_recv)(can_msg_t *msg);
  void (*can_send)(uint32_t id, const uint8_t *payload, uint32_t dlc);
  image_header_t *header;
} dfu_ctx_t;

void do_dfu(dfu_ctx_t *ctx);

#endif
