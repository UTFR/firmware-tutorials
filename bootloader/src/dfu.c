#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "dfu.h"
#include "crc32.h"
#include "flash.h"
#include "memory_map.h"
#include "memory_map_config.h"
#include "shared_ram.h"
#include "stm32_hal_legacy.h"
#include "UTFR_UTILS/utils.h"
#include "utfr_hal.h"

#define CEIL(x, y) (((x) + (y) - 1) / (y))
#define CHUNK_SIZE 8
#define BLOCK_SIZE 2048

static inline void send_ack(dfu_ctx_t *ctx);
static inline void send_nack(dfu_ctx_t *ctx);
static void handle_image_header(dfu_ctx_t *ctx);
static void handle_firmware_data(dfu_ctx_t *ctx);
static void handle_firmware_data_block(dfu_ctx_t *ctx, can_msg_t *firmware_data_msg, uint8_t *data);
static void handle_firmware_data_finish(dfu_ctx_t *ctx);

void do_dfu(dfu_ctx_t *ctx) {
  printf("doing dfu...\n\r");
  send_ack(ctx);
  handle_image_header(ctx);
  handle_firmware_data(ctx);
}

static void handle_image_header(dfu_ctx_t *ctx) {
  can_msg_t header_info_msg;
  ctx->can_recv(&header_info_msg);
  assert_param(header_info_msg.id == BOOTLOADER_CMD_IMAGE_HEADER);
  assert_param(header_info_msg.dlc == 7);
  ctx->header->version_major = header_info_msg.data[0];
  ctx->header->version_minor = header_info_msg.data[1];
  ctx->header->version_patch = header_info_msg.data[2];
  ctx->header->vector_addr = readu32_le(&header_info_msg.data[3]);
  printf("app version: %d.%d.%d\n\rvector address: %p\n\r", ctx->header->version_major,
         ctx->header->version_minor, ctx->header->version_patch, (void *)ctx->header->vector_addr);
  send_ack(ctx);

  ctx->can_recv(&header_info_msg);
  assert_param(header_info_msg.id == BOOTLOADER_CMD_IMAGE_HEADER);
  assert_param(header_info_msg.dlc == 8);
  memcpy(ctx->header->git_sha, header_info_msg.data, 8);
  send_ack(ctx);
  printf("git sha: %.8s\n\r", ctx->header->git_sha);

  ctx->can_recv(&header_info_msg);
  assert_param(header_info_msg.id == BOOTLOADER_CMD_IMAGE_HEADER);
  assert_param(header_info_msg.dlc == 8);
  ctx->header->data_size = readu32_le(header_info_msg.data);
  ctx->header->crc = readu32_le(header_info_msg.data + 4);
  send_ack(ctx);
  printf("data size: %ld\n\rcrc: 0x%lX\n\r", ctx->header->data_size, ctx->header->crc);

  ctx->header->magic = IMAGE_MAGIC;
}

static void handle_firmware_data(dfu_ctx_t *ctx) {
  uint8_t data[BLOCK_SIZE];
  can_msg_t msg;
  while (1) {
    ctx->can_recv(&msg);
    if (msg.id == BOOTLOADER_CMD_FIRMWARE_DATA) {
      handle_firmware_data_block(ctx, &msg, data);
    } else if (msg.id == BOOTLOADER_CMD_FIRMWARE_DATA_FINISH) {
      handle_firmware_data_finish(ctx);
      break;
    } else {
      break;
    }
  }
}

static void handle_firmware_data_block(dfu_ctx_t *ctx, can_msg_t *firmware_data_msg,
                                       uint8_t *data) {
  static bool erased = false;
  assert_param(firmware_data_msg->id == BOOTLOADER_CMD_FIRMWARE_DATA);
  assert_param(firmware_data_msg->dlc == 6);
  uint32_t write_address = readu32_le(firmware_data_msg->data);
  uint16_t num_bytes = readu16_le(&firmware_data_msg->data[4]) + 1;
  send_ack(ctx);

  // printf("client requesting %d bytes to %p\n\r", num_bytes, (void *)write_address);

  ctx->can_recv(firmware_data_msg);
  assert_param(firmware_data_msg->id == BOOTLOADER_CMD_FIRMWARE_DATA);
  assert_param(firmware_data_msg->dlc == 4);
  uint32_t expected_crc = readu32_le(firmware_data_msg->data);
  send_ack(ctx);
  // printf("block CRC: 0x%lX\n\r", expected_crc);

  int size = 0;
  for (int i = 0; i < CEIL(num_bytes, CHUNK_SIZE); i++) {
    ctx->can_recv(firmware_data_msg);
    assert_param(firmware_data_msg->id == BOOTLOADER_CMD_FIRMWARE_DATA);
    memcpy(data + size, firmware_data_msg->data, firmware_data_msg->dlc);
    size += firmware_data_msg->dlc;
  }
  assert_param(num_bytes == size);

  uint32_t calculated_crc = crc32(0, data, size);

  if (calculated_crc != expected_crc) {
    printf("crc mismatch in data block. expected 0x%lX, got 0x%lX\n\r", expected_crc,
           calculated_crc);
    send_nack(ctx);
    return;
  }

  uint32_t app_start = APP_SECTION_START;
  // printf("write address: %p\n\r app_start: %p\n\r", (void *)write_address, (void *)app_start);
  if (!erased) {
    uint32_t flash_end = FLASH_BASE + FLASH_SIZE; // e.g. 0x08080000 for 512KB
    uint32_t erase_size = flash_end - app_start;

    int page = (app_start - FLASH_BASE) / PAGESIZE;
    int num_pages = erase_size / PAGESIZE;

    // int page = (app_start - FLASH_BASE) / PAGESIZE;
    // int num_pages = CEIL(ctx->header->data_size, PAGESIZE);
    printf("erasing pages [%d-%d]\n\r", page, page + num_pages);
    HAL_FLASH_Unlock();
    if (flash_erase(page, num_pages) != HAL_OK) { printf("failed to erase\n\r"); }
    HAL_FLASH_Lock();
    erased = true;
  }

  // printf("writing %d bytes to %p\n\r", size, (void *)write_address);
  HAL_FLASH_Unlock();
  flash_write(write_address, data, size);
  HAL_FLASH_Lock();
  send_ack(ctx);
}

static void handle_firmware_data_finish(dfu_ctx_t *ctx) {
  printf("writing image header to flash:\n\r");
  printf("magic: %d\n\r", ctx->header->magic);
  printf("crc: %ld\n\r", ctx->header->crc);
  printf("data size: %ld\n\r", ctx->header->data_size);
  printf("image kind: %d\n\r", ctx->header->image_kind);
  printf("version: %d.%d.%d\n\r", ctx->header->version_major, ctx->header->version_minor,
         ctx->header->version_patch);
  printf("vector addr: %p\n\r", (void *)ctx->header->vector_addr);
  printf("git sha: %.8s\n\r", ctx->header->git_sha);

  switch (ctx->header->image_kind) {
  case IMAGE_KIND_APP: image_write_header(IMAGE_SLOT_APP, ctx->header); break;
  default:
    printf("invalid image kind %d\n\rskipping writing image header\n\r", ctx->header->image_kind);
    break;
  }

  shared_ram_reset_flag(SHARED_RAM_FLAG_DFU_REQUESTED);
}

static inline void send_ack(dfu_ctx_t *ctx) {
  static const uint8_t dummy = 0;
  ctx->can_send(BOOTLOADER_CMD_ACK, &dummy, 0);
}

static inline void send_nack(dfu_ctx_t *ctx) { ctx->can_send(BOOTLOADER_CMD_NACK, NULL, 0); }
