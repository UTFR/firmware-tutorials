#include <stdint.h>
#include <stdio.h>

#include "image.h"
#include "boot.h"
#include "memory_map.h"
#include "crc32.h"
#include "memory_map_config.h"
#include "vector.h"
#include "flash.h"

const image_header_t *image_get_and_validate_header(image_slot_t slot) {
  const image_header_t *header = image_get_header(slot);
  if (header == NULL) { return NULL; }
  if (!image_valid(slot, header)) { return NULL; }
  return header;
}

const image_header_t *image_get_header(image_slot_t slot) {
  const image_header_t *header = NULL;

  switch (slot) {
  case IMAGE_SLOT_BOOTLOADER: header = (const image_header_t *)BOOTLOADER_SECTION_START; break;
  case IMAGE_SLOT_APP:        header = (const image_header_t *)APP_SECTION_START; break;
  default:                    break;
  }

  // printf("header: %p\n", (void *)header);

  if (header && header->magic == IMAGE_MAGIC) {
    return header;
  } else {
    // printf("header magic invalid\n");
    return NULL;
  }
}

bool image_valid(image_slot_t slot, const image_header_t *header) {
  uint8_t *addr
    = (slot == IMAGE_SLOT_APP ? (uint8_t *)APP_SECTION_START : (uint8_t *)BOOTLOADER_SECTION_START);
  addr += sizeof(image_header_t);
  uint32_t len = header->data_size;
  uint32_t a = crc32(0, addr, len);
  uint32_t b = header->crc;
  return a == b;
}

void image_write_header(image_slot_t slot, const image_header_t *header) {
  uint32_t addr = (uint32_t)(slot == IMAGE_SLOT_APP ? APP_SECTION_START : BOOTLOADER_SECTION_START);
  flash_write(addr, (const uint8_t *const)header, sizeof(image_header_t));
}

#ifdef BOOTLOADER_IMAGE
extern void bootloader_image_deinit(void);
#endif

__attribute__((noreturn)) void image_start(const image_header_t *header) {
  __disable_irq();

  HAL_RCC_DeInit();
  SysTick->CTRL = 0;

#ifdef BOOTLOADER_IMAGE
  bootloader_image_deinit();
#endif

  __enable_irq();

  const vector_table_t *vector_table = (const vector_table_t *)header->vector_addr;
  vtor_init(header->vector_addr);
  __set_MSP((uint32_t)vector_table->initial_sp);
  vector_table->reset();
  __builtin_unreachable();
}
