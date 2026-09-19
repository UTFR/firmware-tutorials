#ifndef UTFR_BOOT_UTILS_IMAGE_H
#define UTFR_BOOT_UTILS_IMAGE_H

#include <stdbool.h>
#include <stdint.h>

#define IMAGE_MAGIC  0x0894
#define GIT_SHA_SIZE 8

typedef enum {
  IMAGE_KIND_BOOTLOADER = 0x1,
  IMAGE_KIND_APP = 0x2,
} image_kind_t;

typedef enum {
  IMAGE_SLOT_BOOTLOADER = 1,
  IMAGE_SLOT_APP = 2,
} image_slot_t;

typedef enum {
  IMAGE_VERSION_1 = 1,
  IMAGE_VERSION_CURRENT = IMAGE_VERSION_1,
} image_version_t;

typedef struct __attribute__((packed)) image_header_t {
  uint16_t magic;
  uint32_t crc;
  uint32_t data_size;
  uint8_t image_kind;
  uint8_t version_major;
  uint8_t version_minor;
  uint8_t version_patch;
  uint32_t vector_addr;
  uint32_t reserved;
  uint8_t git_sha[GIT_SHA_SIZE] __attribute__((nonstring));
} image_header_t;

const image_header_t *image_get_and_validate_header(image_slot_t slot);
const image_header_t *image_get_header(image_slot_t slot);
bool image_valid(image_slot_t slot, const image_header_t *header);
void image_start(const image_header_t *header);
void image_write_header(image_slot_t slot, const image_header_t *header);

#endif
