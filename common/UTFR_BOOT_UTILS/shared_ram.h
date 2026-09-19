#ifndef UTFR_BOOT_UTILS_SHARED_RAM_H
#define UTFR_BOOT_UTILS_SHARED_RAM_H

#include <stdint.h>
#include <stdbool.h>
#include "memory_map.h"

#define SHARED_RAM       __attribute__((section(SHARED_RAM_SECTION_NAME_STR)))
#define SHARED_RAM_MAGIC 0x0894

typedef enum {
  SHARED_RAM_FLAG_DFU_REQUESTED = (1 << 0),
} shared_ram_flag_t;

void shared_ram_init(void);
void shared_ram_invalidate(void);
bool shared_ram_is_flag_set(shared_ram_flag_t flag);
void shared_ram_set_flag(shared_ram_flag_t flag);
void shared_ram_reset_flag(shared_ram_flag_t flag);
void shared_ram_increment_boot_count(void);
void shared_ram_clear_boot_count(void);
uint8_t shared_ram_get_boot_count(void);
void shared_ram_set_controller(uint32_t controller);
void shared_ram_reset_controller(void);
uint32_t shared_ram_get_controller(void);

#endif
