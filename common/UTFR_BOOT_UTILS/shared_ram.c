#include <stdbool.h>
#include <string.h>
#include <assert.h>

#include "cmsis_gcc.h"
#include "memory_map.h"
#include "shared_ram.h"

typedef struct __attribute__((packed)) {
  uint32_t magic;
  uint32_t flags;
  uint8_t boot_count;
} shared_ram_t;

static_assert(sizeof(shared_ram_t) <= SHARED_RAM_SECTION_SIZE,
              "sizeof shared_ram_t exceeded 256B allocated in memory map");

SHARED_RAM shared_ram_t shared_ram;

void shared_ram_init(void) {
  if (shared_ram.magic != SHARED_RAM_MAGIC) {
    memset(&shared_ram, 0, sizeof(shared_ram_t));
    shared_ram.magic = SHARED_RAM_MAGIC;
  }
}

void shared_ram_invalidate(void) {
  shared_ram.magic = 0;
  __DMB(); // be certain this change has taken effect before proceeding
}

bool shared_ram_is_flag_set(shared_ram_flag_t flag) { return (shared_ram.flags & flag) != 0; }

void shared_ram_set_flag(shared_ram_flag_t flag) { shared_ram.flags |= flag; }

void shared_ram_set_controller(uint32_t controller) {
  shared_ram.flags = (shared_ram.flags & ~0xE) | ((controller << 1) & 0xE);
}

uint32_t shared_ram_get_controller(void) { return (shared_ram.flags & 0xE) >> 1; }

void shared_ram_reset_controller(void) { shared_ram.flags &= ~(0xE); }

void shared_ram_reset_flag(shared_ram_flag_t flag) { shared_ram.flags &= ~flag; }

void shared_ram_increment_boot_count(void) { shared_ram.boot_count++; }
void shared_ram_clear_boot_count(void) { shared_ram.boot_count = 0; }
uint8_t shared_ram_get_boot_count(void) { return shared_ram.boot_count; }
