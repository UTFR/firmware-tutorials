#include "bare_metal_mutex.h"
#include "utfr_hal.h"

#define MAX_MUTEXES 8

static volatile bool locked[MAX_MUTEXES];
static int num_mutexes_created = 0;

mutex_t mutex_create(void) {
  mutex_t id = num_mutexes_created++;
  locked[id] = false;
  return id;
}

bool mutex_take(mutex_t mutex) {
  bool acquired = false;
  __disable_irq();
  if (!locked[mutex]) {
    locked[mutex] = true;
    acquired = true;
  }
  __enable_irq();
  return acquired;
}

bool mutex_give(mutex_t mutex) {
  __disable_irq();
  locked[mutex] = false;
  __enable_irq();
  return true;
}
