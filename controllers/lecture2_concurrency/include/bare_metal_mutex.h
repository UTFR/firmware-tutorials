#ifndef LECTURE2_CONCURRENCY_BARE_METAL_MUTEX_H
#define LECTURE2_CONCURRENCY_BARE_METAL_MUTEX_H

#include <stdbool.h>

typedef int mutex_t;

// A minimal, genuinely-working bare-metal mutex: __disable_irq()/
// __enable_irq() critical sections instead of a scheduler-aware primitive.
// This is a toy -- starting next lecture (controllers/lecture3_freertos)
// you'll use the real thing, xSemaphoreCreateMutex().
mutex_t mutex_create(void);
bool mutex_take(mutex_t mutex);
bool mutex_give(mutex_t mutex);

#endif
