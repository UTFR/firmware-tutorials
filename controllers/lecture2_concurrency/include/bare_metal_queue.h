#ifndef LECTURE2_CONCURRENCY_BARE_METAL_QUEUE_H
#define LECTURE2_CONCURRENCY_BARE_METAL_QUEUE_H

#include <stdbool.h>
#include <stddef.h>

typedef int queue_t;

// A minimal, genuinely-working bare-metal ring buffer guarded by
// __disable_irq()/__enable_irq() critical sections -- a toy, like
// bare_metal_mutex.h. Starting next lecture you'll use the real
// xQueueCreate()/xQueueSend()/xQueueReceive().
queue_t queue_create(int num_elements, size_t element_size);
bool queue_send(queue_t queue, const void *data);
bool queue_receive(queue_t queue, void *data);

#endif
