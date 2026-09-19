#include "bare_metal_queue.h"
#include "utfr_hal.h"

#include <stdint.h>
#include <string.h>

#define MAX_QUEUES       4
#define QUEUE_POOL_BYTES 512

typedef struct {
  uint8_t storage[QUEUE_POOL_BYTES];
  size_t element_size;
  int capacity;
  int head;
  int tail;
  int count;
} queue_state_t;

static queue_state_t queues[MAX_QUEUES];
static int num_queues_created = 0;

queue_t queue_create(int num_elements, size_t element_size) {
  queue_t id = num_queues_created++;
  queue_state_t *q = &queues[id];

  q->element_size = element_size;
  q->capacity = QUEUE_POOL_BYTES / element_size;
  if (num_elements < q->capacity) { q->capacity = num_elements; }
  q->head = q->tail = q->count = 0;

  return id;
}

bool queue_send(queue_t queue, const void *data) {
  queue_state_t *q = &queues[queue];
  bool sent = false;

  __disable_irq();
  if (q->count < q->capacity) {
    memcpy(&q->storage[q->tail * q->element_size], data, q->element_size);
    q->tail = (q->tail + 1) % q->capacity;
    q->count++;
    sent = true;
  }
  __enable_irq();

  return sent;
}

bool queue_receive(queue_t queue, void *data) {
  queue_state_t *q = &queues[queue];
  bool received = false;

  __disable_irq();
  if (q->count > 0) {
    memcpy(data, &q->storage[q->head * q->element_size], q->element_size);
    q->head = (q->head + 1) % q->capacity;
    q->count--;
    received = true;
  }
  __enable_irq();

  return received;
}
