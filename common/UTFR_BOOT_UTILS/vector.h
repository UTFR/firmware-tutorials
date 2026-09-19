#ifndef UTFR_BOOT_UTILS_VECTOR_H
#define UTFR_BOOT_UTILS_VECTOR_H

#define NVIC_IRQ_COUNT         239
#define VECTOR_TABLE_ALIGNMENT 512

typedef void (*vector_table_entry_t)(void);

typedef struct __attribute__((packed)) {
  vector_table_entry_t initial_sp;
  vector_table_entry_t reset;
  vector_table_entry_t nmi;
  vector_table_entry_t hard_fault;
  vector_table_entry_t memory_manage_fault;
  vector_table_entry_t bus_fault;
  vector_table_entry_t usage_fault;
  vector_table_entry_t reserved_x001c[4];
  vector_table_entry_t svc;
  vector_table_entry_t debug_monitor;
  vector_table_entry_t reserved_x0034;
  vector_table_entry_t pend_sv;
  vector_table_entry_t systick;
  vector_table_entry_t irq[NVIC_IRQ_COUNT];
} vector_table_t;

extern vector_table_t vector_table;

#endif
