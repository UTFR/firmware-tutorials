#ifndef UTFR_UTILS_BENCH_STAT_H
#define UTFR_UTILS_BENCH_STAT_H
#include <stdint.h>
#define BENCH_HIST_BUCKETS 64U

// Struct for TV STM32 Characterization profiling
typedef struct {
  uint32_t count;
  uint32_t min_cyc;
  uint32_t max_cyc;
  uint64_t sum_cyc;
  uint32_t budget_cyc;
  uint32_t overruns;
  uint32_t bucket_width_cyc;
  uint32_t hist[BENCH_HIST_BUCKETS];
} bench_stat_t;

void bench_stat_init(bench_stat_t *s, uint32_t budget_cyc, uint32_t bucket_width_cyc);
void bench_stat_reset(bench_stat_t *s);
void bench_stat_add(bench_stat_t *s, uint32_t delta_cyc);
uint32_t bench_stat_percentile_cyc(const bench_stat_t *s, uint32_t permille);
void bench_stat_report(const bench_stat_t *s, const char *label, uint32_t cpu_mhz);

#endif // UTFR_UTILS_BENCH_STAT_H
