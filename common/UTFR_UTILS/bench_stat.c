#include "bench_stat.h"

#include <stdio.h>

void bench_stat_init(bench_stat_t *s, uint32_t budget_cyc, uint32_t bucket_width_cyc) {
  s->budget_cyc = budget_cyc;
  s->bucket_width_cyc = (bucket_width_cyc == 0U) ? 1U : bucket_width_cyc;
  bench_stat_reset(s);
}

void bench_stat_reset(bench_stat_t *s) {
  s->count = 0U;
  s->min_cyc = UINT32_MAX;
  s->max_cyc = 0U;
  s->sum_cyc = 0U;
  s->overruns = 0U;
  for (uint32_t i = 0U; i < BENCH_HIST_BUCKETS; i++) { s->hist[i] = 0U; }
}

void bench_stat_add(bench_stat_t *s, uint32_t delta_cyc) {
  s->count++;
  s->sum_cyc += delta_cyc;
  if (delta_cyc < s->min_cyc) { s->min_cyc = delta_cyc; }
  if (delta_cyc > s->max_cyc) { s->max_cyc = delta_cyc; }
  if (s->budget_cyc != 0U && delta_cyc >= s->budget_cyc) { s->overruns++; }

  uint32_t bucket = delta_cyc / s->bucket_width_cyc;
  if (bucket >= BENCH_HIST_BUCKETS) { bucket = BENCH_HIST_BUCKETS - 1U; }
  s->hist[bucket]++;
}

uint32_t bench_stat_percentile_cyc(const bench_stat_t *s, uint32_t permille) {
  if (s->count == 0U) { return 0U; }
  if (permille > 1000U) { permille = 1000U; }

  // Smallest cumulative count that still covers the requested fraction.
  uint64_t threshold = ((uint64_t)s->count * permille + 999U) / 1000U;
  if (threshold == 0U) { threshold = 1U; }

  uint64_t cumulative = 0U;
  for (uint32_t i = 0U; i < BENCH_HIST_BUCKETS; i++) {
    cumulative += s->hist[i];
    if (cumulative >= threshold) {
      uint32_t edge = (i + 1U) * s->bucket_width_cyc;
      return (edge > s->max_cyc) ? s->max_cyc : edge;
    }
  }
  return s->max_cyc;
}

void bench_stat_report(const bench_stat_t *s, const char *label, uint32_t cpu_mhz) {
  if (cpu_mhz == 0U) { cpu_mhz = 1U; }

  if (s->count == 0U) {
    printf("%s: n=0 (no samples this window)\r\n", label);
    return;
  }

  uint32_t mean_cyc = (uint32_t)(s->sum_cyc / s->count);
  uint32_t p99_cyc = bench_stat_percentile_cyc(s, 990U);

  printf(
    "%s: n=%lu  min/mean/max/p99 = %lu/%lu/%lu/%lu us  (%lu/%lu/%lu/%lu cyc)  overruns=%lu\r\n",
    label, (unsigned long)s->count, (unsigned long)(s->min_cyc / cpu_mhz),
    (unsigned long)(mean_cyc / cpu_mhz), (unsigned long)(s->max_cyc / cpu_mhz),
    (unsigned long)(p99_cyc / cpu_mhz), (unsigned long)s->min_cyc, (unsigned long)mean_cyc,
    (unsigned long)s->max_cyc, (unsigned long)p99_cyc, (unsigned long)s->overruns);
}
