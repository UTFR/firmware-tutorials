#ifndef UTFR_UTILS_H
#define UTFR_UTILS_H

#include <stdint.h>

#define ALIGN(x, a)         ALIGN_MASK(x, (typeof(x))(a) - 1)
#define ALIGN_MASK(x, mask) (((x) + (mask)) & ~(mask))

#define CEIL_DIV(x, y) (((x) + (y) - 1) / (y))

#define LIKELY(x)   __builtin_expect(!!(x), 1)
#define UNLIKELY(x) __builtin_expect(!!(x), 0)

#define IS_ISR() (__get_IPSR() != 0U)

#define ARRAY_SIZE(arr) (sizeof((arr)) / sizeof((arr)[0]))

#ifndef UNUSED
  #define UNUSED(x) ((void)(x))
#endif

#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))

#define IS_BIG_ENDIAN                                                                              \
  ((defined(__BYTE_ORDER__) && (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__)) || __BIG_ENDIAN__         \
   || __BIG_ENDIAN || _BIG_ENDIAN)

_Static_assert(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__, "compiling for little endian target");
_Static_assert(__BYTE_ORDER__ != __ORDER_BIG_ENDIAN__, "cannot detect endianness");

extern void assert_handler(const char *expr, const char *file, uint32_t line);

#ifdef DO_UTFR_ASSERTS
  #define UTFR_ASSERT(EXPR) ((EXPR) ? (void)0 : assert_handler(#EXPR, __FILE__, __LINE__))
#else
  #define UTFR_ASSERT(EXPR)
#endif

void write_u32(uint8_t *buf, uint32_t data);
uint32_t readu16_le(const uint8_t *buf);
uint32_t readu32_le(const uint8_t *buf);
uint64_t readu64_le(const uint8_t *buf);

#endif
