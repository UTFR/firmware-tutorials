#include <string.h>

#include "utils.h"

void write_u32(uint8_t *buf, uint32_t data) {
#ifdef __BIG_ENDIAN__
  data = __builtin_bswap32(data);
#endif
  memcpy(buf, &data, sizeof(uint32_t));
}

uint32_t readu32_le(const uint8_t *buf) {
  uint32_t ret;
  memcpy(&ret, buf, sizeof(uint32_t));
#ifdef __BIG_ENDIAN__
  ret = __builtin_bswap32(ret);
#endif
  return ret;
}

uint32_t readu16_le(const uint8_t *buf) {
  uint16_t ret;
  memcpy(&ret, buf, sizeof(uint16_t));
#ifdef __BIG_ENDIAN__
  ret = __builtin_bswap16(ret);
#endif
  return ret;
}

uint64_t readu64_le(const uint8_t *buf) {
  uint64_t ret;
  memcpy(&ret, buf, sizeof(uint64_t));
#ifdef __BIG_ENDIAN__
  ret = __builtin_bswap64(ret);
#endif
  return ret;
}
