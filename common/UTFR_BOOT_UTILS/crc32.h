#ifndef UTFR_DFU_UTILS_CRC32_H
#define UTFR_DFU_UTILS_CRC32_H

#include <stddef.h>
#include <stdint.h>

uint32_t crc32(uint32_t crc, const void *buf, size_t size);

#endif
