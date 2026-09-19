#ifndef UTFR_BOOT_UTILS_FLASH_H
#define UTFR_BOOT_UTILS_FLASH_H

#include <stdint.h>
#include "utfr_hal.h"

HAL_StatusTypeDef flash_erase(uint32_t page, uint32_t num_pages);
HAL_StatusTypeDef flash_write(uint32_t address, const uint8_t *data, uint16_t size);

#endif
