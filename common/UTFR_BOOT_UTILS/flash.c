#include <stdint.h>
#include <string.h>

#include "flash.h"
#include "utfr_hal.h"
#include "UTFR_UTILS/utils.h"

#define FLASH_PAGES_PER_BANK (FLASH_SIZE / FLASH_PAGE_SIZE / 2)
// Named to avoid colliding with CMSIS's own FLASH_END (stm32h755xx.h etc.),
// which is the chip's total physical flash end, not this board's configured
// FLASH_SIZE region (e.g. H7dev only claims the CM7's 1 MB bank out of the
// chip's real 2 MB -- the two are deliberately different values on H7).
#define UTFR_FLASH_REGION_END (FLASH_BASE + FLASH_SIZE)

#if defined(UTFR_MCU_FAMILY_H7)
// H7's flash HAL is structurally different from G4's, not just renamed:
// sector-based erase (FLASH_TYPEERASE_SECTORS/.Sector/.NbSectors, not
// page-based) with a per-bank Banks selector, and 256-bit/32-byte
// "flash word" program granularity (FLASH_TYPEPROGRAM_FLASHWORD) instead of
// G4's 64-bit doubleword. H7dev never calls these (pure GPIO blink, no
// DFU/params writes), so rather than translate write-path logic that can't
// be exercised or verified here, these are explicit stubs -- implement
// properly (mirroring flash_erase/flash_write's G4 shape, sector/bank math
// and FLASHWORD-sized writes) before any H7 board actually needs DFU or
// flash-backed params.
HAL_StatusTypeDef flash_erase(uint32_t page, uint32_t num_pages) {
  UNUSED(page);
  UNUSED(num_pages);
  return HAL_ERROR;
}

HAL_StatusTypeDef flash_write(uint32_t address, const uint8_t *data, uint16_t size) {
  UNUSED(address);
  UNUSED(data);
  UNUSED(size);
  return HAL_ERROR;
}
#else
HAL_StatusTypeDef flash_erase(uint32_t page, uint32_t num_pages) {
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

  HAL_StatusTypeDef status;
  uint32_t page_error;

  while (num_pages > 0) {
    FLASH_EraseInitTypeDef erase_init = {0};

    erase_init.TypeErase = FLASH_TYPEERASE_PAGES;

    if (page < FLASH_PAGES_PER_BANK) {
      // BANK 1
      erase_init.Banks = FLASH_BANK_1;
      erase_init.Page = page;

      uint32_t pages_left_in_bank = FLASH_PAGES_PER_BANK - page;
      erase_init.NbPages = (num_pages < pages_left_in_bank) ? num_pages : pages_left_in_bank;
    } else {
      // BANK 2
      erase_init.Banks = FLASH_BANK_2;
      erase_init.Page = page - FLASH_PAGES_PER_BANK;

      uint32_t pages_left_in_bank = FLASH_PAGES_PER_BANK - erase_init.Page;
      erase_init.NbPages = (num_pages < pages_left_in_bank) ? num_pages : pages_left_in_bank;
    }

    status = HAL_FLASHEx_Erase(&erase_init, &page_error);
    if (status != HAL_OK) return status;

    page += erase_init.NbPages;
    num_pages -= erase_init.NbPages;
  }

  return HAL_OK;
}

/**
 * @brief Write data to flash memory at a certain starting address
 * @warning address must be 8 byte aligned
 */
HAL_StatusTypeDef flash_write(uint32_t address, const uint8_t *data, uint16_t size) {
  assert_param((address & 0x7) == 0);
  assert_param(size >= sizeof(uint64_t));

  if (address >= UTFR_FLASH_REGION_END) { return HAL_ERROR; }

  HAL_StatusTypeDef status = HAL_OK;
  for (unsigned i = 0; i < size - sizeof(uint64_t); i += sizeof(uint64_t)) {
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address + i, readu64_le(data + i));
  }

  // pad the final doubleword if necessary
  uint8_t num_bytes_before_padding = size % sizeof(uint64_t);
  uint32_t final_addr_offset = size - sizeof(uint64_t);
  const uint8_t *final_data_buf = data + final_addr_offset;
  // Must live for the rest of the function (final_data_buf may point into
  // it below) -- declaring it inside the if-block below made it go out of
  // scope while still referenced, hence -Wdangling-pointer.
  uint8_t buf[sizeof(uint64_t)] = {0};
  if (num_bytes_before_padding != 0) {
    memcpy(buf, data + final_addr_offset, num_bytes_before_padding);
    final_data_buf = buf;
  }

  HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address + final_addr_offset,
                    readu64_le(final_data_buf));

  return status == 0 ? HAL_OK : HAL_ERROR;
}
#endif
