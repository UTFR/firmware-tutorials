#ifndef UTFR_SPI_DRIVER_H
#define UTFR_SPI_DRIVER_H

#include "utfr_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"

/**
 * @brief   Use as cs_pin when the peripheral drives NSS in hardware, or when the
 *          caller manages chip select itself.
 *
 * Zero is a valid index into pins[], so "no chip select" needs its own sentinel.
 */
#define SPI_CS_NONE (-1)

/**
 * @brief   Identifies one device on the shared SPI bus.
 *
 * SPI has no on-wire device address, so a device is selected by asserting its
 * chip select line. The driver asserts and deasserts it while holding the bus
 * mutex, so a whole transaction is atomic with respect to other tasks.
 *
 * cs_pin is an index into the board's pins[] table -- its digital_pin_t value --
 * the same way the rest of the codebase names a GPIO. Declare the pin there as
 * DIGITAL_OUTPUT with an initial state of GPIO_PIN_SET: chip select is active
 * low, so it must idle high from reset.
 */
typedef struct {
  int cs_pin; // index into pins[], or SPI_CS_NONE
} spi_device_t;

/**
 * @brief   Creates the mutex guarding the SPI bus
 *
 * Safe to call more than once; every call after the first is a no-op.
 */
void spi_driver_init(void);

/**
 * @brief   Binds the driver to an SPI peripheral
 *
 * The handle belongs to the bus and is set once. Calling this again with the
 * same handle succeeds and changes nothing; calling it with a different handle
 * fails rather than silently re-pointing a bus other devices are already using.
 *
 * @param hspi  SPI handler
 * @return HAL_OK on success, HAL_ERROR if hspi is NULL or the bus is already
 *         bound to a different handle
 */
HAL_StatusTypeDef spi_driver_configure(SPI_HandleTypeDef *hspi);

/**
 * @brief   Mutex protected HAL_SPI_Transmit function
 *
 * @param dev   Device to select for this transaction, or NULL for none
 * @param data  Pointer to the data to be sent
 * @param size  Number of bytes to send
 * @param mutex_acq_timeout      How long to wait for the bus mutex
 * @param transmission_timeout   SPI timeout duration
 */
HAL_StatusTypeDef spi_driver_transmit(const spi_device_t *dev, uint8_t *data, uint16_t size,
                                      TickType_t mutex_acq_timeout, uint32_t transmission_timeout);

/**
 * @brief   Mutex protected HAL_SPI_Receive function
 *
 * @param dev   Device to select for this transaction, or NULL for none
 * @param data  Pointer to the buffer to receive into
 * @param size  Number of bytes to receive
 * @param mutex_acq_timeout      How long to wait for the bus mutex
 * @param transmission_timeout   SPI timeout duration
 */
HAL_StatusTypeDef spi_driver_receive(const spi_device_t *dev, uint8_t *data, uint16_t size,
                                     TickType_t mutex_acq_timeout, uint32_t transmission_timeout);

/**
 * @brief   Mutex protected HAL_SPI_TransmitReceive function
 *
 * @param dev       Device to select for this transaction, or NULL for none
 * @param tx_buff   Pointer to the data to be sent
 * @param rx_buff   Pointer to the buffer to receive into
 * @param size      Number of bytes to exchange
 * @param mutex_acq_timeout      How long to wait for the bus mutex
 * @param transmission_timeout   SPI timeout duration
 */
HAL_StatusTypeDef spi_driver_transmit_receive(const spi_device_t *dev, uint8_t *tx_buff,
                                              uint8_t *rx_buff, uint16_t size,
                                              TickType_t mutex_acq_timeout,
                                              uint32_t transmission_timeout);

/**
 * @brief   Check if the driver is properly initialized
 *
 * @return bool True if initialized, false otherwise
 */
bool spi_driver_is_initialized(void);

#endif
