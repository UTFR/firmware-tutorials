/*
Source file for thread safe SPI peripheral functions
*/
#include "spi_driver.h"
#include "UTFR_DIGITAL/pin_driver.h"
#include "utfr_hal.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "UTFR_LOGGING/logger.h"
#include <stdbool.h>

// Static variables
static SPI_HandleTypeDef *spi_handle = NULL;
static StaticSemaphore_t mutex_buffer;
static SemaphoreHandle_t mutex;

// Chip select
static void cs_select(const spi_device_t *dev) {
  if (dev != NULL && dev->cs_pin != SPI_CS_NONE) { digital_pin_write(dev->cs_pin, GPIO_PIN_RESET); }
}

// Chip deselect
static void cs_deselect(const spi_device_t *dev) {
  if (dev != NULL && dev->cs_pin != SPI_CS_NONE) { digital_pin_write(dev->cs_pin, GPIO_PIN_SET); }
}

// Creates the bus mutex
void spi_driver_init(void) {
  taskENTER_CRITICAL();

  if (mutex == NULL) { mutex = xSemaphoreCreateMutexStatic(&mutex_buffer); }

  taskEXIT_CRITICAL();
}

// Binds the bus to an SPI peripheral; repeat calls with the same handle are a no-op
HAL_StatusTypeDef spi_driver_configure(SPI_HandleTypeDef *hspi) {
  if (hspi == NULL) { return HAL_ERROR; }

  taskENTER_CRITICAL();

  if (spi_handle != NULL && spi_handle != hspi) {
    taskEXIT_CRITICAL();
    LOGE("SPI bus already configured with a different handle");
    return HAL_ERROR;
  }
  spi_handle = hspi;

  taskEXIT_CRITICAL();
  return HAL_OK;
}

// Mutex protected SPI transmit function
HAL_StatusTypeDef spi_driver_transmit(const spi_device_t *dev, uint8_t *data, uint16_t size,
                                      TickType_t mutex_acq_timeout, uint32_t transmission_timeout) {
  if (spi_handle == NULL || mutex == NULL) { return HAL_ERROR; }

  if (xSemaphoreTake(mutex, mutex_acq_timeout) != pdTRUE) { return HAL_TIMEOUT; }

  cs_select(dev);
  HAL_StatusTypeDef status = HAL_SPI_Transmit(spi_handle, data, size, transmission_timeout);
  cs_deselect(dev);

  xSemaphoreGive(mutex);
  return status;
}

HAL_StatusTypeDef spi_driver_receive(const spi_device_t *dev, uint8_t *data, uint16_t size,
                                     TickType_t mutex_acq_timeout, uint32_t transmission_timeout) {
  if (spi_handle == NULL || mutex == NULL) { return HAL_ERROR; }

  if (xSemaphoreTake(mutex, mutex_acq_timeout) != pdTRUE) { return HAL_TIMEOUT; }

  cs_select(dev);
  HAL_StatusTypeDef status = HAL_SPI_Receive(spi_handle, data, size, transmission_timeout);
  cs_deselect(dev);

  xSemaphoreGive(mutex);
  return status;
}

HAL_StatusTypeDef spi_driver_transmit_receive(const spi_device_t *dev, uint8_t *tx_buff,
                                              uint8_t *rx_buff, uint16_t size,
                                              TickType_t mutex_acq_timeout,
                                              uint32_t transmission_timeout) {
  if (spi_handle == NULL || mutex == NULL) { return HAL_ERROR; }

  if (xSemaphoreTake(mutex, mutex_acq_timeout) != pdTRUE) { return HAL_TIMEOUT; }

  cs_select(dev);
  HAL_StatusTypeDef status
    = HAL_SPI_TransmitReceive(spi_handle, tx_buff, rx_buff, size, transmission_timeout);
  cs_deselect(dev);

  xSemaphoreGive(mutex);
  return status;
}

bool spi_driver_is_initialized(void) { return (spi_handle != NULL && mutex != NULL); }
