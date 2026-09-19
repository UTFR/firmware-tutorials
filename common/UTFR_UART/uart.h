#ifndef UTFR_HAL_INIT_UART_H
#define UTFR_HAL_INIT_UART_H

#include "utfr_hal.h"

typedef enum {
  UART_BAUDRATE_9600 = 9600,
  UART_BAUDRATE_115200 = 115200,
} uart_baudrate_t;

typedef enum {
  USART1_RX_PB7 = GPIO_PIN_7,
  USART2_RX_PA3 = GPIO_PIN_3,
  USART3_RX_PE15 = GPIO_PIN_15,
  UART4_RX_PC11 = GPIO_PIN_11,
  UART5_RX_PD2 = GPIO_PIN_2,
} uart_rx_t;

typedef enum {
  USART1_TX_PA9 = GPIO_PIN_9,
  USART2_TX_PA2 = GPIO_PIN_2,
  // USART3_TX_PB10 = GPIO_PIN_10,
  UART4_TX_PC10 = GPIO_PIN_10,
  UART5_TX_PC12 = GPIO_PIN_12,
} uart_tx_t;

typedef struct {
  UART_HandleTypeDef handle;
  uart_baudrate_t baudrate;
  uart_rx_t rx;
  uart_tx_t tx;
} uart_t;

HAL_StatusTypeDef uart_init(uart_t *uart);
HAL_StatusTypeDef uart_deinit(uart_t *uart);
void uart_set_io_handle(UART_HandleTypeDef *handle);

#define IO_CHAR_TIMEOUT 10
extern UART_HandleTypeDef *io_uart_handle;

#endif
