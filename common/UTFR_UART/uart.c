#include <stdbool.h>

#include "uart.h"
#include "utfr_hal.h"

extern void error_handler(void);

static USART_TypeDef *pin_instance(uart_rx_t rx);
#if !defined(UTFR_MCU_FAMILY_H7)
// Only used by the G4 uart_msp_init()/uart_msp_deinit() below.
static GPIO_TypeDef *rx_port(uart_rx_t rx);
static GPIO_TypeDef *tx_port(uart_tx_t tx);
static void init_gpio_clk(uart_t *uart);
static void deinit_gpio_clk(uart_t *uart);
#endif

static void uart_msp_init(uart_t *uart);
static void uart_msp_deinit(uart_t *uart);

UART_HandleTypeDef *io_uart_handle = NULL;

HAL_StatusTypeDef uart_deinit(uart_t *uart) {
  if (uart->handle.Instance == NULL) { return HAL_ERROR; }
  HAL_UART_DeInit(&uart->handle);
  uart_msp_deinit(uart);
  return HAL_OK;
}

HAL_StatusTypeDef uart_init(uart_t *uart) {
  // assert_param(pins_compatible(rx, tx));
  uart->handle.Instance = pin_instance(uart->rx);
  uart->handle.Init.BaudRate = uart->baudrate;
  uart->handle.Init.WordLength = UART_WORDLENGTH_8B;
  uart->handle.Init.StopBits = UART_STOPBITS_1;
  uart->handle.Init.Parity = UART_PARITY_NONE;
  uart->handle.Init.Mode = UART_MODE_TX_RX;
  uart->handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  uart->handle.Init.OverSampling = UART_OVERSAMPLING_16;
  uart->handle.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  uart->handle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  uart_msp_init(uart);
  if (HAL_UART_Init(&uart->handle) != HAL_OK) { return HAL_ERROR; }
  if (HAL_UARTEx_SetTxFifoThreshold(&uart->handle, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) {
    error_handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&uart->handle, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) {
    error_handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&uart->handle) != HAL_OK) { error_handler(); }
  return HAL_OK;
}

void uart_set_io_handle(UART_HandleTypeDef *handle) { io_uart_handle = handle; }

#if defined(UTFR_MCU_FAMILY_H7)
// H7's RCC_PeriphCLKInitTypeDef field names and GPIO alternate-function
// numbers for these USART/UART instances differ from G4's (e.g.
// Usart1ClockSelection -> Usart16ClockSelection, GPIO_AF5_UART4 ->
// GPIO_AF8_UART4) -- not just renamed, the clock mux grouping itself
// differs. Nothing calls uart_init() on H7 yet (see controllers/H7dev/
// board.cmake), so rather than translate untested peripheral config,
// this is an explicit stub: implement properly, per H7 instance, before
// any H7 board actually calls uart_init().
static void uart_msp_init(uart_t *uart) {
  UNUSED(uart);
  error_handler();
}

static void uart_msp_deinit(uart_t *uart) { UNUSED(uart); }
#else
static void uart_msp_init(uart_t *uart) {
  GPIO_InitTypeDef gpio_init = {0};
  RCC_PeriphCLKInitTypeDef clk_init = {0};

  if (uart->handle.Instance == USART1) {
    clk_init.PeriphClockSelection = RCC_PERIPHCLK_USART1;
    clk_init.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
    __HAL_RCC_USART1_CLK_ENABLE();
    gpio_init.Alternate = GPIO_AF7_USART1;
  } else if (uart->handle.Instance == USART2) {
    clk_init.PeriphClockSelection = RCC_PERIPHCLK_USART2;
    clk_init.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
    __HAL_RCC_USART2_CLK_ENABLE();
    gpio_init.Alternate = GPIO_AF7_USART2;
  } else if (uart->handle.Instance == USART3) {
    clk_init.PeriphClockSelection = RCC_PERIPHCLK_USART3;
    clk_init.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
    __HAL_RCC_USART3_CLK_ENABLE();
    gpio_init.Alternate = GPIO_AF7_USART3;
  } else if (uart->handle.Instance == UART4) {
    clk_init.PeriphClockSelection = RCC_PERIPHCLK_UART4;
    clk_init.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
    __HAL_RCC_UART4_CLK_ENABLE();
    gpio_init.Alternate = GPIO_AF5_UART4;
  } else if (uart->handle.Instance == UART5) {
    clk_init.PeriphClockSelection = RCC_PERIPHCLK_UART5;
    clk_init.Uart5ClockSelection = RCC_UART5CLKSOURCE_PCLK1;
    __HAL_RCC_UART5_CLK_ENABLE();
    gpio_init.Alternate = GPIO_AF14_UART5;
  }

  if (HAL_RCCEx_PeriphCLKConfig(&clk_init) != HAL_OK) { error_handler(); }

  init_gpio_clk(uart);
  gpio_init.Pin = uart->rx;
  gpio_init.Mode = GPIO_MODE_AF_PP;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(rx_port(uart->rx), &gpio_init);
  gpio_init.Pin = uart->tx;
  HAL_GPIO_Init(tx_port(uart->tx), &gpio_init);
}

static void uart_msp_deinit(uart_t *uart) {
  if (uart->handle.Instance == USART1) {
    __HAL_RCC_USART1_CLK_DISABLE();
  } else if (uart->handle.Instance == USART2) {
    __HAL_RCC_USART2_CLK_DISABLE();
  } else if (uart->handle.Instance == USART3) {
    __HAL_RCC_USART3_CLK_DISABLE();
  } else if (uart->handle.Instance == UART4) {
    __HAL_RCC_UART4_CLK_DISABLE();
  } else if (uart->handle.Instance == UART5) {
    __HAL_RCC_UART5_CLK_DISABLE();
  }
  HAL_GPIO_DeInit(rx_port(uart->rx), uart->rx);
  HAL_GPIO_DeInit(tx_port(uart->tx), uart->tx);
  deinit_gpio_clk(uart);
}
#endif

// static bool pins_compatible(uart_rx_t rx, uart_tx_t tx) {
//   switch (rx) {
//   case USART1_RX_PB7:
//     switch (tx) {
//     case USART1_TX_PA9:  return true;
//     case USART2_TX_PA2:
//     case USART3_TX_PB10:
//     case UART4_TX_PC10:
//     case UART5_TX_PC12:  return false;
//     }
//   case USART2_RX_PA3:
//     switch (tx) {
//     case USART2_TX_PA2:  return true;
//     case USART1_TX_PA9:
//     case USART3_TX_PB10:
//     case UART4_TX_PC10:
//     case UART5_TX_PC12:  return false;
//     }
//   case USART3_RX_PE15:
//     switch (tx) {
//     case USART3_TX_PB10: return true;
//     case USART1_TX_PA9:
//     case USART2_TX_PA2:
//     case UART4_TX_PC10:
//     case UART5_TX_PC12:  return false;
//     }
//   case UART4_RX_PC11:
//     switch (tx) {
//     case UART4_TX_PC10:  return true;
//     case USART1_TX_PA9:
//     case USART2_TX_PA2:
//     case USART3_TX_PB10:
//     case UART5_TX_PC12:  return false;
//     }
//   case UART5_RX_PD2:
//     switch (tx) {
//     case UART5_TX_PC12:  return true;
//     case USART1_TX_PA9:
//     case USART2_TX_PA2:
//     case USART3_TX_PB10:
//     case UART4_TX_PC10:  return false;
//     }
//   }
//   return false;
// }

static USART_TypeDef *pin_instance(uart_rx_t rx) {
  switch (rx) {
  case USART1_RX_PB7:  return USART1;
  case USART2_RX_PA3:  return USART2;
  case USART3_RX_PE15: return USART3;
  case UART4_RX_PC11:  return UART4;
  case UART5_RX_PD2:   return UART5;
  }
  return NULL;
}

#if !defined(UTFR_MCU_FAMILY_H7)
static GPIO_TypeDef *rx_port(uart_rx_t rx) {
  switch (rx) {
  case USART1_RX_PB7:  return GPIOB;
  case USART2_RX_PA3:  return GPIOA;
  case USART3_RX_PE15: return GPIOE;
  case UART4_RX_PC11:  return GPIOC;
  case UART5_RX_PD2:   return GPIOD;
  }
  return NULL;
}

static GPIO_TypeDef *tx_port(uart_tx_t tx) {
  if (tx == USART1_TX_PA9) {
    return GPIOA;
  } else if (tx == USART2_TX_PA2) {
    return GPIOA;
    // } else if (tx == USART3_TX_PB10) {
    // printf("B\n");
    // return GPIOB;
  } else if (tx == UART4_TX_PC10) {
    return GPIOC;
  } else if (tx == UART5_TX_PC12) {
    return GPIOC;
  }
  return GPIOA;
}

// Only called from the G4 uart_msp_init()/uart_msp_deinit() above.
static void init_gpio_clk(uart_t *uart) {
  GPIO_TypeDef *_rx_port = rx_port(uart->rx);
  GPIO_TypeDef *_tx_port = tx_port(uart->tx);
  if (_rx_port == GPIOA) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
  } else if (_rx_port == GPIOB) {
    __HAL_RCC_GPIOB_CLK_ENABLE();
  } else if (_rx_port == GPIOC) {
    __HAL_RCC_GPIOC_CLK_ENABLE();
  } else if (_rx_port == GPIOD) {
    __HAL_RCC_GPIOD_CLK_ENABLE();
  } else if (_rx_port == GPIOE) {
    __HAL_RCC_GPIOE_CLK_ENABLE();
  } else if (_rx_port == GPIOF) {
    __HAL_RCC_GPIOF_CLK_ENABLE();
  } else if (_rx_port == GPIOG) {
    __HAL_RCC_GPIOG_CLK_ENABLE();
  }
  if (_tx_port == GPIOA) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
  } else if (_tx_port == GPIOB) {
    __HAL_RCC_GPIOB_CLK_ENABLE();
  } else if (_tx_port == GPIOC) {
    __HAL_RCC_GPIOC_CLK_ENABLE();
  } else if (_tx_port == GPIOD) {
    __HAL_RCC_GPIOD_CLK_ENABLE();
  } else if (_tx_port == GPIOE) {
    __HAL_RCC_GPIOE_CLK_ENABLE();
  } else if (_tx_port == GPIOF) {
    __HAL_RCC_GPIOF_CLK_ENABLE();
  } else if (_tx_port == GPIOG) {
    __HAL_RCC_GPIOG_CLK_ENABLE();
  }
}

static void deinit_gpio_clk(uart_t *uart) {
  GPIO_TypeDef *_rx_port = rx_port(uart->rx);
  GPIO_TypeDef *_tx_port = tx_port(uart->tx);
  if (_rx_port == GPIOA) {
    __HAL_RCC_GPIOA_CLK_DISABLE();
  } else if (_rx_port == GPIOB) {
    __HAL_RCC_GPIOB_CLK_DISABLE();
  } else if (_rx_port == GPIOC) {
    __HAL_RCC_GPIOC_CLK_DISABLE();
  } else if (_rx_port == GPIOD) {
    __HAL_RCC_GPIOD_CLK_DISABLE();
  } else if (_rx_port == GPIOE) {
    __HAL_RCC_GPIOE_CLK_DISABLE();
  } else if (_rx_port == GPIOF) {
    __HAL_RCC_GPIOF_CLK_DISABLE();
  } else if (_rx_port == GPIOG) {
    __HAL_RCC_GPIOG_CLK_DISABLE();
  }
  if (_tx_port == GPIOA) {
    __HAL_RCC_GPIOA_CLK_DISABLE();
  } else if (_tx_port == GPIOB) {
    __HAL_RCC_GPIOB_CLK_DISABLE();
  } else if (_tx_port == GPIOC) {
    __HAL_RCC_GPIOC_CLK_DISABLE();
  } else if (_tx_port == GPIOD) {
    __HAL_RCC_GPIOD_CLK_DISABLE();
  } else if (_tx_port == GPIOE) {
    __HAL_RCC_GPIOE_CLK_DISABLE();
  } else if (_tx_port == GPIOF) {
    __HAL_RCC_GPIOF_CLK_DISABLE();
  } else if (_tx_port == GPIOG) {
    __HAL_RCC_GPIOG_CLK_DISABLE();
  }
}
#endif
