#include "pin_driver.h"
#include "utfr_hal.h"
#include <stdint.h>

void digital_pin_init(int pin) {
  digital_config_t *config = &pins[pin];
  GPIO_InitTypeDef init = {0};
  if (config->port == GPIOA) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
  } else if (config->port == GPIOB) {
    __HAL_RCC_GPIOB_CLK_ENABLE();
  } else if (config->port == GPIOC) {
    __HAL_RCC_GPIOC_CLK_ENABLE();
  } else if (config->port == GPIOD) {
    __HAL_RCC_GPIOD_CLK_ENABLE();
  } else if (config->port == GPIOE) {
    __HAL_RCC_GPIOE_CLK_ENABLE();
  } else if (config->port == GPIOF) {
    __HAL_RCC_GPIOF_CLK_ENABLE();
  } else if (config->port == GPIOG) {
    __HAL_RCC_GPIOG_CLK_ENABLE();
  }
  init.Pull = config->pull;
  init.Speed = GPIO_SPEED_LOW;
  init.Pin = config->pin;

  switch (config->mode) {
  case DIGITAL_INPUT:  init.Mode = GPIO_MODE_INPUT; break;
  case DIGITAL_OUTPUT: {
    init.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_WritePin(config->port, config->pin, config->initial_state);
    break;
  }
  case DIGITAL_INTERRUPT_RISING:         init.Mode = GPIO_MODE_IT_RISING; break;
  case DIGITAL_INTERRUPT_FALLING:        init.Mode = GPIO_MODE_IT_FALLING; break;
  case DIGITAL_INTERRUPT_RISING_FALLING: init.Mode = GPIO_MODE_IT_RISING_FALLING; break;
  }

  HAL_GPIO_Init(config->port, &init);
  // HAL_GPIO_LockPin(config->port, (uint16_t)pin);
}

GPIO_PinState digital_pin_read(int pin) {
  digital_config_t *config = &pins[pin];
  return HAL_GPIO_ReadPin(config->port, config->pin);
}

void digital_pin_write(int pin, GPIO_PinState state) {
  digital_config_t *config = &pins[pin];
  HAL_GPIO_WritePin(config->port, config->pin, state);
}

void digital_pin_toggle(int pin) {
  digital_config_t *config = &pins[pin];
  HAL_GPIO_TogglePin(config->port, config->pin);
}

GPIO_TypeDef *digital_pin_get_port(int pin) {
  digital_config_t *config = &pins[pin];
  return config->port;
}
