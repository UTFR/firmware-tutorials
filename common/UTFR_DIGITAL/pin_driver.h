#ifndef UTFR_DIGITAL_PIN_DRIVER_H
#define UTFR_DIGITAL_PIN_DRIVER_H

#include "utfr_hal.h"

typedef enum {
  DIGITAL_INPUT,
  DIGITAL_OUTPUT,
  DIGITAL_INTERRUPT_RISING,
  DIGITAL_INTERRUPT_FALLING,
  DIGITAL_INTERRUPT_RISING_FALLING,
} digital_io_mode_t;

typedef enum {
  DIGITAL_NOPULL = 0x0,
  DIGITAL_PULLUP = 0x1,
  DIGITAL_PULLDOWN = 0x2,
} digital_pull_mode_t;

typedef struct {
  GPIO_TypeDef *port;
  uint16_t pin;
  digital_io_mode_t mode;
  digital_pull_mode_t pull;
  GPIO_PinState initial_state;
} digital_config_t;

extern digital_config_t pins[];

/**
 * @brief Initialize a GPIO pin
 */
void digital_pin_init(int pin);

GPIO_PinState digital_pin_read(int pin);
void digital_pin_write(int pin, GPIO_PinState state);
void digital_pin_toggle(int pin);
GPIO_TypeDef *digital_pin_get_port(int pin);

#endif
