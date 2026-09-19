#ifndef INTRO_PROJECT_BSP_H
#define INTRO_PROJECT_BSP_H

// Board-support layer: real STM32G4 HAL calls, provided so the assignment
// itself (src/intro_project.c) can focus on the concurrency architecture --
// which tasks, which mutexes/queues, how the shared SPI bus and the CAN state
// get protected -- exactly like the original mocked `extern`s did, except
// these now do real hardware transactions instead of nothing.
//
// NOTE -- EXTREMELY IMPORTANT, same as the original assignment's CAN warning:
// g_last_steering_angle_deg is written from CAN RX interrupt context and read
// wherever you read it. This file does NOT synchronize that for you.
// Likewise bsp_lcd_printf()/bsp_bms_get_voltage()/bsp_bms_get_temperature()
// all drive the same physical SPI1 bus and are NOT arbitrated against each
// other. Both are your problem to solve.

#include <stdbool.h>
#include <stdint.h>

// --- CAN (steering angle) -------------------------------------------------
void bsp_can_init(void);
extern volatile float g_last_steering_angle_deg;

// --- Current sensor (ADC1_IN1 / PA0), 1 amp per 10 mV --------------------
void bsp_adc_init(void);
uint16_t bsp_adc_read_current_raw(void);

// --- Wheelspeed (GPIO EXTI tooth counters, 17 teeth/rev) ------------------
typedef enum {
  WHEEL_FL,
  WHEEL_FR,
  WHEEL_RL,
  WHEEL_RR,
} wheel_t;

void bsp_wheelspeed_init(void);
uint32_t bsp_wheelspeed_get_and_reset_count(wheel_t wheel);

// --- Buttons + relays (digital_pin_init() on the pins[] table in main.c
//     already brings these up -- these are just typed accessors) ----------
bool bsp_ts_on_pressed(void);
bool bsp_rtd_pressed(void);
void bsp_air_plus(bool energize);
void bsp_air_minus(bool energize);
void bsp_precharge(bool energize);

// --- LCD + BMS, sharing SPI1 ----------------------------------------------
void bsp_spi_bus_init(void);
void bsp_lcd_printf(const char *fmt, ...);
float bsp_bms_get_voltage(uint8_t cell);
float bsp_bms_get_temperature(uint8_t cell);

// --- Torque calculation, still a provided black box -----------------------
void calculate_torque_cmd(float *torques, float current, const float *wheelspeeds,
                          float steering_angle_deg);

#endif
