#include "bsp.h"
#include "config.h"
#include "main.h"
#include "UTFR_DIGITAL/pin_driver.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

// --- CAN -------------------------------------------------------------------

#define TUTORIAL_STEERING_ANGLE_CAN_ID 0x100u

can_t vcan
  = {.name = "VCAN", .baudrate = CAN_BAUDRATE_500KBPS, .rx = CAN1_RX_PA11, .tx = CAN1_TX_PA12};

volatile float g_last_steering_angle_deg;

static can_filter_t can_filters[] = {
  CAN_FILTER_ACCEPT_ALL(CAN_FILTER_ACTION_RXFIFO0),
};

void bsp_can_init(void) {
  if (can_init(&vcan, can_filters, sizeof(can_filters) / sizeof(can_filters[0]),
               FDCAN_IT_RX_FIFO0_NEW_MESSAGE)
      != HAL_OK) {
    error_handler();
  }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
  (void)RxFifo0ITs;

  FDCAN_RxHeaderTypeDef rx_header = {0};
  can_msg_t msg = {0};
  if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, msg.data) != HAL_OK) { return; }
  msg.raw_id = rx_header.Identifier;

  if (msg.raw_id == TUTORIAL_STEERING_ANGLE_CAN_ID) {
    float angle;
    memcpy(&angle, msg.data, sizeof(angle));
    // See the "EXTREMELY IMPORTANT" note in bsp.h: this write races with
    // whoever reads g_last_steering_angle_deg. That's intentional.
    g_last_steering_angle_deg = angle;
  }
}

// --- ADC (current sensor) ---------------------------------------------------

static ADC_HandleTypeDef adc1;

static void adc_gpio_init(void) {
  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitTypeDef init = {0};
  init.Pin = GPIO_PIN_0;
  init.Mode = GPIO_MODE_ANALOG;
  init.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &init);
}

void bsp_adc_init(void) {
  adc_gpio_init();
  __HAL_RCC_ADC12_CLK_ENABLE();

  adc1.Instance = ADC1;
  adc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  adc1.Init.Resolution = ADC_RESOLUTION_12B;
  adc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  adc1.Init.GainCompensation = 0;
  adc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  adc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  adc1.Init.LowPowerAutoWait = DISABLE;
  adc1.Init.ContinuousConvMode = DISABLE;
  adc1.Init.NbrOfConversion = 1;
  adc1.Init.DiscontinuousConvMode = DISABLE;
  adc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  adc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  adc1.Init.DMAContinuousRequests = DISABLE;
  adc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  adc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&adc1) != HAL_OK) { error_handler(); }

  if (HAL_ADCEx_Calibration_Start(&adc1, ADC_SINGLE_ENDED) != HAL_OK) { error_handler(); }

  ADC_ChannelConfTypeDef ch_config = {0};
  ch_config.Channel = ADC_CHANNEL_1;
  ch_config.Rank = ADC_REGULAR_RANK_1;
  ch_config.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  ch_config.SingleDiff = ADC_SINGLE_ENDED;
  ch_config.OffsetNumber = ADC_OFFSET_NONE;
  ch_config.Offset = 0;
  if (HAL_ADC_ConfigChannel(&adc1, &ch_config) != HAL_OK) { error_handler(); }
}

uint16_t bsp_adc_read_current_raw(void) {
  HAL_ADC_Start(&adc1);
  HAL_ADC_PollForConversion(&adc1, 10);
  uint16_t value = (uint16_t)HAL_ADC_GetValue(&adc1);
  HAL_ADC_Stop(&adc1);
  return value;
}

// --- Wheelspeed --------------------------------------------------------------

static volatile uint32_t wheelspeed_counts[4];

void bsp_wheelspeed_init(void) {
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, INTERRUPT_PRIORITY, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void exti9_5_irq_handler(void) {
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);
}

void HAL_GPIO_EXTI_Callback(uint16_t pin) {
  switch (pin) {
  case GPIO_PIN_6: wheelspeed_counts[WHEEL_FL]++; break;
  case GPIO_PIN_7: wheelspeed_counts[WHEEL_FR]++; break;
  case GPIO_PIN_8: wheelspeed_counts[WHEEL_RL]++; break;
  case GPIO_PIN_9: wheelspeed_counts[WHEEL_RR]++; break;
  default:         break;
  }
}

uint32_t bsp_wheelspeed_get_and_reset_count(wheel_t wheel) {
  __disable_irq();
  uint32_t count = wheelspeed_counts[wheel];
  wheelspeed_counts[wheel] = 0;
  __enable_irq();
  return count;
}

// --- Buttons + relays --------------------------------------------------------

bool bsp_ts_on_pressed(void) { return digital_pin_read(PIN_TS_ON_BUTTON) == GPIO_PIN_RESET; }
bool bsp_rtd_pressed(void) { return digital_pin_read(PIN_RTD_BUTTON) == GPIO_PIN_RESET; }

void bsp_air_plus(bool energize) {
  digital_pin_write(PIN_AIR_PLUS, energize ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
void bsp_air_minus(bool energize) {
  digital_pin_write(PIN_AIR_MINUS, energize ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
void bsp_precharge(bool energize) {
  digital_pin_write(PIN_PRECHARGE, energize ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

// --- LCD + BMS, sharing SPI1 --------------------------------------------------

static SPI_HandleTypeDef spi1;

void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi) {
  (void)hspi;
  __HAL_RCC_SPI1_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitTypeDef init = {0};
  init.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
  init.Mode = GPIO_MODE_AF_PP;
  init.Pull = GPIO_NOPULL;
  init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  init.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &init);
}

void bsp_spi_bus_init(void) {
  spi1.Instance = SPI1;
  spi1.Init.Mode = SPI_MODE_MASTER;
  spi1.Init.Direction = SPI_DIRECTION_2LINES;
  spi1.Init.DataSize = SPI_DATASIZE_8BIT;
  spi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  spi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  spi1.Init.NSS = SPI_NSS_SOFT;
  spi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  spi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  spi1.Init.TIMode = SPI_TIMODE_DISABLE;
  spi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  spi1.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&spi1) != HAL_OK) { error_handler(); }
}

void bsp_lcd_printf(const char *fmt, ...) {
  char buf[64];
  va_list args;
  va_start(args, fmt);
  int len = vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  if (len <= 0) { return; }
  if ((size_t)len >= sizeof(buf)) { len = sizeof(buf) - 1; }

  digital_pin_write(PIN_LCD_CS, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&spi1, (uint8_t *)buf, (uint16_t)len, HAL_MAX_DELAY);
  digital_pin_write(PIN_LCD_CS, GPIO_PIN_SET);
}

static float bms_transact(uint8_t command, uint8_t cell, float scale, float offset) {
  uint8_t tx[3] = {command, cell, 0x00};
  uint8_t rx[3] = {0};

  digital_pin_write(PIN_BMS_CS, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&spi1, tx, rx, sizeof(tx), HAL_MAX_DELAY);
  digital_pin_write(PIN_BMS_CS, GPIO_PIN_SET);

  uint16_t raw = ((uint16_t)rx[1] << 8) | rx[2];
  return ((float)raw * scale) + offset;
}

float bsp_bms_get_voltage(uint8_t cell) { return bms_transact(0x01, cell, 5.0f / 65535.0f, 0.0f); }

float bsp_bms_get_temperature(uint8_t cell) {
  return bms_transact(0x02, cell, 150.0f / 65535.0f, -40.0f);
}

// --- Torque calculation ------------------------------------------------------

void calculate_torque_cmd(float *torques, float current, const float *wheelspeeds,
                          float steering_angle_deg) {
  // Placeholder for the real vehicle model -- treat this as a black box you
  // call, not one you write for this assignment.
  (void)wheelspeeds;
  (void)steering_angle_deg;
  float commanded = current * 0.5f;
  for (int i = 0; i < 4; i++) { torques[i] = commanded; }
}
