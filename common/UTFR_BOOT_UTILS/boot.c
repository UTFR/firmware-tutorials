#include "boot.h"
#include "itm.h"
#include "utfr_hal.h"
#include "vector.h"
#include "memory_map.h"

static inline void fpu_enable(void);

extern int main(void);
extern void error_handler(void);

#if defined(UTFR_MCU_FAMILY_H7)
uint32_t SystemCoreClock = 64000000; // HSI reset default; sys_clock_config() updates this
uint32_t SystemD2Clock = 64000000;
const uint8_t D1CorePrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
#else
uint32_t SystemCoreClock = 48000000;
const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
const uint8_t APBPrescTable[8] = {0, 0, 0, 0, 1, 2, 3, 4};
#endif

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {
  UNUSED(file);
  UNUSED(line);
}
#endif

#if defined(UTFR_MCU_FAMILY_H7)
static inline void h7_cm7_early_init(void) {
  SCB_EnableICache();
  SCB_EnableDCache();

  uint32_t timeout = 0xFFFFU;
  while ((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0)) {}
  if (timeout == 0) { error_handler(); }
}
#endif

__attribute__((noreturn)) void reset_handler(void) {
  fpu_enable();
#if defined(UTFR_MCU_FAMILY_H7)
  h7_cm7_early_init();
#endif
  vtor_init((uint32_t)&vector_table);

  // copy data segment initial values
  uint32_t *src = &_sidata;
  uint32_t *dst = &_sdata;
  while (dst < &_edata) *dst++ = *src++;

  // zero initialize bss
  for (dst = &_sbss; dst < &_ebss; dst++) *dst = 0;

  if (debugger_attached()) { debugger_enable(); }
  main();
  __builtin_unreachable();
}

#if defined(UTFR_MCU_FAMILY_H7)
void sys_clock_config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  if (HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY) != HAL_OK) { error_handler(); }
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
  RCC_OscInitStruct.CSIState = RCC_CSI_OFF;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;   // 8 / 4 = 2 MHz VCO input
  RCC_OscInitStruct.PLL.PLLN = 400; // 2 * 400 = 800 MHz VCO
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  RCC_OscInitStruct.PLL.PLLP = 2;   // 800 / 2 = 400 MHz -- SYSCLK source (PLL1_P)
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { error_handler(); }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_D1PCLK1
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) { error_handler(); }

  SystemCoreClock = 400000000U;
  SystemD2Clock = 200000000U;
}
#else
void sys_clock_config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV3; // 48 / 3 = 16
  RCC_OscInitStruct.PLL.PLLN = 12;            // 16 * 12 = 192
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2; //
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4; // 192 / 4 = 48
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { error_handler(); }

  RCC_ClkInitStruct.ClockType
    = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) { error_handler(); }

  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12 | RCC_PERIPHCLK_ADC345;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_PLL;   /* HSE -> PLL -> ADC */
  PeriphClkInit.Adc345ClockSelection = RCC_ADC345CLKSOURCE_PLL; /* HSE -> PLL -> ADC */
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
}
#endif

static inline void fpu_enable(void) {
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
  // Privileged and User mode access to CP10 and CP11
  SCB->CPACR |= 0xF << 20;
#endif
}
