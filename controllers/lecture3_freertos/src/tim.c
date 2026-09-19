// FreeRTOS owns SysTick once the scheduler starts, so -- same as every real
// RTOS board in the team's firmware repo (e.g. controllers/FC/src/tim.c) --
// the HAL millisecond tick is redirected onto TIM6 instead of SysTick.
#include "main.h"

static TIM_HandleTypeDef tim6;

HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority) {
  __HAL_RCC_TIM6_CLK_ENABLE();

  uint32_t timer_clock = HAL_RCC_GetPCLK1Freq();
  uint32_t prescaler = (timer_clock / 1000000U) - 1U;

  tim6.Instance = TIM6;
  tim6.Init.Prescaler = prescaler;
  tim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  tim6.Init.Period = (1000000U / 1000U) - 1U;
  tim6.Init.ClockDivision = 0;

  HAL_StatusTypeDef status = HAL_TIM_Base_Init(&tim6);
  if (status != HAL_OK) { return status; }

  status = HAL_TIM_Base_Start_IT(&tim6);
  if (status != HAL_OK) { return status; }

  if (TickPriority >= (1UL << __NVIC_PRIO_BITS)) { return HAL_ERROR; }
  HAL_NVIC_SetPriority(TIM6_DAC_IRQn, TickPriority, 0U);
  HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
  uwTickPrio = TickPriority;

  return HAL_OK;
}

void HAL_SuspendTick(void) { __HAL_TIM_DISABLE_IT(&tim6, TIM_IT_UPDATE); }

void HAL_ResumeTick(void) { __HAL_TIM_ENABLE_IT(&tim6, TIM_IT_UPDATE); }

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM6) { HAL_IncTick(); }
}

void tim6_dac_irq_handler(void) { HAL_TIM_IRQHandler(&tim6); }
