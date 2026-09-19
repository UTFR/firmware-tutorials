#include "utfr_hal.h"
#include "vector.h"
#include "memory_map.h"
#include "boot.h"

void dummy_handler(void) {
  for (;;) {
    __asm__(""); // make sure the loop doesn't get optimized away... there are some compiler bugs
                 // with this stuff
  }
}

__attribute__((weak, alias("dummy_handler"))) void nmi_handler(void);
__attribute__((weak, alias("dummy_handler"))) void hard_fault_handler(void);
__attribute__((weak, alias("dummy_handler"))) void memory_manage_fault_handler(void);
__attribute__((weak, alias("dummy_handler"))) void bus_fault_handler(void);
__attribute__((weak, alias("dummy_handler"))) void usage_fault_handler(void);
__attribute__((weak, alias("dummy_handler"))) void svc_handler(void);
__attribute__((weak, alias("dummy_handler"))) void debug_monitor_handler(void);
__attribute__((weak, alias("dummy_handler"))) void pend_sv_handler(void);
__attribute__((weak, alias("dummy_handler"))) void systick_handler(void);

#if defined(UTFR_MCU_FAMILY_H7)
// STM32H755's IRQn_Type (stm32h755xx.h) is a completely different
// peripheral/IRQ layout from G4's -- none of the named IRQ handlers in the
// #else branch below apply. Every NVIC slot defaults to dummy_handler
// (a spin loop -- see above), same weak-alias-with-fallback pattern as the
// core exception handlers above and the G4 IRQ table below: a strong
// definition elsewhere (e.g. controllers/RC/src/tim.c's tim6_dac_irq_handler)
// overrides the weak default when that source is actually linked in, and a
// board that never defines one keeps the harmless dummy_handler fallback.
// Named here only for peripherals controllers/H7dev's currently-linked
// application (RC's real source, see controllers/H7dev/CMakeLists.txt)
// actually enables: a board pulling in a different peripheral needs its own
// slot named the same way, after the matching IRQn_Type entry.
__attribute__((weak, alias("dummy_handler"))) void tim6_dac_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void adc1_2_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void adc3_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void dma1_channel1_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void dma1_channel2_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void dma1_channel3_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void fdcan1_it0_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void fdcan1_it1_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void fdcan2_it0_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void fdcan2_it1_irq_handler(void);

// The .irq initializer below uses a GNU range designator, not standard C --
// fine under -std=gnu11 (this project's actual dialect), just needs
// -Wpedantic told so explicitly; a #pragma can't sit inside the initializer
// list itself, so it wraps the whole declaration instead. The specific-IRQ
// entries below deliberately override slots the range designator already
// set to dummy_handler -- that's -Woverride-init, not a mistake.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Woverride-init"
vector_table_t vector_table __attribute__((used, section(".vector_table"))) = {
  .initial_sp = &_estack,
  .reset = reset_handler,
  .nmi = nmi_handler,
  .hard_fault = hard_fault_handler,
  .memory_manage_fault = memory_manage_fault_handler,
  .bus_fault = bus_fault_handler,
  .usage_fault = usage_fault_handler,
  .svc = svc_handler,
  .debug_monitor = debug_monitor_handler,
  .pend_sv = pend_sv_handler,
  .systick = systick_handler,
  .irq = {
    [0 ... NVIC_IRQ_COUNT - 1] = dummy_handler,
    [DMA1_Stream0_IRQn] = dma1_channel1_irq_handler,
    [DMA1_Stream1_IRQn] = dma1_channel2_irq_handler,
    [DMA1_Stream2_IRQn] = dma1_channel3_irq_handler,
    [ADC_IRQn] = adc1_2_irq_handler,
    [FDCAN1_IT0_IRQn] = fdcan1_it0_irq_handler,
    [FDCAN2_IT0_IRQn] = fdcan2_it0_irq_handler,
    [FDCAN1_IT1_IRQn] = fdcan1_it1_irq_handler,
    [FDCAN2_IT1_IRQn] = fdcan2_it1_irq_handler,
    [TIM6_DAC_IRQn] = tim6_dac_irq_handler,
    [ADC3_IRQn] = adc3_irq_handler,
  },
};
#pragma GCC diagnostic pop
#else

// IRQs
__attribute__((weak, alias("dummy_handler"))) void wwdg_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void rtc_wakeup_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void flash_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void rcc_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void exti0_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void exti1_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void exti2_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void exti3_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void exti4_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void exti9_5_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void tim1_cc_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void tim2_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void tim3_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void tim4_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void i2c1_ev_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void i2c1_er_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void i2c2_ev_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void i2c2_er_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void spi1_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void spi2_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void usart1_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void usart2_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void usart3_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void exti15_10_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void rtc_alarm_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void tim8_cc_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void fmc_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void tim5_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void spi3_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void uart4_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void uart5_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void tim6_dac_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void i2c3_ev_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void i2c3_er_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void rng_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void fpu_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void spi4_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void sai1_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void quadspi_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void lptim1_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void pvd_pvm_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void rtc_tamp_lsecss_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void dma1_channel1_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void dma1_channel2_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void dma1_channel3_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void dma1_channel4_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void dma1_channel5_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void dma1_channel6_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void dma1_channel7_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void dma1_channel8_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void adc1_2_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void usb_hp_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void usb_lp_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void fdcan1_it0_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void fdcan1_it1_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void fdcan2_it0_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void fdcan2_it1_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void fdcan3_it0_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void fdcan3_it1_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void tim1_brk_tim15_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void tim1_up_tim16_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void tim1_trg_com_tim17_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void usb_wakeup_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void tim8_brk_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void tim8_up_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void tim8_trg_com_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void adc3_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void tim7_dac_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void dma2_channel1_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void dma2_channel2_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void dma2_channel3_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void dma2_channel4_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void dma2_channel5_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void dma2_channel6_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void dma2_channel7_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void dma2_channel8_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void adc4_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void adc5_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void ucpd1_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void comp1_2_3_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void comp4_5_6_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void comp7_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void hrtim1_master_irqn(void);
__attribute__((weak, alias("dummy_handler"))) void hrtim1_tima_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void hrtim1_timb_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void hrtim1_timc_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void hrtim1_timd_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void hrtim1_time_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void hrtim1_timf_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void hrtim1_flt_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void crs_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void tim20_brk_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void tim20_up_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void tim20_trg_com_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void tim20_cc_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void i2c4_ev_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void i2c4_er_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void lpuart1_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void dmamux_ovr_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void cordic_irq_handler(void);
__attribute__((weak, alias("dummy_handler"))) void fmac_irq_handler(void);

vector_table_t vector_table __attribute__((used, section(".vector_table"))) = {
  .initial_sp = &_estack,
  .reset = reset_handler,
  .nmi = nmi_handler,
  .hard_fault = hard_fault_handler,
  .memory_manage_fault = memory_manage_fault_handler,
  .bus_fault = bus_fault_handler,
  .usage_fault = usage_fault_handler,
  .svc = svc_handler,
  .debug_monitor = debug_monitor_handler,
  .pend_sv = pend_sv_handler,
  .systick = systick_handler,
  .irq = {
          [WWDG_IRQn] = wwdg_irq_handler,
          [PVD_PVM_IRQn] = pvd_pvm_irq_handler,
          [RTC_TAMP_LSECSS_IRQn] = rtc_tamp_lsecss_irq_handler,
          [RTC_WKUP_IRQn] = rtc_wakeup_irq_handler,
          [FLASH_IRQn] = flash_irq_handler,
          [RCC_IRQn] = rcc_irq_handler,
          [EXTI0_IRQn] = exti0_irq_handler,
          [EXTI1_IRQn] = exti1_irq_handler,
          [EXTI2_IRQn] = exti2_irq_handler,
          [EXTI3_IRQn] = exti3_irq_handler,
          [EXTI4_IRQn] = exti4_irq_handler,
          [DMA1_Channel1_IRQn] = dma1_channel1_irq_handler,
          [DMA1_Channel2_IRQn] = dma1_channel2_irq_handler,
          [DMA1_Channel3_IRQn] = dma1_channel3_irq_handler,
          [DMA1_Channel4_IRQn] = dma1_channel4_irq_handler,
          [DMA1_Channel5_IRQn] = dma1_channel5_irq_handler,
          [DMA1_Channel6_IRQn] = dma1_channel6_irq_handler,
          [DMA1_Channel7_IRQn] = dma1_channel7_irq_handler,
          [ADC1_2_IRQn] = adc1_2_irq_handler,
          [USB_HP_IRQn] = usb_hp_irq_handler,
          [USB_LP_IRQn] = usb_lp_irq_handler,
          [FDCAN1_IT0_IRQn] = fdcan1_it0_irq_handler,
          [FDCAN1_IT1_IRQn] = fdcan1_it1_irq_handler,
          [EXTI9_5_IRQn] = exti9_5_irq_handler,
          [TIM1_BRK_TIM15_IRQn] = tim1_brk_tim15_irq_handler,
          [TIM1_UP_TIM16_IRQn] = tim1_up_tim16_irq_handler,
          [TIM1_TRG_COM_TIM17_IRQn] = tim1_trg_com_tim17_irq_handler,
          [TIM1_CC_IRQn] = tim1_cc_irq_handler,
          [TIM2_IRQn] = tim2_irq_handler,
          [TIM3_IRQn] = tim3_irq_handler,
          [TIM4_IRQn] = tim4_irq_handler,
          [I2C1_EV_IRQn] = i2c1_ev_irq_handler,
          [I2C1_ER_IRQn] = i2c1_er_irq_handler,
          [I2C2_EV_IRQn] = i2c2_ev_irq_handler,
          [I2C2_ER_IRQn] = i2c2_er_irq_handler,
          [SPI1_IRQn] = spi1_irq_handler,
          [SPI2_IRQn] = spi2_irq_handler,
          [USART1_IRQn] = usart1_irq_handler,
          [USART2_IRQn] = usart2_irq_handler,
          [USART3_IRQn] = usart3_irq_handler,
          [EXTI15_10_IRQn] = exti15_10_irq_handler,
          [RTC_Alarm_IRQn] = rtc_alarm_irq_handler,
          [USBWakeUp_IRQn] = usb_wakeup_irq_handler,
          [TIM8_BRK_IRQn] = tim8_brk_irq_handler,
          [TIM8_UP_IRQn] = tim8_up_irq_handler,
          [TIM8_TRG_COM_IRQn] = tim8_trg_com_irq_handler,
          [TIM8_CC_IRQn] = tim8_cc_irq_handler,
          [ADC3_IRQn] = adc3_irq_handler,
          [FMC_IRQn] = fmc_irq_handler,
          [LPTIM1_IRQn] = lptim1_irq_handler,
          [TIM5_IRQn] = tim5_irq_handler,
          [SPI3_IRQn] = spi3_irq_handler,
          [UART4_IRQn] = uart4_irq_handler,
          [UART5_IRQn] = uart5_irq_handler,
          [TIM6_DAC_IRQn] = tim6_dac_irq_handler,
          [TIM7_DAC_IRQn] = tim7_dac_irq_handler,
          [DMA2_Channel1_IRQn] = dma2_channel1_irq_handler,
          [DMA2_Channel2_IRQn] = dma2_channel2_irq_handler,
          [DMA2_Channel3_IRQn] = dma2_channel3_irq_handler,
          [DMA2_Channel4_IRQn] = dma2_channel4_irq_handler,
          [DMA2_Channel5_IRQn] = dma2_channel5_irq_handler,
          [ADC4_IRQn] = adc4_irq_handler,
          [ADC5_IRQn] = adc5_irq_handler,
          [UCPD1_IRQn] = ucpd1_irq_handler,
          [COMP1_2_3_IRQn] = comp1_2_3_irq_handler,
          [COMP4_5_6_IRQn] = comp4_5_6_irq_handler,
          [COMP7_IRQn] = comp7_irq_handler,
          [HRTIM1_Master_IRQn] = hrtim1_master_irqn,
          [HRTIM1_TIMA_IRQn] = hrtim1_tima_irq_handler,
          [HRTIM1_TIMB_IRQn] = hrtim1_timb_irq_handler,
          [HRTIM1_TIMC_IRQn] = hrtim1_timc_irq_handler,
          [HRTIM1_TIMD_IRQn] = hrtim1_timd_irq_handler,
          [HRTIM1_TIME_IRQn] = hrtim1_time_irq_handler,
          [HRTIM1_TIMF_IRQn] = hrtim1_timf_irq_handler,
          [HRTIM1_FLT_IRQn] = hrtim1_flt_irq_handler,
          [CRS_IRQn] = crs_irq_handler,
          [SAI1_IRQn] = sai1_irq_handler,
          [TIM20_BRK_IRQn] = tim20_brk_irq_handler,
          [TIM20_UP_IRQn] = tim20_up_irq_handler,
          [TIM20_TRG_COM_IRQn] = tim20_trg_com_irq_handler,
          [TIM20_CC_IRQn] = tim20_cc_irq_handler,
          [FPU_IRQn] = fpu_irq_handler,
          [I2C4_EV_IRQn] = i2c4_ev_irq_handler,
          [I2C4_ER_IRQn] = i2c4_er_irq_handler,
          [SPI4_IRQn] = spi4_irq_handler,
          [FDCAN2_IT0_IRQn] = fdcan2_it0_irq_handler,
          [FDCAN2_IT1_IRQn] = fdcan2_it1_irq_handler,
          [FDCAN3_IT0_IRQn] = fdcan3_it0_irq_handler,
          [FDCAN3_IT1_IRQn] = fdcan3_it1_irq_handler,
          [RNG_IRQn] = rng_irq_handler,
          [LPUART1_IRQn] = lpuart1_irq_handler,
          [I2C3_EV_IRQn] = i2c3_ev_irq_handler,
          [I2C3_ER_IRQn] = i2c3_er_irq_handler,
          [DMAMUX_OVR_IRQn] = dmamux_ovr_irq_handler,
          [QUADSPI_IRQn] = quadspi_irq_handler,
          [DMA1_Channel8_IRQn] = dma1_channel8_irq_handler,
          [DMA2_Channel6_IRQn] = dma2_channel6_irq_handler,
          [DMA2_Channel7_IRQn] = dma2_channel7_irq_handler,
          [DMA2_Channel8_IRQn] = dma2_channel7_irq_handler,
          [CORDIC_IRQn] = cordic_irq_handler,
          [FMAC_IRQn] = fmac_irq_handler,
          }
};
#endif
