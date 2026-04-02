#pragma once

#include "samd21g18a.h"

extern "C" {

/**
 * ARM Cortex-M0+ CPU peripherals
 */
[[noreturn]] void Reset_Handler(void);
[[noreturn]] void NonMaskableInt_Handler(void);
[[noreturn]] void HardFault_Handler(void);
void SVCall_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);

/**
 * SAMD21 unused peripheral handlers
 */
void PM_Handler(void);
void SYSCTRL_Handler(void);
void WDT_Handler(void);
void RTC_Handler(void);
void EIC_Handler(void);
void NVMCTRL_Handler(void);
void USB_Handler(void);
void EVSYS_Handler(void);
void SERCOM0_Handler(void);
void SERCOM1_Handler(void);
void SERCOM2_Handler(void);
void SERCOM3_Handler(void);
void SERCOM4_Handler(void);
void SERCOM5_Handler(void);
void TCC0_Handler(void);
void TCC1_Handler(void);
void TCC2_Handler(void);
void TC5_Handler(void);
void TC6_Handler(void);
void TC7_Handler(void);
void AC_Handler(void);
void DAC_Handler(void);
void PTC_Handler(void);
void I2S_Handler(void);

/**
 * SAMD21 peripheral handlers
 */
void DMAC_InterruptHandler(void);
void SERCOM3_USART_InterruptHandler(void);
void SERCOM4_USART_InterruptHandler(void);
void SERCOM5_SPI_InterruptHandler(void);
void TC3_TimerInterruptHandler(void);
void TC4_TimerInterruptHandler(void);
void ADC_InterruptHandler(void);
}
