#include "InterruptHandlers.hpp"
#include "device.h"

extern "C" {
void DefaultHandler(void);

void SVCall_Handler(void) __attribute__((weak, alias("DefaultHandler")));
void PendSV_Handler(void) __attribute__((weak, alias("DefaultHandler")));
void PM_Handler(void) __attribute__((weak, alias("DefaultHandler")));
void SYSCTRL_Handler(void) __attribute__((weak, alias("DefaultHandler")));
void WDT_Handler(void) __attribute__((weak, alias("DefaultHandler")));
void RTC_Handler(void) __attribute__((weak, alias("DefaultHandler")));
void EIC_Handler(void) __attribute__((weak, alias("DefaultHandler")));
void NVMCTRL_Handler(void) __attribute__((weak, alias("DefaultHandler")));
void USB_Handler(void) __attribute__((weak, alias("DefaultHandler")));
void EVSYS_Handler(void) __attribute__((weak, alias("DefaultHandler")));
void SERCOM0_Handler(void) __attribute__((weak, alias("DefaultHandler")));
void SERCOM1_Handler(void) __attribute__((weak, alias("DefaultHandler")));
void SERCOM2_Handler(void) __attribute__((weak, alias("DefaultHandler")));
void SERCOM4_Handler(void) __attribute__((weak, alias("DefaultHandler")));
void TCC0_Handler(void) __attribute__((weak, alias("DefaultHandler")));
void TCC1_Handler(void) __attribute__((weak, alias("DefaultHandler")));
void TCC2_Handler(void) __attribute__((weak, alias("DefaultHandler")));
void TC5_Handler(void) __attribute__((weak, alias("DefaultHandler")));
void TC6_Handler(void) __attribute__((weak, alias("DefaultHandler")));
void TC7_Handler(void) __attribute__((weak, alias("DefaultHandler")));
void AC_Handler(void) __attribute__((weak, alias("DefaultHandler")));
void DAC_Handler(void) __attribute__((weak, alias("DefaultHandler")));
void PTC_Handler(void) __attribute__((weak, alias("DefaultHandler")));
void I2S_Handler(void) __attribute__((weak, alias("DefaultHandler")));
}

extern "C" void DefaultHandler(void) {
        __disable_irq();
        NVIC_SystemReset();
}

__attribute__((section(".vectors"), used)) const H3DeviceVectors exception_table = {
        /* Configure Initial Stack Pointer, using linker-generated symbols */
        .pvStack = &__stack,

        .pfnReset_Handler = Reset_Handler,
        .pfnNonMaskableInt_Handler = NonMaskableInt_Handler,
        .pfnHardFault_Handler = HardFault_Handler,
        .pfnSVCall_Handler = SVCall_Handler,
        .pfnPendSV_Handler = PendSV_Handler,
        .pfnSysTick_Handler = SysTick_Handler,
        .pfnPM_Handler = PM_Handler,
        .pfnSYSCTRL_Handler = SYSCTRL_Handler,
        .pfnWDT_Handler = WDT_Handler,
        .pfnRTC_Handler = RTC_Handler,
        .pfnEIC_Handler = EIC_Handler,
        .pfnNVMCTRL_Handler = NVMCTRL_Handler,
        .pfnDMAC_Handler = DMAC_InterruptHandler,
        .pfnUSB_Handler = USB_Handler,
        .pfnEVSYS_Handler = EVSYS_Handler,
        .pfnSERCOM0_Handler = SERCOM0_Handler,
        .pfnSERCOM1_Handler = SERCOM1_Handler,
        .pfnSERCOM2_Handler = SERCOM2_Handler,
        .pfnSERCOM3_Handler = SERCOM3_USART_InterruptHandler,
        .pfnSERCOM4_Handler = SERCOM4_Handler,
        .pfnSERCOM5_Handler = SERCOM5_SPI_InterruptHandler,
        .pfnTCC0_Handler = TCC0_Handler,
        .pfnTCC1_Handler = TCC1_Handler,
        .pfnTCC2_Handler = TCC2_Handler,
        .pfnTC3_Handler = TC3_TimerInterruptHandler,
        .pfnTC4_Handler = TC4_TimerInterruptHandler,
        .pfnTC5_Handler = TC5_Handler,
        .pfnTC6_Handler = TC6_Handler,
        .pfnTC7_Handler = TC7_Handler,
        .pfnADC_Handler = ADC_InterruptHandler,
        .pfnAC_Handler = AC_Handler,
        .pfnDAC_Handler = DAC_Handler,
        .pfnPTC_Handler = PTC_Handler,
        .pfnI2S_Handler = I2S_Handler,
};

