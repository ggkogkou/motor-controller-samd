#include "InterruptHandlers.hpp"

#include <plib_port.h>
#include "ResetBreadcrumb.hpp"

extern "C" {

[[noreturn]] void DefaultHandler() {
        __disable_irq();
        ResetBreadcrumb::recordDefaultHandler(__get_IPSR(), __get_PRIMASK());
        NVIC_SystemReset();
}

[[noreturn]] void NonMaskableInt_Handler(void) __attribute__((weak, alias("DefaultHandler")));
[[noreturn]] void HardFault_Handler(void) __attribute__((weak, alias("DefaultHandler")));
void SVCall_Handler(void) __attribute__((weak, alias("DefaultHandler")));
void PendSV_Handler(void) __attribute__((weak, alias("DefaultHandler")));
void SysTick_Handler(void) __attribute__((weak, alias("DefaultHandler")));

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
void SERCOM3_Handler(void) __attribute__((weak, alias("DefaultHandler")));
void SERCOM4_Handler(void) __attribute__((weak, alias("DefaultHandler")));
void SERCOM5_Handler(void) __attribute__((weak, alias("DefaultHandler")));
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
