#include "DeviceStartup.hpp"

extern "C" void __PROGRAM_START();

__attribute__((section(".vectors"), used)) const DeviceVectorsSAMD21 deviceVectors = {
        /* Configure Initial Stack Pointer */
        .pvStack = reinterpret_cast<std::uintptr_t>(&__stack),

        .pfnReset_Handler = Reset_Handler,
        .pfnNonMaskableInt_Handler = NonMaskableInt_Handler,
        .pfnHardFault_Handler = HardFault_Handler,
        .pvReservedC12 = nullptr,
        .pvReservedC11 = nullptr,
        .pvReservedC10 = nullptr,
        .pvReservedC9 = nullptr,
        .pvReservedC8 = nullptr,
        .pvReservedC7 = nullptr,
        .pvReservedC6 = nullptr,
        .pfnSVCall_Handler = SVCall_Handler,
        .pvReservedC4 = nullptr,
        .pvReservedC3 = nullptr,
        .pfnPendSV_Handler = PendSV_Handler,
        .pfnSysTick_Handler = SysTick_Handler,

        .pfnPM_Handler = PM_Handler,
        .pfnSYSCTRL_Handler = SYSCTRL_Handler,
        .pfnWDT_Handler = WDT_Handler,
        .pfnRTC_Handler = RTC_Handler,
        .pfnEIC_Handler = EIC_InterruptHandler,
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

extern "C" [[noreturn]] void Reset_Handler() {
        SCB->VTOR = reinterpret_cast<std::uintptr_t>(&deviceVectors) & SCB_VTOR_TBLOFF_Msk;

        /* Change default QOS values to have the best performance and correct USB behavior */
        SBMATRIX_REGS->HMATRIXB_SFR[4] = 2;

        USB_REGS->DEVICE.USB_QOSCTRL = 0xA;
        DMAC_REGS->DMAC_QOSCTRL = 0x2A;

        /* Overwriting the default value of the NVMCTRL.CTRLB.MANW bit (errata reference 13134) */
        NVMCTRL_REGS->NVMCTRL_CTRLB |= (1u << 7);

        /* CMSIS runtime entry */
        __PROGRAM_START();
}
