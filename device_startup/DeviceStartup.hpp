#pragma once

#include <cstdint>
#include <type_traits>
#include "InterruptHandlers.hpp"

extern "C" uint32_t __stack;

using isr_t = void (*)();

struct DeviceVectorsSAMD21 {
        /* Stack pointer */
        std::uintptr_t pvStack;

        /* CORTEX-M0PLUS handlers */
        isr_t pfnReset_Handler; /* -15 Reset Vector, invoked on Power up and warm reset */
        isr_t pfnNonMaskableInt_Handler; /* -14 Non maskable Interrupt, cannot be stopped or preempted */
        isr_t pfnHardFault_Handler; /* -13 Hard Fault, all classes of Fault */
        isr_t pvReservedC12;
        isr_t pvReservedC11;
        isr_t pvReservedC10;
        isr_t pvReservedC9;
        isr_t pvReservedC8;
        isr_t pvReservedC7;
        isr_t pvReservedC6;
        isr_t pfnSVCall_Handler; /*  -5 System Service Call via SVC instruction */
        isr_t pvReservedC4;
        isr_t pvReservedC3;
        isr_t pfnPendSV_Handler; /*  -2 Pendable request for system service */
        isr_t pfnSysTick_Handler; /*  -1 System Tick Timer */

        /* Peripheral handlers */
        isr_t pfnPM_Handler; /*   0 Power Manager (PM) */
        isr_t pfnSYSCTRL_Handler; /*   1 System Control (SYSCTRL) */
        isr_t pfnWDT_Handler; /*   2 Watchdog Timer (WDT) */
        isr_t pfnRTC_Handler; /*   3 Real-Time Counter (RTC) */
        isr_t pfnEIC_Handler; /*   4 External Interrupt Controller (EIC) */
        isr_t pfnNVMCTRL_Handler; /*   5 Non-Volatile Memory Controller (NVMCTRL) */
        isr_t pfnDMAC_Handler; /*   6 Direct Memory Access Controller (DMAC) */
        isr_t pfnUSB_Handler; /*   7 Universal Serial Bus (USB) */
        isr_t pfnEVSYS_Handler; /*   8 Event System Interface (EVSYS) */
        isr_t pfnSERCOM0_Handler; /*   9 Serial Communication Interface (SERCOM0) */
        isr_t pfnSERCOM1_Handler; /*  10 Serial Communication Interface (SERCOM1) */
        isr_t pfnSERCOM2_Handler; /*  11 Serial Communication Interface (SERCOM2) */
        isr_t pfnSERCOM3_Handler; /*  12 Serial Communication Interface (SERCOM3) */
        isr_t pfnSERCOM4_Handler; /*  13 Serial Communication Interface (SERCOM4) */
        isr_t pfnSERCOM5_Handler; /*  14 Serial Communication Interface (SERCOM5) */
        isr_t pfnTCC0_Handler; /*  15 Timer Counter Control (TCC0) */
        isr_t pfnTCC1_Handler; /*  16 Timer Counter Control (TCC1) */
        isr_t pfnTCC2_Handler; /*  17 Timer Counter Control (TCC2) */
        isr_t pfnTC3_Handler; /*  18 Basic Timer Counter (TC3) */
        isr_t pfnTC4_Handler; /*  19 Basic Timer Counter (TC4) */
        isr_t pfnTC5_Handler; /*  20 Basic Timer Counter (TC5) */
        isr_t pfnTC6_Handler; /*  21 Basic Timer Counter (TC6) */
        isr_t pfnTC7_Handler; /*  22 Basic Timer Counter (TC7) */
        isr_t pfnADC_Handler; /*  23 Analog Digital Converter (ADC) */
        isr_t pfnAC_Handler; /*  24 Analog Comparators (AC) */
        isr_t pfnDAC_Handler; /*  25 Digital Analog Converter (DAC) */
        isr_t pfnPTC_Handler; /*  26 Peripheral Touch Controller (PTC) */
        isr_t pfnI2S_Handler; /*  27 Inter-IC Sound Interface (I2S) */
};

inline constexpr std::size_t TotalNumberOfIRQs = 44;

static_assert(std::is_standard_layout_v<DeviceVectorsSAMD21>);
static_assert(std::is_trivially_copyable_v<DeviceVectorsSAMD21>);
static_assert(sizeof(DeviceVectorsSAMD21) == TotalNumberOfIRQs * sizeof(std::uintptr_t));
