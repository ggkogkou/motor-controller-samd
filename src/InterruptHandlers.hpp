#pragma once

#include <cstdint>
#include "device_vectors.h"

#ifdef __cplusplus
extern "C" {
#endif

extern uint32_t __stack;
extern const H3DeviceVectors exception_table;

void Reset_Handler();
void NonMaskableInt_Handler();
void HardFault_Handler();
void SysTick_Handler();
void DMAC_InterruptHandler();
void SERCOM3_USART_InterruptHandler();
void SERCOM5_SPI_InterruptHandler();
void TC3_TimerInterruptHandler();
void TC4_TimerInterruptHandler();
void ADC_InterruptHandler();

/* Brief default interrupt handlers for core IRQs.*/
void __attribute__((noreturn, weak)) NonMaskableInt_Handler(void) {
#if defined(__DEBUG) || defined(__DEBUG_D)
        __builtin_software_breakpoint();
#endif
        while (true) {
        }
}

void __attribute__((noreturn, weak)) HardFault_Handler(void) {
#if defined(__DEBUG) || defined(__DEBUG_D)
        __builtin_software_breakpoint();
#endif
        while (true) {
        }
}

#ifdef __cplusplus
}
#endif
