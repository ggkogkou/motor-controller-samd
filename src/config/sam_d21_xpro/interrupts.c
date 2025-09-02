/*******************************************************************************
 System Interrupts File

  Company:
    Microchip Technology Inc.

  File Name:
    interrupt.c

  Summary:
    Interrupt vectors mapping

  Description:
    This file maps all the interrupt vectors to their corresponding
    implementations. If a particular module interrupt is used, then its ISR
    definition can be found in corresponding PLIB source file. If a module
    interrupt is not used, then its ISR implementation is mapped to dummy
    handler.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
* Copyright (C) 2018 Microchip Technology Inc. and its subsidiaries.
*
* Subject to your compliance with these terms, you may use Microchip software
* and any derivatives exclusively with Microchip products. It is your
* responsibility to comply with third party license terms applicable to your
* use of third party software (including open source software) that may
* accompany Microchip software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
* EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
* WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
* PARTICULAR PURPOSE.
*
* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
* BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
* FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
* ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *******************************************************************************/
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include "interrupts.h"
#include "definitions.h"

// *****************************************************************************
// *****************************************************************************
// Section: System Interrupt Vector Functions
// *****************************************************************************
// *****************************************************************************

/* MISRA C-2012 Rule 8.6 deviated below. Deviation record ID -  H3_MISRAC_2012_R_8_6_DR_1 */
extern uint32_t __StackTop;

extern void Dummy_Handler(void);

/* Brief default interrupt handler for unused IRQs.*/
void __attribute__((optimize("-O1"), long_call, noreturn, used))Dummy_Handler(void)
{
    while (true)
    {
    }
}

/* MISRAC 2012 deviation block start */
/* MISRA C-2012 Rule 8.6 deviated 29 times.  Deviation record ID -  H3_MISRAC_2012_R_8_6_DR_1 */
/* Device vectors list dummy definition*/
extern void SVCall_Handler             ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void PendSV_Handler             ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void PM_Handler                 ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void SYSCTRL_Handler            ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void WDT_Handler                ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void RTC_Handler                ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void EIC_Handler                ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void NVMCTRL_Handler            ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void DMAC_Handler               ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void USB_Handler                ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void EVSYS_Handler              ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void SERCOM0_Handler            ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void SERCOM1_Handler            ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void SERCOM2_Handler            ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void SERCOM3_Handler            ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void SERCOM4_Handler            ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void SERCOM5_Handler            ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void TCC1_Handler               ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void TCC2_Handler               ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void TC3_Handler                ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void TC4_Handler                ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void TC5_Handler                ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void TC6_Handler                ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void TC7_Handler                ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void ADC_Handler                ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void AC_Handler                 ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void DAC_Handler                ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void PTC_Handler                ( void ) __attribute__((weak, alias("Dummy_Handler")));
extern void I2S_Handler                ( void ) __attribute__((weak, alias("Dummy_Handler")));

/* MISRAC 2012 deviation block end */

/* Multiple handlers for vector */

/* ARM GCC Compatible Vector Table - replaces H3DeviceVectors struct */
typedef void (*pFunc)(void);

__attribute__ ((section(".vectors"), used))
const pFunc g_pfnVectors[] =
{
    /* Configure Initial Stack Pointer, using linker-generated symbols */
    (pFunc)((uint32_t)&__StackTop),

    Reset_Handler,
    NonMaskableInt_Handler,
    HardFault_Handler,
    0, 0, 0, 0, 0, 0, 0,                    /* Reserved */
    SVCall_Handler,
    0, 0,                                   /* Reserved */
    PendSV_Handler,
    SysTick_Handler,

    /* Peripheral Interrupts */
    PM_Handler,
    SYSCTRL_Handler,
    WDT_Handler,
    RTC_Handler,
    EIC_Handler,
    NVMCTRL_Handler,
    DMAC_Handler,
    USB_Handler,
    EVSYS_Handler,
    SERCOM0_Handler,
    SERCOM1_Handler,
    SERCOM2_Handler,
    SERCOM3_Handler,
    SERCOM4_Handler,
    SERCOM5_Handler,
    TCC0_InterruptHandler,                  /* Keep your custom handler name */
    TCC1_Handler,
    TCC2_Handler,
    TC3_Handler,
    TC4_Handler,
    TC5_Handler,
    TC6_Handler,
    TC7_Handler,
    ADC_Handler,
    AC_Handler,
    DAC_Handler,
    PTC_Handler,
    I2S_Handler,
};

/*******************************************************************************
 End of File
*/