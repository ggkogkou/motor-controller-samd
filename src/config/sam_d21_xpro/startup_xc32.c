/*******************************************************************************
 * ARM GCC Startup File for SAMD21J18A
 * Modern CMSIS-style with MPLAB Harmony peripheral compatibility
 * No XC32 dependencies
 *******************************************************************************/

#include "device.h"
#include "definitions.h"

/*----------------------------------------------------------------------------
 * Linker-provided symbols
 *----------------------------------------------------------------------------*/
extern uint32_t __StackTop;
extern uint32_t __copy_table_start__;
extern uint32_t __copy_table_end__;
extern uint32_t __zero_table_start__;
extern uint32_t __zero_table_end__;

/*----------------------------------------------------------------------------
 * External function prototypes
 *----------------------------------------------------------------------------*/
extern int main(void);
extern void __libc_init_array(void);

/*----------------------------------------------------------------------------
 * Internal function prototypes
 *----------------------------------------------------------------------------*/
void Reset_Handler(void) __attribute__((noreturn));
void Default_Handler(void);

/*----------------------------------------------------------------------------
 * Exception handlers (use existing MPLAB handlers where available)
 *----------------------------------------------------------------------------*/
void NMI_Handler(void) __attribute__((weak, alias("Default_Handler")));
void HardFault_Handler(void) __attribute__((weak, alias("Default_Handler")));
void SVC_Handler(void) __attribute__((weak, alias("Default_Handler")));
void PendSV_Handler(void) __attribute__((weak, alias("Default_Handler")));
void SysTick_Handler(void) __attribute__((weak, alias("Default_Handler")));

/* SAMD21 Peripheral Handlers - use MPLAB naming */
void PM_Handler(void) __attribute__((weak, alias("Default_Handler")));
void SYSCTRL_Handler(void) __attribute__((weak, alias("Default_Handler")));
void WDT_Handler(void) __attribute__((weak, alias("Default_Handler")));
void RTC_Handler(void) __attribute__((weak, alias("Default_Handler")));
void EIC_Handler(void) __attribute__((weak, alias("Default_Handler")));
void NVMCTRL_Handler(void) __attribute__((weak, alias("Default_Handler")));
void DMAC_Handler(void) __attribute__((weak, alias("Default_Handler")));
void USB_Handler(void) __attribute__((weak, alias("Default_Handler")));
void EVSYS_Handler(void) __attribute__((weak, alias("Default_Handler")));
void SERCOM0_Handler(void) __attribute__((weak, alias("Default_Handler")));
void SERCOM1_Handler(void) __attribute__((weak, alias("Default_Handler")));
void SERCOM2_Handler(void) __attribute__((weak, alias("Default_Handler")));
void SERCOM3_Handler(void) __attribute__((weak, alias("Default_Handler")));
void SERCOM4_Handler(void) __attribute__((weak, alias("Default_Handler")));
void SERCOM5_Handler(void) __attribute__((weak, alias("Default_Handler")));
void TCC0_Handler(void) __attribute__((weak, alias("Default_Handler")));
void TCC1_Handler(void) __attribute__((weak, alias("Default_Handler")));
void TCC2_Handler(void) __attribute__((weak, alias("Default_Handler")));
void TC3_Handler(void) __attribute__((weak, alias("Default_Handler")));
void TC4_Handler(void) __attribute__((weak, alias("Default_Handler")));
void TC5_Handler(void) __attribute__((weak, alias("Default_Handler")));
void TC6_Handler(void) __attribute__((weak, alias("Default_Handler")));
void TC7_Handler(void) __attribute__((weak, alias("Default_Handler")));
void ADC_Handler(void) __attribute__((weak, alias("Default_Handler")));
void AC_Handler(void) __attribute__((weak, alias("Default_Handler")));
void DAC_Handler(void) __attribute__((weak, alias("Default_Handler")));
void PTC_Handler(void) __attribute__((weak, alias("Default_Handler")));
void I2S_Handler(void) __attribute__((weak, alias("Default_Handler")));

/*----------------------------------------------------------------------------
 * Vector Table
 *----------------------------------------------------------------------------*/
typedef void (*pFunc)(void);

__attribute__((section(".vectors"), used))
const pFunc g_pfnVectors[] = {
    /* Core exceptions */
    (pFunc)((uint32_t)&__StackTop),    /* Initial stack pointer */
    Reset_Handler,                     /* Reset Handler */
    NMI_Handler,                       /* NMI Handler */
    HardFault_Handler,                 /* Hard Fault Handler */
    0,                                 /* Reserved */
    0,                                 /* Reserved */
    0,                                 /* Reserved */
    0,                                 /* Reserved */
    0,                                 /* Reserved */
    0,                                 /* Reserved */
    0,                                 /* Reserved */
    SVC_Handler,                       /* SVCall Handler */
    0,                                 /* Reserved */
    0,                                 /* Reserved */
    PendSV_Handler,                    /* PendSV Handler */
    SysTick_Handler,                   /* SysTick Handler */

    /* SAMD21 Peripheral interrupts */
    PM_Handler,                        /* 0  Power Manager */
    SYSCTRL_Handler,                   /* 1  System Controller */
    WDT_Handler,                       /* 2  Watchdog Timer */
    RTC_Handler,                       /* 3  Real Time Counter */
    EIC_Handler,                       /* 4  External Interrupt Controller */
    NVMCTRL_Handler,                   /* 5  Non-Volatile Memory Controller */
    DMAC_Handler,                      /* 6  Direct Memory Controller */
    USB_Handler,                       /* 7  Universal Serial Bus */
    EVSYS_Handler,                     /* 8  Event System */
    SERCOM0_Handler,                   /* 9  Serial Communication Interface 0 */
    SERCOM1_Handler,                   /* 10 Serial Communication Interface 1 */
    SERCOM2_Handler,                   /* 11 Serial Communication Interface 2 */
    SERCOM3_Handler,                   /* 12 Serial Communication Interface 3 */
    SERCOM4_Handler,                   /* 13 Serial Communication Interface 4 */
    SERCOM5_Handler,                   /* 14 Serial Communication Interface 5 */
    TCC0_Handler,                      /* 15 Timer/Counter Control 0 */
    TCC1_Handler,                      /* 16 Timer/Counter Control 1 */
    TCC2_Handler,                      /* 17 Timer/Counter Control 2 */
    TC3_Handler,                       /* 18 Timer/Counter 3 */
    TC4_Handler,                       /* 19 Timer/Counter 4 */
    TC5_Handler,                       /* 20 Timer/Counter 5 */
    TC6_Handler,                       /* 21 Timer/Counter 6 */
    TC7_Handler,                       /* 22 Timer/Counter 7 */
    ADC_Handler,                       /* 23 Analog-to-Digital Converter */
    AC_Handler,                        /* 24 Analog Comparators */
    DAC_Handler,                       /* 25 Digital-to-Analog Converter */
    PTC_Handler,                       /* 26 Peripheral Touch Controller */
    I2S_Handler,                       /* 27 Inter-IC Sound */
};

/*----------------------------------------------------------------------------
 * Reset Handler - Modern CMSIS with SAMD21 optimizations
 *----------------------------------------------------------------------------*/
void Reset_Handler(void)
{
    /* CMSIS table-driven data initialization */
    uint32_t *pTable = &__copy_table_start__;
    for (; pTable < &__copy_table_end__; pTable += 3) {
        uint32_t *pSrc  = (uint32_t *)pTable[0];   /* source address */
        uint32_t *pDest = (uint32_t *)pTable[1];   /* destination address */
        uint32_t  count = pTable[2];               /* word count */
        
        for (uint32_t i = 0; i < count; i++) {
            pDest[i] = pSrc[i];
        }
    }

    /* CMSIS table-driven zero initialization */
    pTable = &__zero_table_start__;
    for (; pTable < &__zero_table_end__; pTable += 2) {
        uint32_t *pDest = (uint32_t *)pTable[0];   /* destination address */
        uint32_t  count = pTable[1];               /* word count */
        
        for (uint32_t i = 0; i < count; i++) {
            pDest[i] = 0;
        }
    }

    /* SAMD21-specific hardware optimizations (from MPLAB) */
    /* Change default QOS values for best performance and USB behavior */
    SBMATRIX_REGS->HMATRIXB_SFR[4] = 2;
#if defined(ID_USB)
    USB_REGS->DEVICE.USB_QOSCTRL = 0xA;
#endif
    DMAC_REGS->DMAC_QOSCTRL = 0x2A;
    
    /* Fix for NVMCTRL.CTRLB.MANW bit (errata reference 13134) */
    NVMCTRL_REGS->NVMCTRL_CTRLB |= 1 << 7;

    /* Initialize C library */
    __libc_init_array();

    /* Call main function */
    main();

    /* Infinite loop */
    while (1) {
        /* Should never reach here */
    }
}

/*----------------------------------------------------------------------------
 * Default Handler
 *----------------------------------------------------------------------------*/
void Default_Handler(void)
{
    while (1) {
        /* Stay here */
    }
}