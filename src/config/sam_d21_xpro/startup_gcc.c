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
* ANY WAY RELATED TO THIS SOFTWARE, WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
*******************************************************************************/
// DOM-IGNORE-END

#include <stdbool.h>
#include <stddef.h>
#include "device.h"
#include "interrupts.h"

/*
 * ARM GCC startup code for SAMD21J18A
 * Uses CMSIS table-driven initialization
 * Compatible with existing MPLAB Harmony peripheral drivers
 */

/* CMSIS table symbols from linker script */
extern uint32_t __copy_table_start__;
extern uint32_t __copy_table_end__;
extern uint32_t __zero_table_start__;
extern uint32_t __zero_table_end__;

/* Legacy Harmony symbols for compatibility */
extern uint32_t _sfixed;

/* array initialization function */
extern void __attribute__((long_call)) __libc_init_array(void);

/* Optional application-provided functions - keep for compatibility */
extern void __attribute__((weak,long_call, alias("Dummy_App_Func"))) _on_reset(void);
extern void __attribute__((weak,long_call, alias("Dummy_App_Func"))) _on_bootstrap(void);

extern int main(void);

/* Brief default application function used as a weak reference */
extern void Dummy_App_Func(void);
void __attribute__((optimize("-O1"),long_call))Dummy_App_Func(void)
{
    /* Do nothing */
    return;
}

/**
 * \brief ARM GCC Reset Handler with CMSIS table-driven initialization
 * Replaces XC32-specific initialization with ARM GCC compatible version
 * Maintains all SAMD21-specific hardware optimizations
 */
void __attribute__((optimize("-O1"), section(".text.Reset_Handler"), long_call, noreturn)) Reset_Handler(void)
{
    /* Call the optional application-provided _on_reset() function. */
    _on_reset();

    /* CMSIS table-driven data initialization - replaces __pic32c_data_initialization() */
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

#ifdef SCB_VTOR_TBLOFF_Msk
    /*  Set the vector-table base address in FLASH */
    uint32_t *pSrc = (uint32_t *) &_sfixed;
    SCB->VTOR = ((uint32_t) pSrc & SCB_VTOR_TBLOFF_Msk);
#endif /* SCB_VTOR_TBLOFF_Msk */

    /* SAMD21-specific hardware optimizations - keep from original MPLAB */
    /* Change default QOS values to have the best performance and correct USB behaviour */
    SBMATRIX_REGS->HMATRIXB_SFR[4] = 2;
#if defined(ID_USB)
    USB_REGS->DEVICE.USB_QOSCTRL = 0xA;
#endif
    DMAC_REGS->DMAC_QOSCTRL = 0x2A;

    /* Overwriting the default value of the NVMCTRL.CTRLB.MANW bit (errata reference 13134) */
    NVMCTRL_REGS->NVMCTRL_CTRLB |= 1 << 7;

    /* Initialize the C library */
    __libc_init_array();

    /* Call the optional application-provided _on_bootstrap() function. */
    _on_bootstrap();

    /* Branch to application's main function */
    (void)main();

#if (defined(__DEBUG) || defined(__DEBUG_D)) && defined(__XC32)
    __builtin_software_breakpoint();
#endif

    while (true)
    {
        /* Infinite loop */
    }
}