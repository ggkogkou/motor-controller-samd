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
#include "InterruptHandlers.hpp"
/*
 * ARM GCC startup code for SAMD21J18A
 * Uses traditional GCC startup initialization
 * Compatible with existing MPLAB Harmony peripheral drivers
 */

extern const H3DeviceVectors exception_table;
/**
 * \brief ARM GCC Reset Handler with CMSIS table-driven initialization
 * Replaces XC32-specific initialization with ARM GCC compatible version
 * Maintains all SAMD21-specific hardware optimizations
 */
void __attribute__((optimize("-O1"), section(".text.Reset_Handler"), long_call, noreturn)) Reset_Handler(void) {

#ifdef SCB_VTOR_TBLOFF_Msk
        /*  Set the vector-table base address in FLASH */
        SCB->VTOR = ((uint32_t)&exception_table & SCB_VTOR_TBLOFF_Msk);
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

        /* Hand off to CMSIS program start: copy/zero tables + C runtime + main */
        __PROGRAM_START();

#if (defined(__DEBUG) || defined(__DEBUG_D))
        __builtin_software_breakpoint();
#endif

        while (true) {
                /* Infinite loop */
        }
}
