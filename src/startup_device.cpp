#include "device.h"
#include "device_vectors.h"

extern const H3DeviceVectors exception_table;

extern "C" void Reset_Handler(void) __attribute__((noreturn));

extern "C" void Reset_Handler(void) {
#ifdef SCB_VTOR_TBLOFF_Msk
        SCB->VTOR = ((uint32_t)&exception_table & SCB_VTOR_TBLOFF_Msk);
#endif

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
