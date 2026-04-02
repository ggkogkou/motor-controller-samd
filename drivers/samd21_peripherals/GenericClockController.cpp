#include "GenericClockController.hpp"

void GenericClockController::initialize_SYSCTRL() {
        /****************** OSC32K Initialization  ******************************/
        uint32_t calibValue = (uint32_t)(((*(uint64_t*)0x806020U) >> 38U) & 0x7FU);

        /* Configure 32K RC oscillator */
        SYSCTRL_REGS->SYSCTRL_OSC32K =
                SYSCTRL_OSC32K_CALIB(calibValue) | SYSCTRL_OSC32K_STARTUP(0U) | SYSCTRL_OSC32K_ENABLE_Msk | SYSCTRL_OSC32K_EN32K_Msk;

        while (!((SYSCTRL_REGS->SYSCTRL_PCLKSR & SYSCTRL_PCLKSR_OSC32KRDY_Msk) == SYSCTRL_PCLKSR_OSC32KRDY_Msk)) {
                /* Waiting for the OSC32K Ready state */
        }
}

void GenericClockController::initialize_DFLL() {
        /****************** DFLL Initialization  *********************************/
        SYSCTRL_REGS->SYSCTRL_DFLLCTRL &= (uint16_t)(~SYSCTRL_DFLLCTRL_ONDEMAND_Msk);

        while ((SYSCTRL_REGS->SYSCTRL_PCLKSR & SYSCTRL_PCLKSR_DFLLRDY_Msk) != SYSCTRL_PCLKSR_DFLLRDY_Msk) {
                /* Waiting for the Ready state */
        }

        /* Load Calibration Value */
        uint32_t calibCoarse = (((*((uint32_t*)0x00806020U + 1U)) >> 26U) & 0x3FU);
        calibCoarse = ((calibCoarse == 0x3FU) ? 0x1FU : calibCoarse);

        SYSCTRL_REGS->SYSCTRL_DFLLVAL = SYSCTRL_DFLLVAL_COARSE(calibCoarse) | SYSCTRL_DFLLVAL_FINE(512U);

        while ((SYSCTRL_REGS->SYSCTRL_PCLKSR & SYSCTRL_PCLKSR_DFLLRDY_Msk) != SYSCTRL_PCLKSR_DFLLRDY_Msk) {
                /* Waiting for the Ready state */
        }

        /* Configure DFLL */
        SYSCTRL_REGS->SYSCTRL_DFLLCTRL = SYSCTRL_DFLLCTRL_ENABLE_Msk;

        while ((SYSCTRL_REGS->SYSCTRL_PCLKSR & SYSCTRL_PCLKSR_DFLLRDY_Msk) != SYSCTRL_PCLKSR_DFLLRDY_Msk) {
                /* Waiting for DFLL to be ready */
        }
}

void GenericClockController::initialize_GCLK0() {
        GCLK_REGS->GCLK_GENCTRL = GCLK_GENCTRL_SRC(7U) | GCLK_GENCTRL_GENEN_Msk | GCLK_GENCTRL_ID(0U);

        while ((GCLK_REGS->GCLK_STATUS & GCLK_STATUS_SYNCBUSY_Msk) == GCLK_STATUS_SYNCBUSY_Msk) {
                /* wait for the Generator 0 synchronization */
        }
}

void GenericClockController::initialize_GCLK1() {
        GCLK_REGS->GCLK_GENCTRL = GCLK_GENCTRL_SRC(4U) | GCLK_GENCTRL_GENEN_Msk | GCLK_GENCTRL_ID(1U);

        while ((GCLK_REGS->GCLK_STATUS & GCLK_STATUS_SYNCBUSY_Msk) == GCLK_STATUS_SYNCBUSY_Msk) {
                /* wait for the Generator 1 synchronization */
        }
}

void GenericClockController::initialize_GCLK2() {
        GCLK_REGS->GCLK_GENCTRL = GCLK_GENCTRL_SRC(7U) | GCLK_GENCTRL_GENEN_Msk | GCLK_GENCTRL_ID(2U);

        GCLK_REGS->GCLK_GENDIV = GCLK_GENDIV_DIV(6U) | GCLK_GENDIV_ID(2U);

        while ((GCLK_REGS->GCLK_STATUS & GCLK_STATUS_SYNCBUSY_Msk) == GCLK_STATUS_SYNCBUSY_Msk) {
                /* wait for the Generator 2 synchronization */
        }
}

void GenericClockController::initializePeripheral() {
        /* Function to Initialize the Oscillators */
        initialize_SYSCTRL();

        initialize_DFLL();
        initialize_GCLK1();
        initialize_GCLK0();
        initialize_GCLK2();

        /* Selection of the Generator and write Lock for EIC */
        GCLK_REGS->GCLK_CLKCTRL = GCLK_CLKCTRL_ID(5U) | GCLK_CLKCTRL_GEN(0x0U) | GCLK_CLKCTRL_CLKEN_Msk;

        /* Selection of the Generator and write Lock for EVSYS_0 */
        GCLK_REGS->GCLK_CLKCTRL = GCLK_CLKCTRL_ID(7U) | GCLK_CLKCTRL_GEN(0x0U) | GCLK_CLKCTRL_CLKEN_Msk;

        /* Selection of the Generator and write Lock for SERCOM3_CORE */
        GCLK_REGS->GCLK_CLKCTRL = GCLK_CLKCTRL_ID(23U) | GCLK_CLKCTRL_GEN(0x0U) | GCLK_CLKCTRL_CLKEN_Msk;

        /* Selection of the Generator and write Lock for SERCOM4_CORE */
        GCLK_REGS->GCLK_CLKCTRL = GCLK_CLKCTRL_ID(24U) | GCLK_CLKCTRL_GEN(0x0U) | GCLK_CLKCTRL_CLKEN_Msk;

        /* Selection of the Generator and write Lock for SERCOM5_CORE */
        GCLK_REGS->GCLK_CLKCTRL = GCLK_CLKCTRL_ID(25U) | GCLK_CLKCTRL_GEN(0x0U) | GCLK_CLKCTRL_CLKEN_Msk;

        /* Selection of the Generator and write Lock for TCC0 TCC1 */
        GCLK_REGS->GCLK_CLKCTRL = GCLK_CLKCTRL_ID(26U) | GCLK_CLKCTRL_GEN(0x0U) | GCLK_CLKCTRL_CLKEN_Msk;

        /* Selection of the Generator and write Lock for TC3 TCC2 */
        GCLK_REGS->GCLK_CLKCTRL = GCLK_CLKCTRL_ID(27U) | GCLK_CLKCTRL_GEN(0x0U) | GCLK_CLKCTRL_CLKEN_Msk;

        /* Selection of the Generator and write Lock for TC4 TC5 */
        GCLK_REGS->GCLK_CLKCTRL = GCLK_CLKCTRL_ID(28U) | GCLK_CLKCTRL_GEN(0x2U) | GCLK_CLKCTRL_CLKEN_Msk;

        /* Selection of the Generator and write Lock for ADC */
        GCLK_REGS->GCLK_CLKCTRL = GCLK_CLKCTRL_ID(30U) | GCLK_CLKCTRL_GEN(0x2U) | GCLK_CLKCTRL_CLKEN_Msk;

        /* Configure the APBC Bridge Clocks */
        PM_REGS->PM_APBCMASK = 0x139a2U;

        /* Disable RC oscillator */
        SYSCTRL_REGS->SYSCTRL_OSC8M = 0x0U;
}
