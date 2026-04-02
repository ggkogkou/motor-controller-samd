// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Georgios Gkogkou <ggkogkou125@gmail.com>

/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

/**
 * @file   main.cpp
 * @brief  Location of the main() function and the superloop
 * @author Georgios Gkogkou <ggkogkou125@gmail.com>
 */

#include <GenericClockController.hpp>

#include "DeviceStartup.hpp"
#include "definitions.h"
#include "GenericClockController.hpp"
#include "HardwareDiagnosticsLogger.hpp"
#include "RadiationTestDemo.hpp"
#include "SAMD21_FOC.hpp"

void initializePeripherals() {
        NVMCTRL_REGS->NVMCTRL_CTRLB = NVMCTRL_CTRLB_RWS(3UL);

        PORT_Initialize();
        GenericClockController::initializePeripheral();
        SERCOM3_USART_Initialize();
        SERCOM4_USART_Initialize();
        NVMCTRL_Initialize();
        EVSYS_Initialize();
        TCC0_PWMInitialize();
        SYSTICK_TimerInitialize();
        DMAC_Initialize();
        SERCOM5_SPI_Initialize();
        ADC_Initialize();
        EIC_Initialize();
        TC3_TimerInitialize();
        TC4_TimerInitialize();
        NVIC_Initialize();
}

[[noreturn]] int main() {
        const uint8_t cause = PM_REGS->PM_RCAUSE;

        auto wasItAutomatic = false;
        auto wasPOR = false;

        if (cause & PM_RCAUSE_BOD12_Msk)
                wasItAutomatic = true;

        if (cause & PM_RCAUSE_BOD33_Msk)
                wasItAutomatic = true;

        if (cause & PM_RCAUSE_WDT_Msk)
                wasItAutomatic = true;

        if (cause & PM_RCAUSE_EXT_Msk)
                wasItAutomatic = true;

        if (cause & PM_RCAUSE_POR_Msk) {
                wasPOR = true;
        }

        // initializePeripherals();
        SYS_Initialize(NULL);
        SYSTICK_TimerStart();
        SPI_Buffer::init();

        USART_TxStream diagnostics{DMAC_CHANNEL_1};
        diagnostics.init();
        HardwareDiagnosticsLogger diagnosticsLogger{diagnostics};

        diagnosticsLogger.logBoot();

        if (cause & PM_RCAUSE_POR_Msk)
                diagnosticsLogger.logResetPOR();

        if (cause & PM_RCAUSE_BOD12_Msk)
                diagnosticsLogger.writeLiteral("RESET: BOD12\r\n");

        if (cause & PM_RCAUSE_BOD33_Msk)
                diagnosticsLogger.logResetBOD33();

        if (cause & PM_RCAUSE_WDT_Msk)
                diagnosticsLogger.logResetWDT();

        if (cause & PM_RCAUSE_EXT_Msk)
                diagnosticsLogger.logResetExternal();

        if (cause & PM_RCAUSE_SYST_Msk)
                diagnosticsLogger.logResetSoftware();

        if (cause == 0U)
                diagnosticsLogger.logResetUnknown();

        if (wasPOR) {
                diagnosticsLogger.writeLiteral("STATE: POWER_ON_RESET_BOOT\r\n");
        }

        if (wasItAutomatic) {
                diagnosticsLogger.writeLiteral("STATE: HALT_AFTER_AUTOMATIC_RESET\r\n");

                __disable_irq();
                while (true) {
                        BENCHMARK_IO_Set();
                        for (volatile uint32_t i = 0; i < 300000U; i = i + 1) {
                                __NOP();
                        }
                        BENCHMARK_IO_Clear();
                        for (volatile uint32_t i = 0; i < 300000U; i = i + 1) {
                                __NOP();
                        }
                }
        }

        static constexpr frequency_kHz_t PWM_Frequency = 18.0f;
        static constexpr bool EnableLogging = true;

        USART_TxStream logging{DMAC_CHANNEL_0};
        TelemetryLogger<TelemetryPayload44> telemetry;
        TelemetryLogger<TelemetryPayload44>* telemetryPtr = nullptr;
        if (EnableLogging)
                telemetryPtr = &telemetry;
        SAMD21_FOC foc{PWM_Frequency, telemetryPtr};

        if (telemetryPtr != nullptr)
                logging.init();

        // RadiationTestDemo radiationTestDemo(foc, telemetryPtr);
        RadiationTestDemo radiationTestDemo(foc, telemetryPtr, &diagnosticsLogger);
        radiationTestDemo.start();

        diagnosticsLogger.writeLiteral("STATE: MAIN_LOOP_ENTERED\r\n");

        while (true) {
                if (radiationTestDemo.loggingEnabled())
                        telemetry.writeFrame(logging);

                SYSTICK_DelayMs(3);
        }
}
