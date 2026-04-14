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
#include "HardwareDiagnosticsLogger.hpp"
#include "RadiationTestDemo.hpp"
#include "ResetEventMonitor.hpp"
#include "SAMD21_FOC.hpp"
#include "definitions.h"

inline constexpr frequency_kHz_t PWM_Frequency = 17.0f;
inline constexpr bool EnableLogging = false;

static volatile bool extWdtWakeFlag = true;

static void extWdtWakeCallback(uintptr_t) {
        extWdtWakeFlag = true;
}

[[noreturn]] int main() {
        SYS_Initialize(NULL);
        SYSTICK_TimerStart();
        SPI_Buffer::init();

        HardwareDiagnostics::diagnostics.init();
        HardwareDiagnostics::diagnosticsLogger.logBoot();

        ResetEventMonitor::determineResetCause();

        EIC_CallbackRegister(EIC_PIN_7, extWdtWakeCallback, 0);
        EIC_InterruptEnable(EIC_PIN_7);

        USART_TxStream logging{DMAC_CHANNEL_0};
        TelemetryLogger<TelemetryPayload12> telemetry;
        TelemetryLogger<TelemetryPayload12>* telemetryPtr = nullptr;
        if (EnableLogging)
                telemetryPtr = &telemetry;
        SAMD21_FOC foc{PWM_Frequency, telemetryPtr};

        if (telemetryPtr != nullptr)
                logging.init();

        RadiationTestDemo radiationTestDemo(foc, telemetryPtr);
        radiationTestDemo.start();

        HardwareDiagnostics::diagnosticsLogger.writeLiteral("\r\nSTATE: MAIN-LOOP ENTERED\r\n");

        EXT_WDT_DONE_Clear();

        while (true) {
                if constexpr (EnableLogging)
                        telemetry.writeFrame(logging);

                if (extWdtWakeFlag) {
                        extWdtWakeFlag = false;
                        EXT_WDT_DONE_Set();
                        SYSTICK_DelayMs(100);
                        EXT_WDT_DONE_Clear();
                        HardwareDiagnostics::diagnosticsLogger.writeLiteral("EXT WDT KICKED\r\n");
                }

                // SYSTICK_DelayMs(1);
        }
}
