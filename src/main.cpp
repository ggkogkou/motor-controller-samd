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
#include "GenericClockController.hpp"
#include "RadiationTestDemo.hpp"
#include "SAMD21_FOC.hpp"

void initializePeripherals() {
        NVMCTRL_REGS->NVMCTRL_CTRLB = NVMCTRL_CTRLB_RWS(3UL);

        PORT_Initialize();
        GenericClockController::initializePeripheral();
        SERCOM3_USART_Initialize();
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
        uint8_t cause = PM_REGS->PM_RCAUSE;

        auto wasItAutomatic = false;
        auto wasPOR = false;

        if (cause & PM_RCAUSE_BOD12_Msk)
                wasItAutomatic = true;

        if (cause & PM_RCAUSE_BOD33_Msk)
                wasItAutomatic = true;

        if (cause & PM_RCAUSE_WDT_Msk)
                wasItAutomatic = true;

        if (cause & PM_RCAUSE_POR_Msk) {
                wasPOR = true;
        }

        if (wasItAutomatic) {
                __disable_irq();
        }

        initializePeripherals();

        if (wasItAutomatic) {
                __disable_irq();
                while (true) {
                        BENCHMARK_IO_Set();
                        SYSTICK_DelayMs(100);
                        BENCHMARK_IO_Clear();
                }
        }

        SYSTICK_TimerStart();
        SPI_Buffer::init();

        static constexpr frequency_kHz_t PWM_Frequency = 19.0f;
        static constexpr bool EnableLogging = true;

        USART_TxStream logging;
        TelemetryLogger<TelemetryPayload44> telemetry;
        TelemetryLogger<TelemetryPayload44>* telemetryPtr = nullptr;
        if (EnableLogging)
                telemetryPtr = &telemetry;
        SAMD21_FOC foc{PWM_Frequency, telemetryPtr};

        if (telemetryPtr != nullptr)
                logging.init();

        RadiationTestDemo radiationTestDemo(foc, telemetryPtr);
        radiationTestDemo.start();

        while (true) {
                if (radiationTestDemo.loggingEnabled())
                        telemetry.writeFrame(logging);

                SYSTICK_DelayMs(3);
        }
}
