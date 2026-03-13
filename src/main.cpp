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

#include "RadiationTestDemo.hpp"
#include "SAMD21_FOC.hpp"

[[noreturn]] int main() {
        SYS_Initialize(nullptr);
        SYSTICK_TimerStart();
        SPI_Buffer::init();

        static constexpr frequency_kHz_t PWM_Frequency = 19.0f;
        static constexpr bool EnableLogging = true;

        USART_TxStream logging;
        TelemetryLogger telemetry;
        TelemetryLogger* telemetryPtr = nullptr;
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
