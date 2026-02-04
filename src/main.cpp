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

#include "SAMD21_FOC.hpp"

using namespace PermanentMagnetSynchronousMotor;

[[noreturn]] int main() {
        SYS_Initialize(nullptr);
        SYSTICK_TimerStart();
        SPI_Buffer::init();

        static constexpr frequency_kHz_t PWM_Frequency = 24.0f;

        USART_TxStream logging;
        TelemetryLogger telemetry;
        SAMD21_FOC foc{PWM_Frequency, &telemetry};

        logging.init();

        uint32_t lastDropped = 0;
        uint32_t loopCounter = 0;

        static constexpr int32_t TargetAngle90deg_mrad = 1571; // ~pi/2 mrad

        foc.moveToAngle(TargetAngle90deg_mrad, PMSM_Controller::PositionDirection::CCW);

        while (true) {
                telemetry.writeFrame(logging);

                if (const uint32_t dropped = logging.getDroppedBytes(); dropped != lastDropped) {
                        lastDropped = dropped;
                        // BENCHMARK_IO_Set();
                        // BENCHMARK_IO_Clear();
                }

                if (loopCounter++ % 100u == 0) { // move every 100 * 50ms = 5sec
                        foc.moveToAngle(TargetAngle90deg_mrad, PMSM_Controller::PositionDirection::CCW, 1);
                }

                SYSTICK_DelayMs(50);
        }
}
