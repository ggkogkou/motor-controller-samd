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
 * @file   RadiationTestDemo.hpp
 * @brief  Angle-step demo driver for TC4-based radiation testing
 * @author Georgios Gkogkou <ggkogkou125@gmail.com>
 */

#pragma once

#include <array>
#include <cstdint>
#include "SAMD21_FOC.hpp"
#include "definitions.h"

using namespace PermanentMagnetSynchronousMotor;

/**
 * @class RadiationTestDemo
 *
 * A class that implements a demo application to run during the radiation tests
 *
 * @brief Periodically move the rotor to a fixed angle sequence
 */
class RadiationTestDemo {
public:
        /**
         * Constructor
         *
         * @param foc Reference to the motor FOC controller
         * @param telemetry Optional telemetry logger (nullptr disables logging)
         */
        explicit RadiationTestDemo(SAMD21_FOC& foc, TelemetryLogger<TelemetryPayload44>* telemetry = nullptr) :
            samd21_FOC(foc), telemetryLogger(telemetry) {}

        /**
         * Check if logging is enabled for this demo instance
         * @return True when telemetry logging is enabled
         */
        [[nodiscard]] bool loggingEnabled() const {
                return telemetryLogger != nullptr;
        }

        /**
         * Function that starts the TC that will periodically throw an interrupt and apply the initial target angle
         */
        void start() {
                TC4_TimerCallbackRegister(&RadiationTestDemo::TimerCounterCallback, reinterpret_cast<uintptr_t>(this));
                TC4_TimerStart();
                samd21_FOC.moveToAngle(TargetAngles_mrad[targetIndex], positionDirection);
        }

        void setDirection(PMSM_Controller::PositionDirection direction) {
                positionDirection = direction;
        }

private:
        /**
         * The array of the fixed angles that the rotor should move to sequentially
         */
        static constexpr std::array<int32_t, 4> TargetAngles_mrad{0, 1571, 3141, 4712};

        /**
         * Callback
         * @param status
         * @param context
         */
        static void TimerCounterCallback(TC_TIMER_STATUS status, uintptr_t context) {
                auto* self = reinterpret_cast<RadiationTestDemo*>(context);
                self->setTargetPosition(status);
        }

        /**
         * Function that runs from callback and updates the next target position
         * @param status
         */
        void setTargetPosition(TC_TIMER_STATUS status) {
                if ((status & TC_TIMER_STATUS_OVERFLOW) == 0U)
                        return;

                const uint32_t next = (targetIndex + 1U) % static_cast<uint32_t>(TargetAngles_mrad.size());
                targetIndex = next;
                samd21_FOC.moveToAngle(TargetAngles_mrad[targetIndex], positionDirection);
        }

        /**
         * Reference to the FOC implementation object
         */
        SAMD21_FOC& samd21_FOC;

        /**
         * Optional telemetry logger (nullptr disables logging)
         */
        TelemetryLogger<TelemetryPayload44>* telemetryLogger = nullptr;

        /**
         * Counter that keeps track of the order of the positions
         */
        volatile uint32_t targetIndex = 0;

        PMSM_Controller::PositionDirection positionDirection = PMSM_Controller::PositionDirection::CW;
};
