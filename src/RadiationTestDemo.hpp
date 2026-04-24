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
#include "CriticalVariables.hpp"
#include "SAMD21_FOC.hpp"
#include "TMR.hpp"
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
        explicit RadiationTestDemo(SAMD21_FOC& foc, TelemetryLogger<TelemetryPayload12>* telemetry = nullptr) :
            samd21_FOC(foc), telemetryLogger(telemetry),
            targetIndex(tmrDemoStateMachine1.targetIndex, tmrDemoStateMachine2.targetIndex, tmrDemoStateMachine3.targetIndex),
            demoState(tmrDemoStateMachine1.demoState, tmrDemoStateMachine2.demoState, tmrDemoStateMachine3.demoState),
            positionDirectionTMR(tmrDemoStateMachine1.positionDirection, tmrDemoStateMachine2.positionDirection,
                                 tmrDemoStateMachine3.positionDirection) {}

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
                targetIndex.write(0U);
                demoState.write(static_cast<uint32_t>(DemoState::READY_TO_START));
                positionDirectionTMR.write(static_cast<int32_t>(PMSM_Controller::PositionDirection::CW));

                TC4_TimerCallbackRegister(&RadiationTestDemo::TimerCounterCallback, reinterpret_cast<uintptr_t>(this));
                TC4_TimerStart();
        }

        void setDirection(PMSM_Controller::PositionDirection direction) {
                positionDirectionTMR.write(static_cast<int32_t>(direction));
        }

private:
        /**
         * A list of the internal demo states
         */
        enum class DemoState : uint32_t {
                READY_TO_START = 0,
                RUN_SEQUENCE = 1,
        };

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

                healthMonitor.tc4EnterCounter = healthMonitor.tc4EnterCounter + 1;

                bool tc4CycleValid = false;

                const auto state = static_cast<DemoState>(demoState.read());
                const auto direction = static_cast<PMSM_Controller::PositionDirection>(positionDirectionTMR.read());

                switch (state) {
                case DemoState::READY_TO_START:
                        samd21_FOC.moveToAngle(TargetAngles_mrad[targetIndex.read()], direction);
                        demoState.write(static_cast<uint32_t>(DemoState::RUN_SEQUENCE));
                        tc4CycleValid = true;
                        break;

                case DemoState::RUN_SEQUENCE:
                        {
                                const uint32_t currentIndex = targetIndex.read();
                                const uint32_t next = (currentIndex + 1U) % static_cast<uint32_t>(TargetAngles_mrad.size());
                                targetIndex.write(next);
                                samd21_FOC.moveToAngle(TargetAngles_mrad[next], direction);
                                tc4CycleValid = true;
                                break;
                        }

                default:
                        demoState.write(static_cast<uint32_t>(DemoState::READY_TO_START));
                        targetIndex.write(0U);
                        break;
                }

                if (tc4CycleValid)
                        healthMonitor.tc4ValidCounter = healthMonitor.tc4ValidCounter + 1;
        }

        /**
         * Reference to the FOC implementation object
         */
        SAMD21_FOC& samd21_FOC;

        /**
         * Optional telemetry logger (nullptr disables logging)
         */
        TelemetryLogger<TelemetryPayload12>* telemetryLogger = nullptr;

        /**
         * TMR-backed demo variables stored in CriticalVariables.{hpp,cpp}
         */
        TMR<uint32_t> targetIndex;
        TMR<uint32_t> demoState;
        TMR<int32_t> positionDirectionTMR;
};
