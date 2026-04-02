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
#include "HardwareDiagnosticsLogger.hpp"
#include "SAMD21_FOC.hpp"
#include "TrigonometricLUT.hpp"
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
         * @param diagnostics Optional hardware diagnostics logger (nullptr disables diagnostics logging)
         */
        explicit RadiationTestDemo(SAMD21_FOC& foc, TelemetryLogger<TelemetryPayload44>* telemetry = nullptr,
                                   HardwareDiagnosticsLogger* diagnostics = nullptr) :
            samd21_FOC(foc), telemetryLogger(telemetry), diagnosticsLogger(diagnostics) {}

        /**
         * Check if logging is enabled for this demo instance
         * @return True when telemetry logging is enabled
         */
        [[nodiscard]] bool loggingEnabled() const {
                return telemetryLogger != nullptr;
        }

        /**
         * Check whether the LUT self-test has detected a mismatch
         * @return True if a mismatch has been detected
         */
        [[nodiscard]] bool lutFaultDetected() const {
                return lutMismatchDetected;
        }

        /**
         * Return the LUT index where the first mismatch was detected
         * @return LUT mismatch index
         */
        [[nodiscard]] uint32_t lutFaultIndex() const {
                return lutMismatchIndex;
        }

        /**
         * Function that starts the TC that will periodically throw an interrupt and apply the initial target angle
         */
        void start() {
                targetIndex = 0;
                lutScrubIndex = 0;
                lutMismatchDetected = false;
                lutMismatchIndex = 0;
                lutFaultAlreadyLogged = false;
                demoState = DemoState::CHECK_SINE_LUT;

                TC4_TimerCallbackRegister(&RadiationTestDemo::TimerCounterCallback, reinterpret_cast<uintptr_t>(this));
                TC4_TimerStart();
        }

        void setDirection(PMSM_Controller::PositionDirection direction) {
                positionDirection = direction;
        }

private:
        /**
         * A list of the internal demo states
         */
        enum class DemoState : uint8_t {
                CHECK_SINE_LUT,
                READY_TO_START,
                RUN_SEQUENCE,
                FAULT_LUT_MISMATCH,
        };

        /**
         * A list of the return values used by the LUT checker
         */
        enum class LutCheckResult : uint8_t {
                IN_PROGRESS,
                PASSED,
                FAILED,
        };

        /**
         * The array of the fixed angles that the rotor should move to sequentially
         */
        static constexpr std::array<int32_t, 4> TargetAngles_mrad{0, 1571, 3141, 4712};

        /**
         * Number of LUT entries that are compared per TC4 callback while the startup self-test is running
         */
        static constexpr uint32_t LutEntriesPerCheck = 32;

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
         * Compare a small chunk of the sine LUT in RAM against the reference copy in flash
         *
         * @return Result of the current scrub chunk
         */
        [[nodiscard]] LutCheckResult checkSineLutChunk() {
                using SineLut = TrigonometricLUT::SineLookUpTableQ15<4096>;

                for (uint32_t i = 0; i < LutEntriesPerCheck; ++i) {
                        const uint32_t idx = lutScrubIndex;

                        if (SineLut::sineLUT[idx] != SineLut::sineLUT_Flash[idx]) {
                                lutMismatchDetected = true;
                                lutMismatchIndex = idx;
                                return LutCheckResult::FAILED;
                        }

                        lutScrubIndex++;

                        if (lutScrubIndex >= SineLut::sineLUT.size()) {
                                lutScrubIndex = 0;
                                return LutCheckResult::PASSED;
                        }
                }

                return LutCheckResult::IN_PROGRESS;
        }

        /**
         * Log the LUT mismatch only once
         */
        void logLutFaultOnce() {
                if (lutFaultAlreadyLogged)
                        return;

                lutFaultAlreadyLogged = true;

                if (diagnosticsLogger != nullptr)
                        diagnosticsLogger->writeLiteral("FAULT: SINE_LUT_MISMATCH\r\n");
        }

        /**
         * Function that runs from callback and updates the next target position
         * @param status
         */
        void setTargetPosition(TC_TIMER_STATUS status) {
                if ((status & TC_TIMER_STATUS_OVERFLOW) == 0U)
                        return;

                switch (demoState) {
                case DemoState::CHECK_SINE_LUT:
                        switch (checkSineLutChunk()) {
                        case LutCheckResult::IN_PROGRESS:
                                break;
                        case LutCheckResult::PASSED:
                                demoState = DemoState::READY_TO_START;
                                break;
                        case LutCheckResult::FAILED:
                        default:
                                demoState = DemoState::FAULT_LUT_MISMATCH;
                                logLutFaultOnce();
                                samd21_FOC.stop();
                                break;
                        }
                        break;

                case DemoState::READY_TO_START:
                        samd21_FOC.moveToAngle(TargetAngles_mrad[targetIndex], positionDirection);
                        demoState = DemoState::RUN_SEQUENCE;
                        break;

                case DemoState::RUN_SEQUENCE:
                        {
                                const uint32_t next = (targetIndex + 1U) % static_cast<uint32_t>(TargetAngles_mrad.size());
                                targetIndex = next;
                                samd21_FOC.moveToAngle(TargetAngles_mrad[targetIndex], positionDirection);
                                break;
                        }

                case DemoState::FAULT_LUT_MISMATCH:
                default:
                        logLutFaultOnce();
                        samd21_FOC.stop();
                        break;
                }
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
         * Optional hardware diagnostics logger (nullptr disables diagnostics logging)
         */
        HardwareDiagnosticsLogger* diagnosticsLogger = nullptr;

        /**
         * Counter that keeps track of the order of the positions
         */
        volatile uint32_t targetIndex = 0;

        /**
         * Current demo state
         */
        volatile DemoState demoState = DemoState::CHECK_SINE_LUT;

        /**
         * Current LUT scrub index
         */
        volatile uint32_t lutScrubIndex = 0;

        /**
         * Latched LUT fault flag
         */
        volatile bool lutMismatchDetected = false;

        /**
         * LUT index where the first mismatch was detected
         */
        volatile uint32_t lutMismatchIndex = 0;

        /**
         * Ensure the LUT fault is logged only once
         */
        volatile bool lutFaultAlreadyLogged = false;

        PMSM_Controller::PositionDirection positionDirection = PMSM_Controller::PositionDirection::CW;
};
