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
 * @file   pmsm_config.hpp
 * @brief  A collection of parameters that characterize the motor that is being driven
 * @author Georgios Gkogkou <ggkogkou125@gmail.com>
 */

#pragma once

#include "math_utils.hpp"

namespace PermanentMagnetSynchronousMotor {

using namespace MathUtilities;

struct PMSM_Config {
        /**
         * The DC link voltage
         */
        static constexpr float DCLinkVoltage = 20.0f;

        /**
         * The voltage limit -- DC bus utilization
         */
        static constexpr float CloseLoopVoltageLimit = 8.0f;

        /**
         * Encoder electrical offset and direction calibration voltage limit
         */
        static constexpr float InitialCalibrationVoltageLimit = 3.0f;

        /**
         * Target velocity for the outer velocity loop
         */
        static constexpr float TargetVelocity = 10.0f;

        /**
         * Target velocity for the encoder calibration loop (ω = 2π rad/s)
         */
        static constexpr float TargetCalibrationVelocity = TWO_PI;

        /**
         * The motor's pole pairs
         */
        static constexpr uint8_t MotorPolePairs = 11;

        static constexpr float OpenLoopVoltageLimit = InitialCalibrationVoltageLimit;

        static constexpr auto OpenLoopVoltLimit_mV = static_cast<int32_t>(OpenLoopVoltageLimit * 1000.0f);

        /**
         *
         * Brushless DC GM4108H-120T Gimbal Motor
         * --------------------------------------
         * Pole pairs: 11
         * No-load current: 0.07±0.1A
         * No-load voltage: 20 V
         * Load torque: 1200-1800 g*cm
         * Motor internal resistance: 11.1±5% Ω
         * No-load RPM: 513-567 RPM @ 20 V => calculate its Kv rating as approximately 25.65-28.35 RPM/V
         *
         */
        static constexpr float MotorKV_Rating = 26.0f;
        static constexpr float MotorInternalResistance = 11.0f;
};

} // namespace PermanentMagnetSynchronousMotor
