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
 * @file   pid.cpp
 * @brief  Hardware-independent implementaton of a PID using fixed-point arithmetic
 * @author Georgios Gkogkou <ggkogkou125@gmail.com>
 */

#include "pid.hpp"

int32_t PID::compute(int32_t error) {
        const int32_t ProportionalTerm = K_Proportional * error;
        const int32_t DerivativeTerm = K_Derivative * (error - previousError);

        const auto IntegralTerm = [&]() -> int32_t {
                const int32_t sumErr = error + previousError;
                const int32_t incI = K_Integral * sumErr;
                const int32_t IntegralTermUnclamped = previousIntegralTerm + incI;
                return std::clamp(IntegralTermUnclamped, -limit, static_cast<int32_t>(limit));
        }();

        const auto PID_Output = [&]() -> int32_t {
                const int32_t PID_OutputUnclamped = ProportionalTerm + DerivativeTerm + IntegralTerm;
                return std::clamp(PID_OutputUnclamped, -limit, static_cast<int32_t>(limit));
        }();

        previousError = error;
        previousIntegralTerm = IntegralTerm;
        previousControllerOutput = PID_Output;

        return PID_Output;
}

void PID::setGainsFloat(float Kp, float Ki, float Kd, float clampLimit, float Ts) {
        limit = static_cast<int32_t>(clampLimit);
        dT = Ts;

        K_Proportional = Q16_t(Kp);
        K_Integral = Q16_t(Ki * 0.5f * Ts);
        K_Derivative = (Ts > 0.0f) ? Q16_t(Kd / Ts) : Q16_t(0.0f);
}

void PID::reset() {
        previousError = 0;
        previousIntegralTerm = 0;
        previousControllerOutput = 0;
}
