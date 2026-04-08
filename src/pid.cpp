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

PID::PID() :
    K_ProportionalRaw(localCriticalVariables1.Kp_raw, localCriticalVariables2.Kp_raw, localCriticalVariables3.Kp_raw),
    K_IntegralRaw(localCriticalVariables1.Ki_raw, localCriticalVariables2.Ki_raw, localCriticalVariables3.Ki_raw),
    K_DerivativeRaw(localCriticalVariables1.Kd_raw, localCriticalVariables2.Kd_raw, localCriticalVariables3.Kd_raw),
    outputClampLimit(localCriticalVariables1.limit, localCriticalVariables2.limit, localCriticalVariables3.limit) {
        setGainsFloat(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

PID::PID(float Kp, float Ki, float Kd, float clampLimit, float Ts) :
    K_ProportionalRaw(localCriticalVariables1.Kp_raw, localCriticalVariables2.Kp_raw, localCriticalVariables3.Kp_raw),
    K_IntegralRaw(localCriticalVariables1.Ki_raw, localCriticalVariables2.Ki_raw, localCriticalVariables3.Ki_raw),
    K_DerivativeRaw(localCriticalVariables1.Kd_raw, localCriticalVariables2.Kd_raw, localCriticalVariables3.Kd_raw),
    outputClampLimit(localCriticalVariables1.limit, localCriticalVariables2.limit, localCriticalVariables3.limit) {
        setGainsFloat(Kp, Ki, Kd, clampLimit, Ts);
}

PID::PID(PermanentMagnetSynchronousMotor::PID_CriticalVariables& bank1, PermanentMagnetSynchronousMotor::PID_CriticalVariables& bank2,
         PermanentMagnetSynchronousMotor::PID_CriticalVariables& bank3, float Kp, float Ki, float Kd, float clampLimit, float Ts) :
    K_ProportionalRaw(bank1.Kp_raw, bank2.Kp_raw, bank3.Kp_raw), K_IntegralRaw(bank1.Ki_raw, bank2.Ki_raw, bank3.Ki_raw),
    K_DerivativeRaw(bank1.Kd_raw, bank2.Kd_raw, bank3.Kd_raw), outputClampLimit(bank1.limit, bank2.limit, bank3.limit) {
        setGainsFloat(Kp, Ki, Kd, clampLimit, Ts);
}

int32_t __attribute__((section(".ramfunc"))) PID::compute(int32_t error) {
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
        dT = Ts;

        K_ProportionalRaw.write(Q16_t(Kp).getRaw());
        K_IntegralRaw.write(Q16_t(Ki * 0.5f * Ts).getRaw());
        K_DerivativeRaw.write((Ts > 0.0f) ? Q16_t(Kd / Ts).getRaw() : Q16_t(0.0f).getRaw());
        outputClampLimit.write(static_cast<int32_t>(clampLimit));

        refreshCachedCoefficients();
}

void PID::refreshCachedCoefficients() {
        K_Proportional = Q16_t(K_ProportionalRaw.read());
        K_Integral = Q16_t(K_IntegralRaw.read());
        K_Derivative = Q16_t(K_DerivativeRaw.read());
        limit = outputClampLimit.read();
}

void PID::reset() {
        previousError = 0;
        previousIntegralTerm = 0;
        previousControllerOutput = 0;
}
