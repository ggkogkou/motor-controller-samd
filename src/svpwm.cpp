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
 * @file   svpwm.cpp
 * @brief  Hardware-independent implementation of the SVPWM modulation technique using fixed-point arithmetic
 * @author Georgios Gkogkou <ggkogkou125@gmail.com>
 */

#include "svpwm.hpp"

namespace SpaceVectorModulation {

DutyCycles __attribute__((section(".ramfunc"))) SVPWM::compute(int32_t vAlpha, int32_t vBeta) const {
        constexpr int32_t HALF_Q15 = 16384; // 0.5 * 32768
        constexpr int32_t SQRT3_2_Q15 = 28378; // (sqrt(3)/2) * 32768

        int32_t Va = vAlpha;
        int32_t Vb = -(vAlpha >> 1) + ((SQRT3_2_Q15 * vBeta) >> 15);
        int32_t Vc = -(vAlpha >> 1) - ((SQRT3_2_Q15 * vBeta) >> 15);

        const auto [V_Min, V_Max] = findMinMax(Va, Vb, Vc);

        int32_t zeroSequenceComponent = 0;

        if (zeroSequenceModulation == ZeroSequenceModulationType::MIDPOINT_CLAMP)
                zeroSequenceComponent -= (V_Min + V_Max) / 2;

        Va += zeroSequenceComponent;
        Vb += zeroSequenceComponent;
        Vc += zeroSequenceComponent;

        // Va = std::clamp(Va, -dcLinkVoltage, dcLinkVoltage);
        // Vb = std::clamp(Vb, -dcLinkVoltage, dcLinkVoltage);
        // Vc = std::clamp(Vc, -dcLinkVoltage, dcLinkVoltage);

        constexpr int32_t MIN_Q15 = 0;
        constexpr int32_t MAX_Q15 = 32767;

        const uint32_t DutyCycleA = std::clamp(HALF_Q15 + Va * inverseVdc_Q15, MIN_Q15, MAX_Q15);
        const uint32_t DutyCycleB = std::clamp(HALF_Q15 + Vb * inverseVdc_Q15, MIN_Q15, MAX_Q15);
        const uint32_t DutyCycleC = std::clamp(HALF_Q15 + Vc * inverseVdc_Q15, MIN_Q15, MAX_Q15);

        return DutyCycles{DutyCycleA, DutyCycleB, DutyCycleC};
}

PWM_Periods __attribute__((section(".ramfunc"))) SVPWM::compute(int32_t vAlpha, int32_t vBeta, uint32_t pwmPeriod) const {
        const auto [dA, dB, dC] = compute(vAlpha, vBeta);

        constexpr uint32_t POW_2_14 = 1 << 14; /// 2^14

        const uint32_t pwmPeriodQ15_A = pwmPeriod * dA;
        const uint32_t pwmPeriodQ15_B = pwmPeriod * dB;
        const uint32_t pwmPeriodQ15_C = pwmPeriod * dC;

        const uint32_t pwmPeriodDec_A = (pwmPeriodQ15_A + POW_2_14) >> 15;
        const uint32_t pwmPeriodDec_B = (pwmPeriodQ15_B + POW_2_14) >> 15;
        const uint32_t pwmPeriodDec_C = (pwmPeriodQ15_C + POW_2_14) >> 15;

        return {pwmPeriodDec_A, pwmPeriodDec_B, pwmPeriodDec_C};
}

} // namespace SpaceVectorModulation
