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
 * @file   svpwm.hpp
 * @brief  Hardware-independent implementation of the SVPWM modulation technique using fixed-point arithmetic
 * @author Georgios Gkogkou <ggkogkou125@gmail.com>
 */

#pragma once

#include <algorithm>
#include <cassert>
#include <cstdint>

namespace SpaceVectorModulation {

/**
 * @enum ZeroSequenceModulationType
 *
 * A collection of the possible types that zero sequence modulation can be performed
 */
enum class ZeroSequenceModulationType {
        MIDPOINT_CLAMP,
        UPPER_BOUND_CLAMP,
        LOWER_BOUND_CLAMP,
        THIRD_HARMONIC_INJECTION,
};

/**
 * Structure that holds the duty cycles that are calculated by the SVPWM::compute function
 */
struct DutyCycles {
        uint32_t dutyCycleA = 0;
        uint32_t dutyCycleB = 0;
        uint32_t dutyCycleC = 0;
};

struct PWM_Periods {
        uint32_t pwmPeriodeA = 0;
        uint32_t pwmPeriodeB = 0;
        uint32_t pwmPeriodeC = 0;
};

/**
 * Class that implements the SVPWM technique
 */
class SVPWM {
public:
        SVPWM() = default;

        /**
         * Constructor of SVPWM class
         *
         * @param dcMotorVoltage_mV The DC link voltage (in mV)
         * @param zsm The zero sequence modulation type
         */
        explicit SVPWM(int16_t dcMotorVoltage_mV, ZeroSequenceModulationType zsm) :
            dcLinkVoltage(dcMotorVoltage_mV), zeroSequenceModulation(zsm) {
                assert(dcMotorVoltage_mV > 0);

                inverseVdc_Q15 = ((static_cast<int32_t>(1) << 15) + dcMotorVoltage_mV / 2) / dcMotorVoltage_mV;
        }

        /**
         * Function that modulates the duty cycles for the center-aligned PWM signals that will drive the three-phase inverter using
         * Space Vector Modulation techniques
         *
         * @param vAlpha The Vα component found after inverse Park transformation in mV
         * @param vBeta The Vβ component found after inverse Park transformation in mV
         * @return The duty cycles in the Q15 fixed-point arithmetic interval [0, 32'767]
         */
        [[nodiscard]] DutyCycles compute(int32_t vAlpha, int32_t vBeta) const;

        [[nodiscard]] PWM_Periods compute(int32_t vAlpha, int32_t vBeta, uint32_t pwmPeriod) const;

private:
        /**
         * The DC link voltage in mV
         */
        int32_t dcLinkVoltage = 20'000;

        /**
         * Fixed-point inverse of the DC link voltage (1/Vdc) in Q15 format
         */
        int32_t inverseVdc_Q15 = 0;

        /**
         * The zero-sequence modulation type
         */
        ZeroSequenceModulationType zeroSequenceModulation = ZeroSequenceModulationType::MIDPOINT_CLAMP;

        /**
         * Structure that will hold the minimum and maximum values, meant to be used with findMinMax
         */
        struct MinMax {
                int32_t min;
                int32_t max;
        };

        /**
         * Function that finds the minimum and maximum between three integer numbers
         * @param a Number a
         * @param b Number b
         * @param c Number c
         * @return
         */
        static inline MinMax findMinMax(int32_t a, int32_t b, int32_t c) {
                MinMax r{a, a};

                if (b < r.min)
                        r.min = b;

                if (b > r.max)
                        r.max = b;

                if (c < r.min)
                        r.min = c;

                if (c > r.max)
                        r.max = c;

                return r;
        }
};

} // namespace SpaceVectorModulation
