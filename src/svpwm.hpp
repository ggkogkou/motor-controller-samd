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
        int32_t dutyCycleA = 0;
        int32_t dutyCycleB = 0;
        int32_t dutyCycleC = 0;
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

private:
        /**
         * The DC link voltage in mV
         */
        int32_t dcLinkVoltage = 5000;

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
