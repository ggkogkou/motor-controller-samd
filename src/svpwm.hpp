#pragma once

#include <algorithm>
#include <array>
#include <cassert>
#include <cstdint>

namespace SpaceVectorModulation {

enum class ZeroSequenceModulationType {
        MIDPOINT_CLAMP,
        UPPER_BOUND_CLAMP,
        LOWER_BOUND_CLAMP,
        THIRD_HARMONIC_INJECTION,
};

struct DutyCycles {
        int32_t dutyCycleA = 0;
        int32_t dutyCycleB = 0;
        int32_t dutyCycleC = 0;
};

/**
 * Q15 duty cycles (0..32767) where 32768 would be 1.0.
 */
struct DutyCyclesQ15 {
        uint16_t dutyA_q15 = 0;
        uint16_t dutyB_q15 = 0;
        uint16_t dutyC_q15 = 0;
};

/**
 * Class that implements the SVPWM technique
 */
class SVPWM {
public:
        SVPWM() = default;

        /**
         * @param vdc The DC link voltage (in mV)
         */
        explicit SVPWM(int16_t vdc, ZeroSequenceModulationType zsm) : dcLinkVoltage(vdc), zeroSequenceModulation(zsm) {
                assert(vdc > 0);
        }

        /**
         * Fixed-point (preferred): returns duty in Q15.
         *
         * @param vAlpha (in mV)
         * @param vBeta  (in mV)
         */
        [[nodiscard]] DutyCyclesQ15 computeQ15(int16_t vAlpha, int16_t vBeta) const;

        /**
         * Function that modulates the duty cycles for the center-aligned PWM signals that will drive the three-phase inverter using
         * Space Vector Modulation techniques
         *
         * @param vAlpha The Vα component found after inverse Park transformation in mV
         * @param vBeta The Vβ component found after inverse Park transformation in mV
         * @return The duty cycles in the the interval [0, 1'0000'000]
         */
        [[nodiscard]] DutyCycles compute(int16_t vAlpha, int16_t vBeta) const;

private:
        /**
         * The DC link voltage in mV
         */
        int16_t dcLinkVoltage = 5000;

        /**
         * The DC Link voltage in Volts
         */
        int32_t DC_LinkVoltage = 5;

        ZeroSequenceModulationType zeroSequenceModulation = ZeroSequenceModulationType::MIDPOINT_CLAMP;

        struct MinMax {
                int32_t min;
                int32_t max;
        };

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
