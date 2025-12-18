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
        float dutyCycleA = 0.0f;
        float dutyCycleB = 0.0f;
        float dutyCycleC = 0.0f;
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
         * @param vAlpha (in mV)
         * @param vBeta  (in mV)
         */
        [[nodiscard]] DutyCycles compute(int16_t vAlpha, int16_t vBeta) const;

private:
        /**
         * The DC link voltage in mV
         */
        int16_t dcLinkVoltage = 5000;

        ZeroSequenceModulationType zeroSequenceModulation = ZeroSequenceModulationType::MIDPOINT_CLAMP;
};

} // namespace SpaceVectorModulation
