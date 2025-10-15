#pragma once

#include <cstdint>
#include <array>
#include <cmath>
#include <algorithm>
#include <cassert>
#include "math_utils.hpp"

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
    /**
     * Default constructor (call setters before use)
     */
    SVPWM() = default;

    /**
     * Simple constructor that initializes the useful member variables
     *
     * @param vdc The DC link voltage
     */
    explicit SVPWM(float vdc, ZeroSequenceModulationType zsm) : dcLinkVoltage(vdc), zeroSequenceModulation(zsm) {
        assert(std::isfinite(vdc) && vdc > 0.0f);
    }

    /**
     * Function that computes the dutiy cycles for the SVPWM
     *
     * @param vAlpha The voltage of coordinate alpha
     * @param vBeta The voltage of coordinate beta
     * @return The duty cycles binded as a struct
     */
    [[nodiscard]] DutyCycles compute(float vAlpha, float vBeta) const;

private:
    /**
     * The DC link voltage (maybe it would be better to read it from somewhere else?)
     */
    float dcLinkVoltage = 5.0f;

    /**
     * The type of Zero-Sequence Modulation (ZSM) that is added to the carrier
     */
    ZeroSequenceModulationType zeroSequenceModulation = ZeroSequenceModulationType::MIDPOINT_CLAMP;

};

} /// namespace SpaceVectorModulation
