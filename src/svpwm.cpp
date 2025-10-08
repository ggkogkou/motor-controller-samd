/**
* @file svpwm.cpp
 * @brief SVPWM implementation helpers and example fixed-vector usage
 */

#include "svpwm.hpp"
#include <cmath>

namespace ZeroSequenceModulation {

    /**
     * @brief Example: compute duties for a fixed vector (magnitude, angle)
     *
     * Build a normalized alpha-beta vector from magnitude and electrical angle,
     * then compute centered SVPWM duties in timer ticks.
     *
     * - magnitude: 0..~0.9 (normalized to Vdc)
     * - theta_rad: electrical angle in radians
     *
     * @param period_ticks PWM period in timer ticks
     * @param magnitude    Modulation index (normalized to Vdc)
     * @param theta_rad    Electrical angle in radians
     * @return SVPWM::DutyCycles Duty ticks for phases A/B/C
     */
    SVPWM::DutyCycles compute_fixed_vector(uint32_t period_ticks, float magnitude, float theta_rad)
    {
        SVPWM svm(period_ticks, 1.0f); // inputs are normalized (vbus = 1)
        const float v_alpha = magnitude * std::cos(theta_rad);
        const float v_beta  = magnitude * std::sin(theta_rad);
        return svm.computeFromAB(v_alpha, v_beta);
    }

} // namespace ZeroSequenceModulation