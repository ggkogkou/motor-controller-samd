#pragma once

#include <algorithm>

/**
 * @brief Simple PID controller with output clamp.
 *
 * Output clamp implicitly limits integral windup; add explicit anti-windup if needed.
 */
class PID {
public:
    /**
     * @brief Default construct with zero gains and zero limits
     */
    PID() = default;
    ~PID() = default;

    /**
     * @brief Construct with gains and symmetric output limit.
     * @param Kp Proportional gain
     * @param Ki Integral gain
     * @param Kd Derivative gain
     * @param limit Absolute output clamp (±limit)
     */
    PID(float Kp, float Ki, float Kd, float limit) : K_Proportional(Kp), K_Integral(Ki), K_Derivative(Kd), limit(limit) {}

    /**
     * @brief Compute PID output for a given error (fixed dt implied in gains).
     *
     * Notes:
     * - Integral windup is limited by clamping the output; a simple conditional
     *   integration guard is applied so the integrator doesn't grow when the
     *   controller is saturated in the same direction as the error.
     * - Derivative term is computed on error (not measurement) with a simple
     *   backward difference using the last error.
     *
     * @param error Setpoint minus measurement
     * @return Clamped controller output in ±limit
     */
    float compute(float error);

private:
    /**
     * Proportional, integral, derivative gains
     */
    float K_Proportional = 0.0f;
    float K_Integral = 0.0f;
    float K_Derivative = 0.0f;

    /**
     * Absolute output clamp (±limit)
     */
    float limit = 0.0f;

    /**
     * Internal integrator and last error (for D)
     */
    float previousError = 0.0f;
    float previousIntegralTerm = 0.0f;
    float previousControllerOutput = 0.0f;
    float previousTimestamp = 0.0f;

};
