#pragma once

#include <algorithm>

/**
 * @brief Simple PID controller with output clamp.
 *
 * Output clamp implicitly limits integral windup; add explicit anti-windup if needed.
 *
 * TODO: Separate integral and output clamping limits
 * TODO: Add a constructor that doesn't initialize the Kd so that it's a PI controller for FOC
 * TODO: Add a reset function
 * TODO: Require dT > 0, use of std::optional in the future
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
    PID(float Kp, float Ki, float Kd, float limit, float Ts) : K_Proportional(Kp), K_Integral(Ki), K_Derivative(Kd), limit(limit), dT(Ts) {}

    /**
     * @brief Compute PID output for a given error (fixed dt implied in gains).
     *
     * @param error Setpoint minus measurement
     * @return Clamped controller output in ±limit
     */
    float compute(float error);

private:
    /**
     * Proportional, integral, derivative gains
     */
    float K_Proportional = 0.2f;
    float K_Integral = 20.0f;
    float K_Derivative = 0.0f;

    /**
     * Absolute output clamp (±limit)
     */
    float limit = 0.0f;

    /**
     * The sampling time
     */
    float dT = 0.0f;

    /**
     * Internal integrator and last error (for D)
     */
    float previousError = 0.0f;
    float previousIntegralTerm = 0.0f;
    float previousControllerOutput = 0.0f;

};
