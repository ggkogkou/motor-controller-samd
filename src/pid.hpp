#pragma once

#include <algorithm>
#include <cstdint>
#include <type_traits>

/**
 * @brief Simple PID controller with output clamp.
 *
 * - For T=float: behaves like the original float PID.
 * - For T=int32_t: error/output are integer units (e.g. mA, mV, mrad/s), while gains are stored as fixed-point
 *   multipliers internally (Q(GAIN_Q)).
 *
 * Output clamp implicitly limits integral windup; add explicit anti-windup if needed.
 *
 * TODO: Separate integral and output clamping limits
 * TODO: Add a constructor that doesn't initialize the Kd so that it's a PI controller for FOC
 * TODO: Add a reset function
 * TODO: Require dT > 0, use of std::optional in the future
 */
template<typename T, int GAIN_Q = 16>
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
         * @param Ts Sampling time (seconds)
         */
        PID(float Kp, float Ki, float Kd, float limit, float Ts);

        /**
         * @brief Compute PID output for a given error (fixed dt implied in gains).
         *
         * @param error Setpoint minus measurement
         * @return Clamped controller output in ±limit
         */
        T compute(T error);

        /**
         * @brief Set the given floating point gains.
         *
         * For T=float: stores gains directly.
         * For T=int32_t: stores discrete-time coefficients as Q(GAIN_Q):
         *   K_Integral = Ki * 0.5 * Ts
         *   K_Derivative = Kd / Ts
         *
         * @param Kp
         * @param Ki
         * @param Kd
         * @param limit
         * @param Ts Sampling time (seconds)
         */
        void setGainsFloat(float Kp, float Ki, float Kd, float limit, float Ts);

        /**
         * @brief Reset the controller
         */
        void reset();

private:
        // For float: gains are T. For int: gains are Q(GAIN_Q) in int32_t.
        using GainType = std::conditional_t<std::is_floating_point_v<T>, T, int32_t>;

        /**
         * Proportional, integral, derivative gains
         */
        GainType K_Proportional = 0;
        GainType K_Integral = 0;
        GainType K_Derivative = 0;

        /**
         * Absolute output clamp (±limit)
         */
        T limit = 0;

        /**
         * The sampling time (seconds)
         */
        float dT = 0.0f;

        /**
         * Internal integrator and last error (for D)
         */
        T previousError = 0;
        T previousIntegralTerm = 0;
        T previousControllerOutput = 0;

        static int32_t gainToQ(float g);
        static T mulQ(int32_t gainQ, T x);
};

// Explicit instantiations are provided in pid_templated.cpp for PID<float> and PID<int32_t>.
