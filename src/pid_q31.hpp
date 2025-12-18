#pragma once

#include <algorithm>
#include <cstdint>
#include <limits>

using q31_t = int32_t;

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
class PID_Q31 {
public:
        /**
         * @brief Default construct with zero gains and zero limits
         */
        PID_Q31() = default;
        ~PID_Q31() = default;

        /**
         * @brief Construct with gains and symmetric output limit.
         * @param Kp Proportional gain
         * @param Ki Integral gain
         * @param Kd Derivative gain
         * @param limit Absolute output clamp (±limit)
         */
        PID_Q31(q31_t Kp, q31_t Ki, q31_t Kd, q31_t limit, float Ts) :
            K_Proportional(Kp), K_Integral(Ki), K_Derivative(Kd), limit(limit), dT(Ts) {}

        /**
         * @brief Compute PID output for a given error (fixed dt implied in gains).
         *
         * @param error Setpoint minus measurement
         * @return Clamped controller output in ±limit
         */
        q31_t compute(q31_t error);

        /**
         * Set the given floating point gains in Q31 fixed-point arithmetic
         *
         * @param Kp
         * @param Ki
         * @param Kd
         * @param limit_f
         * @param Ts
         */
        void setGainsFloat(float Kp, float Ki, float Kd, float limit_f, float Ts);

        /**
         * Reset the controller
         */
        void reset();

private:
        /**
         * Proportional gain Kp
         */
        q31_t K_Proportional = 0;

        /**
         * Integral gain Ki * (dT/2)
         */
        q31_t K_Integral = 0;

        /**
         * Derivative gain Kd / dT
         */
        q31_t K_Derivative = 0;

        /**
         * Absolute output clamp (±limit)
         */
        q31_t limit = 0;

        /**
         * The sampling time
         */
        float dT = 0;

        /**
         * Internal integrator and last error (for D)
         */
        q31_t previousError = 0;
        q31_t previousIntegralTerm = 0;
        q31_t previousControllerOutput = 0;

        /**
         * Helper function that converts floating point number to Q31 scaling; used only at startup, not fast loop
         * @param x
         * @return
         */
        static q31_t floatToQ31(float x) {
                if (x >= 0.99999994f)
                        x = 0.99999994f;

                if (x <= -1.0f)
                        x = -1.0f;

                constexpr auto MAX_Q31 = static_cast<float>(std::numeric_limits<q31_t>::max());

                return static_cast<q31_t>(x * 2147483648.0f);
        }

        /**
         * Helper function that performs multiplication and rescaling/shifting
         *
         * @param a
         * @param b
         * @return
         */
        static q31_t q31_mul(q31_t a, q31_t b) {
                constexpr auto SHIFT = 31;
                return static_cast<q31_t>((static_cast<int64_t>(a) * static_cast<int64_t>(b)) >> SHIFT);
        }

};
