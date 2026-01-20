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
 * @file   pid.hpp
 * @brief  Hardware-independent implementaton of a PID using fixed-point arithmetic
 * @author Georgios Gkogkou <ggkogkou125@gmail.com>
 */

#pragma once

#include <algorithm>
#include <cstdint>
#include <type_traits>

class Q16_t {
public:
        static constexpr int32_t Q16_Factor = 16;

        constexpr Q16_t() = default;
        explicit constexpr Q16_t(int32_t raw) : raw(raw) {}

        explicit Q16_t(float x) : raw(static_cast<int32_t>(x * (1u << Q16_Factor))) {}

        __attribute__((always_inline))
        [[nodiscard]] inline int32_t getRaw() const {
                return raw;
        }

        /**
         * Operator overload to multiply Q16_t * int32_t numbers
         *
         * @param a
         * @param b
         * @return
         */
        __attribute__((always_inline))
        friend inline int32_t operator*(Q16_t k, int32_t x) {
                int64_t p = static_cast<int64_t>(k.getRaw()) * static_cast<int64_t>(x);

                return static_cast<int32_t>(p >> Q16_Factor);
        }

        /**
         * Operator overload to multiply int32_t * Q16_t numbers
         *
         * @note Use the previous operator overload function in order to do that
         *
         * @param a
         * @param b
         * @return
         */
        __attribute__((always_inline))
        friend inline int32_t operator*(int32_t x, Q16_t k) {
                return k * x;
        }

        /**
         * Operator overload to multiply Q16_t * Q16_t numbers
         *
         * @param a
         * @param b
         * @return
         */
        __attribute__((always_inline))
        friend inline Q16_t operator*(Q16_t a, Q16_t b) {
                int64_t p = static_cast<int64_t>(a.getRaw()) * static_cast<int64_t>(b.getRaw());
                return Q16_t(static_cast<int32_t>(p >> Q16_Factor));
        }

private:
        int32_t raw = 0;
};

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
        PID(float Kp, float Ki, float Kd, float limit, float Ts) {
                setGainsFloat(Kp, Ki, Kd, limit, Ts);
        }

        /**
         * @brief Compute PID output for a given error (fixed dt implied in gains).
         *
         * @param error Setpoint minus measurement
         * @return Clamped controller output in ±limit
         */
        int32_t compute(int32_t error);

        /**
         * @brief Reset the controller
         */
        void reset();

private:
        /**
         * Proportional, integral, derivative gains
         */
        Q16_t K_Proportional {0.0f};
        Q16_t K_Integral {0.0f};
        Q16_t K_Derivative {0.0f};

        /**
         * Absolute output clamp (±limit)
         */
        int32_t limit = 0;

        /**
         * The sampling time (seconds)
         */
        float dT = 0.0f;

        /**
         * Internal integrator and last error (for D)
         */
        int32_t previousError = 0;
        int32_t previousIntegralTerm = 0;
        int32_t previousControllerOutput = 0;

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
};
