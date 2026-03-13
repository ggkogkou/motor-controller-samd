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
 * @file   math_utils.hpp
 * @brief  Clarke/Park transformations and sin/cos compile-time LUT construction using floating-point arithmetic
 * @author Georgios Gkogkou <ggkogkou125@gmail.com>
 */

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <span>
#include "TrigonometricLUT.hpp"
#include "clarke_park_q31.hpp"

namespace MathUtilities {

/**
 * Useful constants
 */
inline constexpr float PI = 3.14159265358979323846f;
inline constexpr float HALF_PI = PI / 2.0f;
inline constexpr float TWO_PI = 2.0f * PI;
inline constexpr float SQRT3 = 1.7320508075688772f;
inline constexpr float SQRT3_2 = SQRT3 / 2.0f;

/**
 * Struct that binds the look-up table generation for sine
 *
 * @tparam N The number of samples for the quantization of the continuous sin(x)
 */
template <std::size_t N = 4096>
struct SineLookUpTable {
        /**
         * Default constructor
         */
        constexpr SineLookUpTable() = default;

        static_assert(N % 4 == 0, "LUT size must be dividable by 4 for calculation reasons");

        /**
         * Function that calculates sin(x) from the Maclaurin power series with 6 extra terms
         * Now includes terms up to x^13 (n = 6 beyond the linear term).
         *
         * @param x The angle in radians
         * @return The value of sin(x)
         */
        static constexpr float calculateSineFromMaclaurin(float x) {
                const float xSquare = x * x;

                constexpr float Factorial_3 = 6.0f;
                constexpr float Factorial_5 = 120.0f;
                constexpr float Factorial_7 = 5040.0f;
                constexpr float Factorial_9 = 362880.0f;
                constexpr float Factorial_11 = 39916800.0f;
                constexpr float Factorial_13 = 6227020800.0f;

                constexpr float Factorial_3_Inv = 1.0f / Factorial_3;
                constexpr float Factorial_5_Inv = 1.0f / Factorial_5;
                constexpr float Factorial_7_Inv = 1.0f / Factorial_7;
                constexpr float Factorial_9_Inv = 1.0f / Factorial_9;
                constexpr float Factorial_11_Inv = 1.0f / Factorial_11;
                constexpr float Factorial_13_Inv = 1.0f / Factorial_13;

                return x *
                        (1.0f +
                         xSquare *
                                 (-Factorial_3_Inv +
                                  xSquare *
                                          (Factorial_5_Inv +
                                           xSquare *
                                                   (-Factorial_7_Inv +
                                                    xSquare *
                                                            (Factorial_9_Inv +
                                                             xSquare *
                                                                     (-Factorial_11_Inv +
                                                                      xSquare * Factorial_13_Inv))))));
        }

        /**
         * Alias for the look-up table type
         */
        using LookUpTable = std::array<float, N + 1>;

        static constexpr std::size_t LUT_Q90_SIZE = N / 4;

        /**
         * Function that generates a Look-Up Table at compile time
         *
         * @return The look-up table
         */
        static consteval LookUpTable generateLookUpTable() {
                LookUpTable sineLUT{0.0f};

                sineLUT[0] = 0.0f;

                for (size_t i = 1; i < LUT_Q90_SIZE - 1; i++) {
                        sineLUT[i] = calculateSineFromMaclaurin(TWO_PI * static_cast<float>(i) / static_cast<float>(N));
                }

                sineLUT[LUT_Q90_SIZE] = 1.0f;

                for (size_t i = LUT_Q90_SIZE + 1; i < 2 * LUT_Q90_SIZE - 1; i++) {
                        sineLUT[i] = sineLUT[2 * LUT_Q90_SIZE - i];
                }

                sineLUT[2 * LUT_Q90_SIZE] = 0.0f;

                for (size_t i = 2 * LUT_Q90_SIZE + 1; i < 3 * LUT_Q90_SIZE - 1; i++) {
                        sineLUT[i] = -sineLUT[i - 2 * LUT_Q90_SIZE];
                }

                sineLUT[3 * LUT_Q90_SIZE] = -1.0f;

                for (size_t i = 3 * LUT_Q90_SIZE + 1; i < N - 1; i++) {
                        sineLUT[i] = -sineLUT[4 * LUT_Q90_SIZE - i];
                }

                sineLUT[N] = 0.0f;

                return sineLUT;
        }

        /**
         * The generated sine LUT
         */
        static constexpr LookUpTable sineLUT = generateLookUpTable();

        /**
         * Operator [] with linear interpolation for best accuracy
         *
         * Maps theta (radians) to a fractional index i = theta * N / (2π)
         * then linearly interpolates between floor(i) and floor(i)+1
         * The LUT has size N+1 with sine[N] == sine[0] to simplify wraparound
         *
         */
        constexpr float operator[](float theta) const {
                constexpr auto convertTo_0_2PI_Interval = [](float x) {
                        while (x < 0.0f)
                                x += TWO_PI;
                        while (x >= TWO_PI)
                                x -= TWO_PI;
                        return x;
                };

                const float fidx = convertTo_0_2PI_Interval(theta) * static_cast<float>(N) / TWO_PI;

                const auto Index0 = static_cast<std::size_t>(fidx);
                const auto Index1 = Index0 + 1;
                const float frac = fidx - static_cast<float>(Index0);

                const float y0 = sineLUT[Index0];
                const float y1 = sineLUT[Index1];

                return y0 + (y1 - y0) * frac;
        }
};

/**
 * Struct that shifts the sine LUT to produce the cosine
 *
 * @tparam N The number of samples for the quantization of the continuous cos(x)
 */
template <std::size_t N = 4096>
struct CosineLookUpTable : private SineLookUpTable<N> {
        using Base = SineLookUpTable<N>;

        constexpr float operator[](float theta) const { return Base::operator[](theta + HALF_PI); }
};

/**
 * Sine and cosine look-up tables that must reside in internal flash memory
 */
inline constexpr SineLookUpTable sine;
inline constexpr CosineLookUpTable cosine;

using DQFrame = std::array<float, 2>;
using AlphaBetaFrame = std::array<float, 2>;

/**
 * Function that performs the "Direct-quadrature-zero" or "Park" transformation that turns the stationary
 * αβ-frame to the rotating dq-frame
 *
 * @see [Direct-quadrature-zero transformation](https://en.wikipedia.org/wiki/Direct-quadrature-zero_transformation)
 *
 * @param Ua The a-axis component of the two-dimensional stationary frame
 * @param Ub The β-axis component of the two-dimensional stationary frame
 * @param theta The angle θ
 * @return The dq-frame coordinates which are Vd and Vq
 */
[[nodiscard]] [[maybe_unused]] inline DQFrame performParkTransform(float Ua, float Ub, float theta) {
        const float CosineTheta = cosine[theta];
        const float SineTheta = sine[theta];

        const float Ud = CosineTheta * Ua + SineTheta * Ub;
        const float Uq = -SineTheta * Ua + CosineTheta * Ub;

        return {Ud, Uq};
}

/**
 * Function that performs the inverse Park transformation that turns the rotating dq-frame into the
 * stationary αβ-frame
 *
 * @param Ud
 * @param Uq
 * @param theta
 * @return
 */
[[nodiscard]] [[maybe_unused]] inline AlphaBetaFrame performInverseParkTransform(float Ud, float Uq, float theta) {
        const float CosineTheta = cosine[theta];
        const float SineTheta = sine[theta];
        const float Ualpha = CosineTheta * Ud - SineTheta * Uq;
        const float Ubeta = SineTheta * Ud + CosineTheta * Uq;

        return {Ualpha, Ubeta};
}

/**
 * Function that performs the "Alpha–beta" or "Clarke" transformation that projects three-dimensional systems
 * onto two-dimensional axes
 *
 * @see [Alpha–beta transformation](https://en.wikipedia.org/wiki/Alpha%E2%80%93beta_transformation)
 * @param Ua
 * @param Ub
 * @param Uc
 * @return
 */
[[nodiscard]] [[maybe_unused]] inline AlphaBetaFrame performClarkeTransform(float Ua, float Ub) {
        const float Ualpha = Ua;
        const float Ubeta = (Ua + Ub * 2.0f) / SQRT3;

        return {Ualpha, Ubeta};
}

/**
 * Function that performs the chained Clarke-Park transformations in order to project a balanced three-phase system
 * into the rotating dq-frame
 *
 * @note This transformation assumes a balanced three-phase system (ia + ib + ic = 0)
 *
 * @see [Park transformation
 * derivation](https://en.wikipedia.org/wiki/Direct-quadrature-zero_transformation#Park_transformation_derivation)
 * @param Ua
 * @param Ub
 * @param theta
 * @return
 */
[[nodiscard]] inline DQFrame performClarkeParkTransforms(float Ua, float Ub, float theta) {
        const float Ualpha = Ua;
        const float Ubeta = (Ua + 2.0f * Ub) / SQRT3;

        const float CosineTheta = cosine[theta];
        const float SineTheta = sine[theta];

        const float Ud = Ualpha * CosineTheta + Ubeta * SineTheta;
        const float Uq = -Ualpha * SineTheta + Ubeta * CosineTheta;

        return {Ud, Uq};
}

/**
 * Function that converts an angle from degrees to radians
 *
 * @param angleDegrees The angle in degrees
 * @return The angle in radians
 */
[[nodiscard]] inline float degreesToRadians(float angleDegrees) { return angleDegrees * PI / 180.0f; }

/**
 * Function that converts an angle from degrees to milli-radians (mrad); useful for fixed-point implementation
 *
 * @param angleDegrees The angle in degrees
 * @return The angle in mrad
 */
[[nodiscard]] inline int32_t degreesToMilliRad(float angleDegrees) noexcept {
        constexpr auto ConversionFactor = std::numbers::pi_v<float> / 180.0f * 1000.0f;
        return static_cast<int32_t>(angleDegrees * ConversionFactor);
}

/**
 * Wrap an angle into [0, 2π)
 */
[[nodiscard]] inline float wrapAngle(float x) noexcept {
        while (x < 0.0f)
                x += TWO_PI;
        while (x >= TWO_PI)
                x -= TWO_PI;
        return x;
}

/**
 * Function that checks whether the (Vd, Vq) vector is inside the limit inscribed circle / linear range of SVM
 * @param Vd The d-axis voltage
 * @param Vq The q-axis voltage
 * @param VLim The maximum/limiting voltage that marks the threshold of overmodulation
 */
inline void limitCircle(float& Vd, float& Vq, float VLim) {
        const float MagnitudeSquare = Vd * Vd + Vq * Vq;

        if (const float VLimSquare = VLim * VLim; MagnitudeSquare > VLimSquare) {
                Vd = Vd / 2;
                Vq = Vq / 2;
        }
}

/**
 * Simple static assertions to quickly showcase the correctness
 */
static_assert(sine[0.0f] == 0.0f, "The sin(pi/2) does not evaluate to 1");
static_assert(sine[HALF_PI] >= 0.9999999f, "The sin(0) does not evaluate to 0");
// static_assert(sinLUT[TWO_PI] <= 1.0f, "The sin(2pi) does not evaluate to 0");

/**
 * Error limit to ensure a certain level of accuracy
 */
inline constexpr float ErrorLimit = 0.001f;

/**
 * Function that calculates the absolute difference of two numbers at compile time
 *
 * @param a Number a
 * @param b Number a
 * @return The absolute |a-b|
 */
constexpr float absoluteError(float a, float b) {
        const float diff = a - b;
        return diff < 0.0f ? -diff : diff;
}

static_assert(absoluteError(sine[0.356f], 0.34852783777f) <= ErrorLimit, "Error not acceptable");
static_assert(absoluteError(sine[1.255f], 0.95054936231f) <= ErrorLimit, "Error not acceptable");
static_assert(absoluteError(sine[1.788f], 0.97650387439f) <= ErrorLimit, "Error not acceptable");
static_assert(absoluteError(sine[2.500f], 0.59847214410f) <= ErrorLimit, "Error not acceptable");
static_assert(absoluteError(sine[3.657f], -0.49288931877f) <= ErrorLimit, "Error not acceptable");
static_assert(absoluteError(sine[4.438f], -0.96259093846f) <= ErrorLimit, "Error not acceptable");
static_assert(absoluteError(sine[5.796f], -0.46814053122f) <= ErrorLimit, "Error not acceptable");
static_assert(absoluteError(sine[6.200f], -0.08308940281f) <= ErrorLimit, "Error not acceptable");

} // namespace MathUtilities
