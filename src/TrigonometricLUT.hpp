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
 * @file   TrigonometricLUT.hpp
 * @brief  Construction of LUT for sin/cos using Q15 fixed-point arithmetic (compile-time via GCEM)
 * @author Georgios Gkogkou <ggkogkou125@gmail.com>
 */

#pragma once

#include <array>
#include <cstdint>
#include <numbers>
#include "gcem.hpp"

namespace TrigonometricLUT {

/**
 * Sine LUT in Q15 for a 14-bit encoder (0..16383)
 */
template <std::size_t N = 4096>
struct SineLookUpTableQ15 {
        static_assert(N % 4 == 0, "LUT size must be dividable by 4");
        static_assert((N & (N - 1)) == 0, "LUT size must be a power of two");

        static constexpr int32_t TWO_PI_MRAD = 6283;
        static constexpr std::size_t ENCODER_COUNTS = 16384;
        static constexpr std::size_t QUARTER = N / 4;

        using LookUpTable = std::array<int16_t, N>;

        static consteval int16_t floatToQ15(double x) {
                // clamp to [-1, +1) for Q15
                if (x >= 0.9999694824)
                        x = 0.9999694824; // 32767/32768
                if (x <= -1.0)
                        x = -1.0;
                return static_cast<int16_t>(x * 32768.0);
        }

        static consteval LookUpTable generateLookUpTable() {
                LookUpTable lut{};

                constexpr double TWO_PI = 2.0 * std::numbers::pi_v<double>;

                for (std::size_t i = 0; i <= QUARTER; ++i) {
                        const double theta = TWO_PI * static_cast<double>(i) / static_cast<double>(N);
                        lut[i] = floatToQ15(gcem::sin(theta));
                }

                for (std::size_t i = QUARTER + 1; i < 2 * QUARTER; ++i)
                        lut[i] = lut[2 * QUARTER - i];

                for (std::size_t i = 2 * QUARTER; i <= 3 * QUARTER; ++i)
                        lut[i] = static_cast<int16_t>(-lut[i - 2 * QUARTER]);

                for (std::size_t i = 3 * QUARTER + 1; i < N; ++i)
                        lut[i] = static_cast<int16_t>(-lut[4 * QUARTER - i]);

                return lut;
        }

        static constexpr LookUpTable sineLUT_Flash = generateLookUpTable();
        static constexpr LookUpTable sineLUT __attribute__((section(".ram_lut"), used)) = generateLookUpTable();

        /**
         * Operator[] for raw encoder counts (0..16383)
         */
        constexpr int16_t operator[](uint16_t encoder14) const noexcept {
                const uint32_t idx = (static_cast<uint32_t>(encoder14) * static_cast<uint32_t>(N)) >> 14;
                return sineLUT[static_cast<std::size_t>(idx) & (N - 1)];
        }

        /**
         * Operator[] for milli-radians (mrad)
         */
        constexpr int16_t operator[](int32_t theta_mrad) const noexcept {
                int32_t w = theta_mrad % TWO_PI_MRAD;
                if (w < 0)
                        w += TWO_PI_MRAD;

                const auto idx = static_cast<std::size_t>((static_cast<int64_t>(w) * static_cast<int64_t>(N)) /
                                                          static_cast<int64_t>(TWO_PI_MRAD)) &
                        (N - 1);

                return sineLUT[idx];
        }
};

/**
 * Cosine LUT by quarter-cycle shift of sine LUT
 */
template <std::size_t N = 4096>
struct CosineLookUpTableQ15 : private SineLookUpTableQ15<N> {
        using Base = SineLookUpTableQ15<N>;

        constexpr int16_t operator[](uint16_t encoder14) const noexcept {
                const uint32_t idx = (static_cast<uint32_t>(encoder14) * static_cast<uint32_t>(N)) >> 14;
                return Base::sineLUT[(static_cast<std::size_t>(idx) + Base::QUARTER) & (N - 1)];
        }

        constexpr int16_t operator[](int32_t theta_mrad) const noexcept {
                int32_t w = theta_mrad % Base::TWO_PI_MRAD;
                if (w < 0)
                        w += Base::TWO_PI_MRAD;

                const auto idx = static_cast<std::size_t>((static_cast<int64_t>(w) * static_cast<int64_t>(N)) /
                                                          static_cast<int64_t>(Base::TWO_PI_MRAD));

                return Base::sineLUT[(idx + Base::QUARTER) & (N - 1)];
        }
};

inline constexpr SineLookUpTableQ15<4096> sine_q15_14bit{};
inline constexpr CosineLookUpTableQ15<4096> cosine_q15_14bit{};

} // namespace TrigonometricLUT

/**
 * Compile-time checks
 */
namespace TrigonometricLUT::LUT_Tests {
constexpr int32_t absolute_q31(int32_t x) {
        return (x < 0) ? -x : x;
}

constexpr bool close_q15(int16_t actual, int16_t expected, int32_t tol) {
        return absolute_q31(static_cast<int32_t>(actual) - static_cast<int32_t>(expected)) <= tol;
}

constexpr int16_t Q15_ONE = 32767;
constexpr int16_t Q15_ZERO = 0;

constexpr uint16_t QTR = 4096u;
constexpr uint16_t HALF = 8192u;
constexpr uint16_t THREE_QTR = 12288u;

constexpr int32_t TOL_ONE = 300; // about 0.009 in pu
constexpr int32_t TOL_ZERO = 8;

static_assert(close_q15(sine_q15_14bit[uint16_t{0}], Q15_ZERO, TOL_ZERO), "sin(0) should be ~0");
static_assert(close_q15(sine_q15_14bit[QTR], Q15_ONE, TOL_ONE), "sin(pi/2) should be ~+1");
static_assert(close_q15(sine_q15_14bit[HALF], Q15_ZERO, TOL_ZERO), "sin(pi) should be ~0");
static_assert(close_q15(sine_q15_14bit[THREE_QTR], -Q15_ONE, TOL_ONE), "sin(3pi/2) should be ~-1");
static_assert(close_q15(cosine_q15_14bit[uint16_t{0}], Q15_ONE, TOL_ONE), "cos(0) should be ~+1");
static_assert(close_q15(cosine_q15_14bit[QTR], Q15_ZERO, TOL_ZERO), "cos(pi/2) should be ~0");

constexpr int32_t s0 = static_cast<int32_t>(sine_q15_14bit[uint16_t{1234}]);
constexpr int32_t c0 = static_cast<int32_t>(cosine_q15_14bit[uint16_t{1234}]);
constexpr int64_t mag2 = static_cast<int64_t>(s0) * s0 + static_cast<int64_t>(c0) * c0;
static_assert(absolute_q31(static_cast<int32_t>(mag2 - 1073741824LL)) < 25'000'000, "sin^2+cos^2 should be near 1");

constexpr int32_t TWO_PI_MRAD = SineLookUpTableQ15<4096>::TWO_PI_MRAD;
static_assert(sine_q15_14bit[int32_t{-100}] == sine_q15_14bit[TWO_PI_MRAD - 100], "mrad wrap should match");

} // namespace TrigonometricLUT::LUT_Tests
