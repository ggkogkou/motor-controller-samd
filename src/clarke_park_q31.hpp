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
 * @file   clarke_park_q31.hpp
 * @brief  Clarke and Park transformations using fixed-point arithmetic (based on the implementation of math_utils.hpp)
 * @author Georgios Gkogkou <ggkogkou125@gmail.com>
 */

#pragma once

#include <array>
#include <cstdint>
#include "attributes.hpp"
#include "sin_cos_lut_q15.hpp"

namespace MathUtils {

/**
 * Fixed-point frame types
 */
using DQFrame_i32 = std::array<int32_t, 2>;
using AlphaBetaFrame_i32 = std::array<int32_t, 2>;

/**
 * Float frame types (kept for compatibility)
 */
using DQFrame_f = std::array<float, 2>;
using AlphaBetaFrame_f = std::array<float, 2>;

/**
 * Q15 helpers/constants
 */
inline constexpr int32_t Q15_SHIFT = 15;
inline constexpr int16_t INV_SQRT3_Q15 = 18919; // round((1/sqrt(3))*32768)

/**
 * Multiply an int32 value by a Q15 coefficient -> int32 result (same unit as value)
 * Truncates toward -infinity for negative products (due to arithmetic right shift)
 */
[[nodiscard]] ALWAYS_INLINE int32_t mul_q15(int32_t value, int16_t q15) noexcept {
        const int64_t prod = static_cast<int64_t>(value) * static_cast<int64_t>(q15);
        return static_cast<int32_t>(prod >> Q15_SHIFT);
}

/**
 * Park transform (theta in 14-bit raw value)
 */
[[nodiscard]] ALWAYS_INLINE DQFrame_i32 performParkTransform(int32_t Ua, int32_t Ub, uint16_t theta14) noexcept {
        const int16_t CosineTheta = MathUtilities::cosine_q15_14bit[theta14];
        const int16_t SineTheta = MathUtilities::sine_q15_14bit[theta14];

        const int32_t Ud = mul_q15(Ua, CosineTheta) + mul_q15(Ub, SineTheta);
        const int32_t Uq = -mul_q15(Ua, SineTheta) + mul_q15(Ub, CosineTheta);

        return {Ud, Uq};
}

/**
 * Inverse Park transform (theta in 14-bit raw value)
 */
[[nodiscard]] ALWAYS_INLINE AlphaBetaFrame_i32 performInverseParkTransform(int32_t Ud, int32_t Uq, uint16_t theta14) noexcept {
        const int16_t CosineTheta = MathUtilities::cosine_q15_14bit[theta14];
        const int16_t SineTheta = MathUtilities::sine_q15_14bit[theta14];

        const int32_t Ualpha = mul_q15(Ud, CosineTheta) - mul_q15(Uq, SineTheta);
        const int32_t Ubeta = mul_q15(Ud, SineTheta) + mul_q15(Uq, CosineTheta);

        return {Ualpha, Ubeta};
}

/**
 * Clarke transform
 */
[[nodiscard]] ALWAYS_INLINE AlphaBetaFrame_i32 performClarkeTransform(int32_t Ua, int32_t Ub, int32_t /*Uc*/ = 0) noexcept {
        const int32_t Ualpha = Ua;
        const int32_t Ubeta = mul_q15(Ua + 2 * Ub, INV_SQRT3_Q15);
        return {Ualpha, Ubeta};
}

/**
 * Chained Clarke + Park (theta in 14-bit raw)
 */
[[nodiscard]] ALWAYS_INLINE DQFrame_i32 performClarkeParkTransforms(int32_t Ua, int32_t Ub, uint16_t theta14) noexcept {
        const int32_t Ualpha = Ua;
        const int32_t Ubeta = mul_q15(Ua + 2 * Ub, INV_SQRT3_Q15);

        const int16_t CosineTheta = MathUtilities::cosine_q15_14bit[theta14];
        const int16_t SineTheta = MathUtilities::sine_q15_14bit[theta14];

        const int32_t Ud = mul_q15(Ualpha, CosineTheta) + mul_q15(Ubeta, SineTheta);
        const int32_t Uq = -mul_q15(Ualpha, SineTheta) + mul_q15(Ubeta, CosineTheta);

        return {Ud, Uq};
}

} // namespace MathUtils
