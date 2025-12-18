#pragma once

#include <array>
#include <cstdint>
#include "sin_cos_lut_q15.hpp"

namespace MathUtils {

/**
 * Fixed-point frame types (int32_t in your chosen physical units: mA, mV, etc.)
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
 * Multiply an int32 value by a Q15 coefficient -> int32 result (same unit as value).
 * Uses rounding before shifting.
 */
[[nodiscard]] inline int32_t mul_q15(int32_t value, int16_t q15) noexcept {
        const int64_t prod = static_cast<int64_t>(value) * static_cast<int64_t>(q15);
        const int64_t round = (prod >= 0) ? (1LL << (Q15_SHIFT - 1)) : -(1LL << (Q15_SHIFT - 1));
        return static_cast<int32_t>((prod + round) >> Q15_SHIFT);
}

/**
 * -----------------------------------------
 * Fixed-point versions (new overloads)
 * -----------------------------------------
 *
 * Angle variants:
 *  - uint16_t theta14 : raw 14-bit angle counts (0..16383) (recommended)
 *  - int32_t  theta_mrad : milli-radians
 */

/**
 * Park transform (int32, theta in 14-bit counts)
 */
[[nodiscard]] inline DQFrame_i32 performParkTransform(int32_t Ua, int32_t Ub, uint16_t theta14) noexcept {
        const int16_t CosineTheta = MathUtilities::cosine_q15_14bit[theta14];
        const int16_t SineTheta   = MathUtilities::sine_q15_14bit[theta14];

        const int32_t Ud = mul_q15(Ua, CosineTheta) + mul_q15(Ub, SineTheta);
        const int32_t Uq = -mul_q15(Ua, SineTheta) + mul_q15(Ub, CosineTheta);

        return {Ud, Uq};
}

/**
 * Park transform (int32, theta in mrad)
 */
[[nodiscard]] inline DQFrame_i32 performParkTransform(int32_t Ua, int32_t Ub, int32_t theta_mrad) noexcept {
        const int16_t CosineTheta = MathUtilities::cosine_q15_14bit[theta_mrad];
        const int16_t SineTheta   = MathUtilities::sine_q15_14bit[theta_mrad];

        const int32_t Ud = mul_q15(Ua, CosineTheta) + mul_q15(Ub, SineTheta);
        const int32_t Uq = -mul_q15(Ua, SineTheta) + mul_q15(Ub, CosineTheta);

        return {Ud, Uq};
}

/**
 * Inverse Park transform (int32, theta in 14-bit counts)
 */
[[nodiscard]] inline AlphaBetaFrame_i32 performInverseParkTransform(int32_t Ud, int32_t Uq, uint16_t theta14) noexcept {
        const int16_t CosineTheta = MathUtilities::cosine_q15_14bit[theta14];
        const int16_t SineTheta   = MathUtilities::sine_q15_14bit[theta14];

        const int32_t Ualpha = mul_q15(Ud, CosineTheta) - mul_q15(Uq, SineTheta);
        const int32_t Ubeta  = mul_q15(Ud, SineTheta) + mul_q15(Uq, CosineTheta);

        return {Ualpha, Ubeta};
}

/**
 * Inverse Park transform (int32, theta in mrad)
 */
[[nodiscard]] inline AlphaBetaFrame_i32 performInverseParkTransform(int32_t Ud, int32_t Uq, int32_t theta_mrad) noexcept {
        const int16_t CosineTheta = MathUtilities::cosine_q15_14bit[theta_mrad];
        const int16_t SineTheta   = MathUtilities::sine_q15_14bit[theta_mrad];

        const int32_t Ualpha = mul_q15(Ud, CosineTheta) - mul_q15(Uq, SineTheta);
        const int32_t Ubeta  = mul_q15(Ud, SineTheta) + mul_q15(Uq, CosineTheta);

        return {Ualpha, Ubeta};
}

/**
 * Clarke transform (int32)
 *
 * Matches your current float formula:
 *  Ualpha = Ua
 *  Ubeta  = (Ua + 2*Ub) / sqrt(3)
 */
[[nodiscard]] inline AlphaBetaFrame_i32 performClarkeTransform(int32_t Ua, int32_t Ub, int32_t /*Uc*/ = 0) noexcept {
        const int32_t Ualpha = Ua;
        const int32_t Ubeta  = mul_q15(Ua + 2 * Ub, INV_SQRT3_Q15);
        return {Ualpha, Ubeta};
}

/**
 * Chained Clarke + Park (int32, theta in 14-bit counts)
 */
[[nodiscard]] inline DQFrame_i32 performClarkeParkTransforms(int32_t Ua, int32_t Ub, uint16_t theta14) noexcept {
        const int32_t Ualpha = Ua;
        const int32_t Ubeta  = mul_q15(Ua + 2 * Ub, INV_SQRT3_Q15);

        const int16_t CosineTheta = MathUtilities::cosine_q15_14bit[theta14];
        const int16_t SineTheta   = MathUtilities::sine_q15_14bit[theta14];

        const int32_t Ud = mul_q15(Ualpha, CosineTheta) + mul_q15(Ubeta, SineTheta);
        const int32_t Uq = -mul_q15(Ualpha, SineTheta) + mul_q15(Ubeta, CosineTheta);

        return {Ud, Uq};
}

/**
 * Chained Clarke + Park (int32, theta in mrad)
 */
[[nodiscard]] inline DQFrame_i32 performClarkeParkTransforms(int32_t Ua, int32_t Ub, int32_t theta_mrad) noexcept {
        const int32_t Ualpha = Ua;
        const int32_t Ubeta  = mul_q15(Ua + 2 * Ub, INV_SQRT3_Q15);

        const int16_t CosineTheta = MathUtilities::cosine_q15_14bit[theta_mrad];
        const int16_t SineTheta   = MathUtilities::sine_q15_14bit[theta_mrad];

        const int32_t Ud = mul_q15(Ualpha, CosineTheta) + mul_q15(Ubeta, SineTheta);
        const int32_t Uq = -mul_q15(Ualpha, SineTheta) + mul_q15(Ubeta, CosineTheta);

        return {Ud, Uq};
}

} // namespace MathUtils
