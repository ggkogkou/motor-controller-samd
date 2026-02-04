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
 * @file   VelocityEstimator.hpp
 * @brief  Angle velocity estimator used in the velocity control loop
 * @author Georgios Gkogkou <ggkogkou125@gmail.com>
 */

#pragma once

#include <cstdint>

namespace PermanentMagnetSynchronousMotor {

/**
 * @struct AngleVelocityEstimator
 *
 * Implementation of a velocity estimator to be used in the velocity control loop
 */
struct AngleVelocityEstimator {
        int32_t lastWrappedAngle; // mrad
        int32_t unwrappedAngle; // mrad
        int32_t angularVelocity; // mrad/sec
        uint32_t filterTimeConstant; // usec

        /**
         * Useful constants
         */
        static constexpr int32_t TWO_PI_MRAD = 6283;
        static constexpr int32_t PI_MRAD = TWO_PI_MRAD / 2;

        /**
         * Class constructor
         *
         * @param initialWrappedAngle The initial wrapped angle (in mrad)
         * @param tau The change of time dT (in μsec)
         */
        explicit AngleVelocityEstimator(int32_t initialWrappedAngle, uint32_t tau = 10'000) :
            lastWrappedAngle(initialWrappedAngle), unwrappedAngle(initialWrappedAngle), angularVelocity(0), filterTimeConstant(tau) {}

        /**
         * Function that must be called periodically in order to run the velocity control loop
         *
         * @param wrappedAngle The wrapped angle (in mrad)
         * @param deltaTime The change of time dT (in μsec)
         */
        void update(int32_t wrappedAngle, uint32_t deltaTime) {
                const auto wrapDelta = [&](int32_t a, int32_t b) -> int32_t {
                        int32_t d = a - b;
                        if (d > PI_MRAD)
                                d -= TWO_PI_MRAD;
                        if (d < -PI_MRAD)
                                d += TWO_PI_MRAD;
                        return d;
                };

                const auto derivative_mrad_per_sec = [&](int32_t delta, uint32_t dt_us) -> int32_t {
                        if (dt_us == 0u)
                                return 0;
                        const auto num = static_cast<int64_t>(delta) * 1'000'000LL;
                        return static_cast<int32_t>(num / static_cast<int64_t>(dt_us));
                };

                const auto alpha_q15 = [&](uint32_t tau_us, uint32_t dt_us) -> int32_t {
                        const uint32_t Denominator = tau_us + dt_us;
                        if (Denominator == 0u)
                                return 0;
                        const auto a = (static_cast<int64_t>(tau_us) << 15) / static_cast<int64_t>(Denominator);
                        if (a < 0)
                                return 0;
                        if (a > 32768)
                                return 32768;
                        return static_cast<int32_t>(a);
                };

                const int32_t delta = wrapDelta(wrappedAngle, lastWrappedAngle);
                unwrappedAngle += delta;

                const int32_t rawDerivative = derivative_mrad_per_sec(delta, deltaTime);

                const int32_t a_q15 = alpha_q15(filterTimeConstant, deltaTime);
                const int32_t one_minus_a_q15 = 32768 - a_q15;

                const auto filt = static_cast<int64_t>(a_q15) * static_cast<int64_t>(angularVelocity) +
                        static_cast<int64_t>(one_minus_a_q15) * static_cast<int64_t>(rawDerivative);

                angularVelocity = static_cast<int32_t>(filt >> 15);
                lastWrappedAngle = wrappedAngle;
        }
};

} // namespace PermanentMagnetSynchronousMotor
