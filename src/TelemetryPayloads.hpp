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
 * @file   TelemetryPayloads.hpp
 * @brief  A collection of telemetry payloads with different sizes and parameters
 * @author Georgios Gkogkou <ggkogkou125@gmail.com>
 */

#pragma once

#include <cstdint>
#include <type_traits>

namespace Telemetry {

/**
 * @struct TelemetryPayload44
 *
 * A collection of the parameters that can be monitored during the algorithm execution
 */
struct TelemetryPayload44 {
        int32_t dirSign = 0;
        uint32_t ZeroOffsetElectricalAngle = 0;
        uint32_t ThetaEl = 0;
        int32_t ia_mA = 0;
        int32_t ib_mA = 0;
        uint32_t adcOffsetU = 0;
        uint32_t adcOffsetV = 0;
        int32_t iq_ref_mA = 0;
        uint32_t angle_raw = 0;
        uint32_t runtime_mem_corruption_err = 0;
        uint32_t encoder_err = 0;
};

/**
 * A set of compile-time checks verifying whether the TelemetryParameters struct is aligned correctly
 */
namespace Tests {

static_assert(std::is_trivially_copyable_v<TelemetryPayload44>);
static_assert(sizeof(TelemetryPayload44) == 44, "TelemetryParameters size changed");
static_assert(sizeof(TelemetryPayload44) % 4 == 0);

} // namespace Tests
} // namespace Telemetry
