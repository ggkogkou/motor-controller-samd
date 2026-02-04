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
 * @file   as5047p_config.hpp
 * @brief  Configuration structure for the AS5047P magnetic encoder
 * @author Georgios Gkogkou <ggkogkou125@gmail.com>
 */

#pragma once

#include <cstdint>

struct AS5047P_Config {
        enum class RotationDirection : std::uint16_t {
                CLOCKWISE = 0b0000'0000,
                COUNTER_CLOCKWISE = 0b0000'0100,
        };

        enum class PWM_OutputPin : std::uint16_t {
                PIN_W = 0b0000'0000, // ABI is operating , I is used as PWM
                PIN_I = 0b0000'1000, // UVW is operating , I is used as PWM
        };

        enum class DAEC_Status : std::uint16_t {
                ENABLED = 0b0000'0000,
                DISABLED = 0b0001'0000,
        };

        enum class DataSelect : std::uint16_t {
                DAEC_ANG = 0b0000'0000,
                CORDIC_ANG = 0b0100'0000,
        };

        enum class PWM_Status : std::uint16_t {
                ENABLED = 0b1000'0000,
                DISABLED = 0b0000'0000,
        };

        /**
         * The 14-bit zero position (default 0x0000)
         */
        std::uint16_t zeroPosition = 0x0000;

        /**
         * The rotation direction, bit DIR of SETTINGS1
         */
        RotationDirection rotationDirection = RotationDirection::CLOCKWISE;

        /**
         * Bit UVW_ABI of SETTINGS1
         */
        PWM_OutputPin pwmOutputPin = PWM_OutputPin::PIN_W;

        /**
         * Bit DAECDIS of SETTINGS1
         */
        DAEC_Status dynamicAngleCompensation = DAEC_Status::ENABLED;

        /**
         * Bit Dataselect of SETTINGS1
         */
        DataSelect dataSelect = DataSelect::DAEC_ANG;

        /**
         * Bit PWMon of SETTINGS1
         */
        PWM_Status pwmStatus = PWM_Status::DISABLED;

        enum class PolePairs : std::uint8_t {
                ONE = 0b000,
                TWO = 0b001,
                THREE = 0b010,
                FOUR = 0b011,
                FIVE = 0b100,
                SIX = 0b101,
                SEVEN = 0b110,
                EIGHT = 0b111,
        };

        enum class HysteresisBits : std::uint8_t {
                THREE = 0b00,
                TWO = 0b01,
                ONE = 0b10,
                ZERO = 0b11,
        };

        enum class ABI_Resolution : std::uint8_t {
                ABIRES_000 = 0b000,
                ABIRES_001 = 0b001,
                ABIRES_010 = 0b010,
                ABIRES_011 = 0b011,
                ABIRES_100 = 0b100,
                ABIRES_101 = 0b101,
                ABIRES_110 = 0b110,
                ABIRES_111 = 0b111,
        };

        enum class ABIBIN : std::uint8_t {
                DECIMAL = 0,
                BINARY = 1,
        };
};
