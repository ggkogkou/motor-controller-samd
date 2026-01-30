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
 * @file   Telemetry.hpp
 * @brief  A telemetry implementation for parameter monitoring to be used along with the FOC algorithm
 * @author Georgios Gkogkou <ggkogkou125@gmail.com>
 */

#pragma once

#include <algorithm>
#include <span>
#include <type_traits>
#include "USART_TxStream.hpp"
#include "definitions.h"
#include <cstdint>

namespace Telemetry {

/**
 * @struct TelemetryParameters
 *
 * A collection of the parameters that can be monitored during the algorithm execution
 */
struct TelemetryParameters {
        uint32_t seq = 0;
        uint32_t t_us = 0;

        int32_t ia_mA = 0;
        int32_t ib_mA = 0;
        int32_t ic_mA = 0;

        int32_t vd_mV = 0;
        int32_t vq_mV = 0;

        int32_t id_mA = 0;
        int32_t iq_mA = 0;
        int32_t id_ref_mA = 0;
        int32_t iq_ref_mA = 0;
        int32_t id_err_mA = 0;
        int32_t iq_err_mA = 0;

        int32_t vd_i_mV = 0;
        int32_t vq_i_mV = 0;

        int32_t angle_mrad = 0;
        int32_t omega_mrad_s = 0;

        uint32_t adc_seq = 0;
        uint32_t missed_pairs = 0;

        int32_t adc_u_raw = 0;
        int32_t adc_v_raw = 0;
        int32_t adc_u_off = 0;
        int32_t adc_v_off = 0;
};

/**
 * A set of compile-time checks verifying whether the TelemetryParameters struct is aligned correctly
 */
namespace Tests {

static_assert(std::is_trivially_copyable_v<TelemetryParameters>);
static_assert(sizeof(TelemetryParameters) == 92, "TelemetryParameters size changed");
static_assert(sizeof(TelemetryParameters) % 4 == 0);

} // namespace Tests

class TelemetryLogger {
public:
        /**
         * Constructor is the default compiler-generated
         */
        TelemetryLogger() = default;

        /**
         * Function that when called by the producer process, stores a complete TelemetryParameters snapshot and marks it as available
         *
         * @param sample The TelemetryParameters snapshot
         */
        __attribute__((always_inline)) void updateLatest(const TelemetryParameters& sample) {
                const uint32_t NextSample = 1 - latestBufferIndex;

                buffers[NextSample] = sample;

                __DMB();

                latestBufferIndex = NextSample;
                availableSnapshot = true;
        }

        /**
         * Function that when called by the consumer process and if there is a new snapshot, copies it out and marks it as consumed
         * @param encodedFrame
         * @return
         */
        __attribute__((always_inline)) [[nodiscard]] bool tryTakeLatest(TelemetryParameters& encodedFrame) {
                if (not availableSnapshot)
                        return false;

                const uint32_t SampleIndex = latestBufferIndex;

                __DMB();

                encodedFrame = buffers[SampleIndex];

                availableSnapshot = false;
                return true;
        }

        /**
         * Function that returns a reference to the most recent TelemetryParameters object
         * @return
         */
        [[nodiscard]] const TelemetryParameters& latest() const {
                return buffers[latestBufferIndex];
        }

        /**
         * Function that encodes the data into a frame ready to be transmitted by USART Tx
         *
         * @param tp The struct that contains the parameters that are monitored and encoded into the frame
         * @param encodedFrame The encoded binary frame
         * @return Number of bytes encoded
         */
        size_t encodeFrame(const TelemetryParameters& tp, std::span<uint8_t> encodedFrame);

        /**
         * Function that takes care of writing to the USART TX after encoding the struct
         * @param usart The USART handle implementation
         * @return True if succeeded to place the process
         */
        bool writeFrame(USART_TxStream& usart);

private:
        /**
         * Function that performs some CRC checking for validating whether a frame is valid or corrupted
         * @param data
         * @param crc
         * @return
         */
        [[nodiscard]] uint16_t crc16_ccitt(std::span<const uint8_t> data, uint16_t crc);

        /**
         * An array of TelemetryParameters objects used to store and manage telemetry data
         */
        TelemetryParameters buffers[2]{};

        /**
         * The index of the most recently buffered snapshot/sample
         */
        volatile uint32_t latestBufferIndex = 0;

        /**
         * Cache whether there is a snapshot ready to be transmitted
         */
        volatile bool availableSnapshot = false;

        /**
         * First synchronization byte
         */
        static constexpr uint8_t SyncByte0 = 0xA5;

        /**
         * Second synchronization byte
         */
        static constexpr uint8_t SyncByte1 = 0x5A;
        static constexpr uint8_t TypeOfTelemetry = 0x01;

        /**
         * The number of bytes that will be sent through telemetry
         */
        static constexpr size_t PayloadSize = sizeof(TelemetryParameters);

        /**
         * TypeOfTelemetry word length in bytes
         */
        static constexpr size_t TypeOfTelemetrySizeBytes = 1;

        /**
         * The size (in bytes) of the word carrying the information about the payload length
         */
        static constexpr size_t LengthSizeBytes = 1;

        /**
         * The number of bytes that are used for syncing with this encoding
         */
        static constexpr size_t SyncSizeBytes = 2;

        /**
         * The number of bytes that are used for CRC calculations with this encoding
         */
        static constexpr size_t CRC_SizeBytes = 2;

        /**
         * The total length of the payload and type information
         */
        static constexpr uint8_t Length = TypeOfTelemetrySizeBytes + PayloadSize;

        /**
         * The total frame size in bytes
         */
        static constexpr size_t FrameSizeBytes =
                SyncSizeBytes + LengthSizeBytes + TypeOfTelemetrySizeBytes + PayloadSize + CRC_SizeBytes;
};
} // namespace Telemetry
