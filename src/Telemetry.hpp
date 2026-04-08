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
#include <array>
#include <cstdint>
#include <span>
#include <type_traits>
#include "USART_TxStream.hpp"
#include "definitions.h"
#include "TelemetryPayloads.hpp"

namespace Telemetry {

template <typename TelemetryParameters>
class TelemetryLogger {
public:
        static_assert(std::is_trivially_copyable_v<TelemetryParameters>);
        static_assert(sizeof(TelemetryParameters) % 4 == 0);

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
                const uint32_t NextSample = 1U - latestBufferIndex;

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
        size_t encodeFrame(const TelemetryParameters& tp, std::span<uint8_t> encodedFrame) {
                if (encodedFrame.size() < FrameSizeBytes)
                        return 0;

                size_t i = 0;

                encodedFrame[i++] = SyncByte0;
                encodedFrame[i++] = SyncByte1;
                encodedFrame[i++] = Length;
                encodedFrame[i++] = TypeOfTelemetry;

                // std::memcpy(&encodedFrame[i], &tp, PayloadSize);

                const auto* src = reinterpret_cast<const uint8_t*>(&tp);
                std::copy_n(src, static_cast<std::ptrdiff_t>(PayloadSize), encodedFrame.data() + i);

                i += PayloadSize;

                const uint16_t CRC =
                        crc16_ccitt(std::span<const uint8_t>(&encodedFrame[3], TypeOfTelemetrySizeBytes + PayloadSize), 0xFFFF);

                encodedFrame[i++] = static_cast<uint8_t>(CRC & 0xFF);
                encodedFrame[i++] = static_cast<uint8_t>((CRC >> 8) & 0xFF);

                return i;
        }

        /**
         * Function that takes care of writing to the USART TX after encoding the struct
         * @param usart The USART handle implementation
         * @return True if succeeded to place the process
         */
        bool writeFrame(USART_TxStream& usart) {
                TelemetryParameters TelemetryParams{};

                if (not tryTakeLatest(TelemetryParams))
                        return false;

                std::array<uint8_t, FrameSizeBytes> txEncodedBuffer{};

                const auto FrameWrittenBytes = encodeFrame(TelemetryParams, std::span(txEncodedBuffer.data(), txEncodedBuffer.size()));

                if (FrameWrittenBytes == 0)
                        return false;

                usart.write(std::span<const uint8_t>(txEncodedBuffer.data(), FrameWrittenBytes));

                return true;
        }

private:
        /**
         * Function that performs some CRC checking for validating whether a frame is valid or corrupted
         * @param data
         * @param crc
         * @return
         */
        [[nodiscard]] uint16_t crc16_ccitt(std::span<const uint8_t> data, uint16_t crc) {
                for (const auto b : data) {
                        crc ^= static_cast<uint16_t>(b) << 8;

                        for (int i = 0; i < 8; ++i)
                                crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021) : static_cast<uint16_t>(crc << 1);
                }

                return crc;
        }

        /**
         * An array of TelemetryParameters objects used to store and manage telemetry data
         */
        std::array<TelemetryParameters, 2> buffers {};

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
        static constexpr size_t Length = static_cast<uint8_t>(TypeOfTelemetrySizeBytes + PayloadSize);

        /**
         * The total frame size in bytes
         */
        static constexpr size_t FrameSizeBytes =
                SyncSizeBytes + LengthSizeBytes + TypeOfTelemetrySizeBytes + PayloadSize + CRC_SizeBytes;
};

} // namespace Telemetry
