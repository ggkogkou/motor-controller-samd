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
 * @file   Telemetry.cpp
 * @brief  A telemetry implementation for parameter monitoring to be used along with the FOC algorithm
 * @author Georgios Gkogkou <ggkogkou125@gmail.com>
 */

#include "Telemetry.hpp"

namespace Telemetry {

size_t TelemetryLogger::encodeFrame(const TelemetryParameters& tp, std::span<uint8_t> encodedFrame) {
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

        const uint16_t CRC = crc16_ccitt(std::span<const uint8_t>(&encodedFrame[3], TypeOfTelemetrySizeBytes + PayloadSize), 0xFFFF);

        encodedFrame[i++] = static_cast<uint8_t>(CRC & 0xFF);
        encodedFrame[i++] = static_cast<uint8_t>((CRC >> 8) & 0xFF);

        return i;
}

bool TelemetryLogger::writeFrame(USART_TxStream& usart) {
        TelemetryParameters TelemetryParams;

        if (not tryTakeLatest(TelemetryParams))
                return false;

        std::array<uint8_t, FrameSizeBytes> txEncodedBuffer{};

        const auto FrameWrittenBytes = encodeFrame(TelemetryParams, std::span(txEncodedBuffer.data(), txEncodedBuffer.size()));

        if (FrameWrittenBytes == 0)
                return false;

        usart.write(std::span<const uint8_t>(txEncodedBuffer.data(), FrameWrittenBytes));

        return true;
}

uint16_t TelemetryLogger::crc16_ccitt(std::span<const uint8_t> data, uint16_t crc) {
        for (const auto b : data) {
                crc ^= static_cast<uint16_t>(b) << 8;

                for (int i = 0; i < 8; ++i)
                        crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021) : static_cast<uint16_t>(crc << 1);
        }

        return crc;
}
} // namespace Telemetry
