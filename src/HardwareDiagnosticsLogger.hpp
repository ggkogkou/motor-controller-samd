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
 * @file   HardwareDiagnosticsLogger.hpp
 * @brief  Lightweight UART logger for hardware diagnostic events
 * @author Georgios Gkogkou <ggkogkou125@gmail.com>
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include "USART_TxStream.hpp"

namespace HardwareDiagnostics {
class HardwareDiagnosticsLogger {
public:
        /**
         * Constructor
         *
         * @param usart The USART TX stream used for diagnostic logging
         */
        explicit HardwareDiagnosticsLogger(USART_TxStream& usart) : usart(usart) {}

        /**
         * Send a raw string literal.
         *
         * @param message Null-terminated string literal
         * @return True if the message was submitted to the TX stream
         */
        bool writeLiteral(const char* message);

        /**
         * Send a raw byte span.
         *
         * @param bytes Byte span to transmit
         * @return True if the message was submitted to the TX stream
         */
        bool write(std::span<const uint8_t> bytes);

        /**
         * Pre-configured diagnostic messages
         */
        bool logBoot();
        bool logResetPOR();
        bool logResetWDT();
        bool logResetSoftware();
        bool logResetExternal();
        bool logResetBOD33();
        bool logResetUnknown();
        bool logFaultEncoder();
        bool logFaultMemoryCorruption();
        bool logStartupCalibrationBegin();
        bool logStartupCalibrationDone();
        bool logClosedLoopEntered();
        bool logMotorStopped();

private:
        /**
         * Helper that returns the length of a null-terminated string
         *
         * @param message Null-terminated string
         * @return Length in bytes excluding the null terminator
         */
        [[nodiscard]] static std::size_t stringLength(const char* message);

        /**
         * USART stream used for diagnostics output
         */
        USART_TxStream& usart;
};

inline USART_TxStream diagnostics{DMAC_CHANNEL_1};
inline HardwareDiagnosticsLogger diagnosticsLogger{diagnostics};

} // namespace HardwareDiagnostics
