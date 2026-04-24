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
 * @file   HardwareDiagnosticsLogger.cpp
 * @brief  Lightweight UART logger for hardware diagnostic events
 * @author Georgios Gkogkou <ggkogkou125@gmail.com>
 */

#include "HardwareDiagnosticsLogger.hpp"

namespace HardwareDiagnostics {

std::size_t HardwareDiagnosticsLogger::stringLength(const char* message) {
        if (message == nullptr)
                return 0;

        std::size_t length = 0;
        while (message[length] != '\0')
                ++length;

        return length;
}

bool HardwareDiagnosticsLogger::write(std::span<const uint8_t> bytes) {
        if (bytes.empty())
                return false;

        usart.write(bytes);
        return true;
}

bool HardwareDiagnosticsLogger::writeString(const char* message) {
        if (message == nullptr)
                return false;

        const std::size_t length = stringLength(message);
        if (length == 0)
                return false;

        const auto* bytes = reinterpret_cast<const uint8_t*>(message);
        return write(std::span(bytes, length));
}

bool HardwareDiagnosticsLogger::logBoot() {
        return writeString("\n\r\n\rBOOT\r\n");
}

bool HardwareDiagnosticsLogger::logResetPOR() {
        return writeString("RESET: POR\r\n");
}

bool HardwareDiagnosticsLogger::logResetWDT() {
        return writeString("RESET: WDT\r\n");
}

bool HardwareDiagnosticsLogger::logResetSoftware() {
        return writeString("RESET: SOFTWARE\r\n");
}

bool HardwareDiagnosticsLogger::logResetExternal() {
        return writeString("RESET: EXTERNAL\r\n");
}

bool HardwareDiagnosticsLogger::logResetBOD33() {
        return writeString("RESET: BOD33\r\n");
}

bool HardwareDiagnosticsLogger::logResetUnknown() {
        return writeString("RESET: UNKNOWN\r\n");
}

bool HardwareDiagnosticsLogger::logFaultEncoder() {
        return writeString("FAULT: ENCODER\r\n");
}

bool HardwareDiagnosticsLogger::logFaultMemoryCorruption() {
        return writeString("FAULT: MEMORY_CORRUPTION\r\n");
}

bool HardwareDiagnosticsLogger::logStartupCalibrationBegin() {
        return writeString("STATE: STARTUP_CALIBRATION_BEGIN\r\n");
}

bool HardwareDiagnosticsLogger::logStartupCalibrationDone() {
        return writeString("STATE: STARTUP_CALIBRATION_DONE\r\n");
}

bool HardwareDiagnosticsLogger::logClosedLoopEntered() {
        return writeString("STATE: CLOSED_LOOP\r\n");
}

bool HardwareDiagnosticsLogger::logMotorStopped() {
        return writeString("STATE: MOTOR_STOPPED\r\n");
}

} // namespace HardwareDiagnostics
