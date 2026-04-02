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

bool HardwareDiagnosticsLogger::writeLiteral(const char* message) {
        if (message == nullptr)
                return false;

        const std::size_t length = stringLength(message);
        if (length == 0)
                return false;

        const auto* bytes = reinterpret_cast<const uint8_t*>(message);
        return write(std::span<const uint8_t>(bytes, length));
}

bool HardwareDiagnosticsLogger::logBoot() {
        return writeLiteral("BOOT\r\n");
}

bool HardwareDiagnosticsLogger::logResetPOR() {
        return writeLiteral("RESET: POR\r\n");
}

bool HardwareDiagnosticsLogger::logResetWDT() {
        return writeLiteral("RESET: WDT\r\n");
}

bool HardwareDiagnosticsLogger::logResetSoftware() {
        return writeLiteral("RESET: SOFTWARE\r\n");
}

bool HardwareDiagnosticsLogger::logResetExternal() {
        return writeLiteral("RESET: EXTERNAL\r\n");
}

bool HardwareDiagnosticsLogger::logResetBOD33() {
        return writeLiteral("RESET: BOD33\r\n");
}

bool HardwareDiagnosticsLogger::logResetUnknown() {
        return writeLiteral("RESET: UNKNOWN\r\n");
}

bool HardwareDiagnosticsLogger::logFaultEncoder() {
        return writeLiteral("FAULT: ENCODER\r\n");
}

bool HardwareDiagnosticsLogger::logFaultMemoryCorruption() {
        return writeLiteral("FAULT: MEMORY_CORRUPTION\r\n");
}

bool HardwareDiagnosticsLogger::logStartupCalibrationBegin() {
        return writeLiteral("STATE: STARTUP_CALIBRATION_BEGIN\r\n");
}

bool HardwareDiagnosticsLogger::logStartupCalibrationDone() {
        return writeLiteral("STATE: STARTUP_CALIBRATION_DONE\r\n");
}

bool HardwareDiagnosticsLogger::logClosedLoopEntered() {
        return writeLiteral("STATE: CLOSED_LOOP\r\n");
}

bool HardwareDiagnosticsLogger::logMotorStopped() {
        return writeLiteral("STATE: MOTOR_STOPPED\r\n");
}
