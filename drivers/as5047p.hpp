// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Georgios Gkogkou <ggkogkou125@gmail.com>

/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>
 */

/**
 * @file   as5047p.hpp
 * @brief  Device driver for the AS5047P magnetic encoder (implements only SPI mode)
 * @author Georgios Gkogkou <ggkogkou125@gmail.com>
 */

#pragma once

#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <type_traits>
#include "as5047p_config.hpp"
#include "definitions.h"
#include "etl/expected.h"
#include "spi_buffer.hpp"

using namespace ATSAMD21_GGKOGKOU;

/**
 * @class AS5047P
 * @brief Represents an interface for interacting with the AS5047P magnetic rotary sensor
 *
 * The AS5047P is a high-resolution magnetic rotary encoder. This class provides
 * functionality to read the current angle value reported by the sensor
 *
 * SPI Transaction
 * ----------------
 * Consists of a 16-bit command frame follower by a 16-bit data frame
 *
 * Command frame => | PARC | RW | ADDR |
 *                  |  15  | 14 | 13:0 |
 *
 * Read data frame => | PARD | EF | DATA |
 *                    |  15  | 14 | 13:0 |
 *
 * SPI clock frequency has a maximum of 10MHz
 */
class AS5047P {
public:
        /**
         * Type aliases for the device registers addresses and data
         */
        using RegisterAddress_t = std::uint16_t;
        using RegisterData_t = std::uint16_t;
        using ReadWriteCommandMask_t = std::uint16_t;
        using AngleDegrees_t = float;
        using FieldMagnitude_t = RegisterData_t;

        /**
         * Default constructor of the driver class
         *
         * Meant to be used when default configurations are OK or device OTP is programmed
         */
        AS5047P();

        /**
         * Constructor that initializes the device with the user-defined configurations
         *
         * @param config The full user configuration of the device
         */
        explicit AS5047P(AS5047P_Config config);

        /**
         * Default destructor of the class
         */
        ~AS5047P() = default;

        AS5047P(const AS5047P&) = delete;
        AS5047P& operator=(const AS5047P&) = delete;
        AS5047P(AS5047P&&) = delete;
        AS5047P& operator=(AS5047P&&) = delete;

        /**
         * SPI_Request object; passed between the SPI_Buffer layer and AS5047 driver
         */
        SPI_Request spiRequest;

        /**
         * @enum RegisterAddress
         * @brief Lists all the register addresses (both volatile and non-volatile) for the AS5047P magnetic rotary
         * sensor
         *
         * This enumeration provides the complete list of both volatile and non-volatile register addresses
         *
         * Volatile registers are typically used for real-time data such as angle measurements or diagnostics
         *
         * Non-volatile registers are used for storing calibration or configuration settings that persist across power
         * cycles
         *
         * Each register has a 14-bit address and holds 14-bit data
         */
        enum class RegisterAddress : RegisterAddress_t {
                /// Volatile registers
                NOP = 0x0000, /// No operation
                ERRFL = 0x0001, /// Error register (Read-Only)
                PROG = 0x0003, /// Programming register (Read/Write)
                DIAAGC = 0x3FFC, /// Diagnostic and AGC (Read-Only)
                MAG = 0x3FFD, /// CORDIC magnitude (Read-Only)
                ANGLEUNC = 0x3FFE, /// Measured angle without dynamic angle error compensation (Read-Only)
                ANGLECOM = 0x3FFF, /// Measured angle with dynamic angle error compensation (Read-Only)

                /// Non-volatile registers
                ZPOSM = 0x0016, /// Zero position MSB (Read/Write/Program)
                ZPOSL = 0x0017, /// Zero position LSB/MAG diagnostic (Read/Write/Program)
                SETTINGS1 = 0x0018, /// Custom setting register 1 (Read/Write/Program)
                SETTINGS2 = 0x0019 /// Custom setting register 2 (Read/Write/Program)
        };

        /**
         * @enum AGC_DiagnosticsError
         *
         * A set of diagnostic features that can be read from, AGC register
         */
        /**
         * @enum AGC_DiagnosticsError
         * @brief Diagnostic flags derived from the DIAAGC register
         */
        enum class AGC_DiagnosticsError {
                MAGNETIC_FIELD_TOO_LOW,
                MAGNETIC_FIELD_TOO_HIGH,
                CORDIC_OVF,
                OFFSET_COMP_NOT_READY,
        };

        /**
         * @enum AGC_DiagnosticsReadError
         * @brief Errors reported when reading diagnostics
         */
        enum class AGC_DiagnosticsReadError : std::uint8_t {
                SENSOR_BUSY,
        };

        /**
         * @enum ReadError
         * @brief Error codes for read transactions and cached reads
         */
        enum class ReadError : std::uint8_t {
                NOT_READY,
                BUSY,
                SUBMIT_FAILED,
                PARITY_ERROR,
                ERROR_FLAG_SET,
                OUT_OF_RANGE,
                UNEXPECTED_STATE,
        };

        /**
         * @enum WriteError
         * @brief Error codes for write transactions
         */
        enum class WriteError : std::uint8_t {
                NOT_READY,
                BUSY,
                SUBMIT_FAILED,
                PARITY_ERROR,
                ERROR_FLAG_SET,
                UNEXPECTED_STATE,
        };

        /**
         * @struct ERRFL_Status
         * @brief Parsed ERRFL status flags with raw register value
         */
        struct ERRFL_Status {
                /// Parity error flag
                bool parityError = false;
                /// Invalid command flag
                bool invalidCommand = false;
                /// Framing error flag
                bool framingError = false;
                /// Raw ERRFL register contents
                RegisterData_t raw = 0;
        };

        /**
         * @struct AGC_DiagnosticsReport
         * @brief Parsed diagnostics report from DIAAGC
         */
        struct AGC_DiagnosticsReport {
                /// Automatic gain control value (AGC)
                std::uint8_t agcValue = 0;
                /// Array of diagnostics errors collected from DIAAGC
                std::array<AGC_DiagnosticsError, 4> errors{};
                /// Number of valid entries in @ref errors
                std::uint8_t errorCount = 0;
        };

        /**
         * Function that initiates a new SPI Read transaction and reports submission errors
         *
         * @param registerAddress
         * @return Expected success or error
         */
        etl::expected<void, ReadError> request(RegisterAddress registerAddress);

        /**
         * Function that initiates a new SPI Write transaction and reports submission errors
         *
         * @param registerAddress
         * @param data
         * @return Expected success or error
         */
        etl::expected<void, WriteError> writeRegister(RegisterAddress registerAddress, RegisterData_t data);

        /**
         * Function that returns the status of the last completed read transaction
         *
         * @return Expected success or error
         */
        etl::expected<void, ReadError> lastReadResult() const;

        /**
         * Function that returns the status of the last completed write transaction
         *
         * @return Expected success or error
         */
        etl::expected<void, WriteError> lastWriteResult() const;

        /**
         * Function that returns the last ERRFL status flags
         *
         * @return The last ERRFL status
         */
        ERRFL_Status lastErrflStatus() const;

        /**
         * Function that reads the compensated angle (DAEC output)
         *
         * @return The 14-bit raw register value for angle compensated
         */
        [[nodiscard]] etl::expected<RegisterData_t, ReadError> measureAngleCompensatedRaw() const;

        /**
         * Function that reads the uncompensated angle (skip DAEC)
         *
         * @return The 14-bit raw register value for angle uncompensated
         */
        [[nodiscard]] etl::expected<RegisterData_t, ReadError> measureAngleUncompensatedRaw() const;

        /**
         * Function that reads the CORDIC magnetic field magnitude
         *
         * @return The 14-bit measured magnetic field magnitude
         */
        [[nodiscard]] etl::expected<FieldMagnitude_t, ReadError> measureFieldMagnitude() const;

        /**
         * Utility function to convert a raw 14-bit angle to degrees [0, 360)
         *
         * @param rawAngle The 14-bit raw angle value
         * @return Angle in degrees
         */
        [[nodiscard]] static AngleDegrees_t rawAngleToDegrees(RegisterData_t rawAngle);

        /**
         * Function that reads the DIAAGC register and returns diagnostics
         *
         * @return Diagnostics report or error when no stable data is available
         */
        etl::expected<AGC_DiagnosticsReport, AGC_DiagnosticsReadError> readAGC_Diagnostics() const;

        /**
         * Function that reads and clears (by IC design) the error flags from ERRFL register
         */
        void readAndClearErrorFlags() const;

        /**
         * Function that checks for ongoing sensor transaction
         * @return True if there is an ongoing sensor transaction
         */
        static bool sensorBusy();

private:
        /**
         * @var latestRegisterRequested
         *
         * The latest value read from a register
         */
        /**
         * @struct RegisterCache
         * @brief Local cache of most recently read register values
         */
        struct RegisterCache {
                /// Cached ANGLECOM raw value
                RegisterData_t angleComRaw = 0;
                /// Cached ANGLEUNC raw value
                RegisterData_t angleUncRaw = 0;
                /// Cached MAG raw value
                RegisterData_t magRaw = 0;
                /// Cached DIAAGC raw value
                RegisterData_t diagRaw = 0;
                /// Cached ERRFL raw value
                RegisterData_t errflRaw = 0;
                /// Cached AGC value (lower 8 bits of DIAAGC)
                std::uint8_t agcValue = 0;
        };

        /// Cached register values from the last successful read
        RegisterCache registerCache{};
        /// Last requested register address for the pending transaction
        RegisterAddress lastRequestedRegister = RegisterAddress::NOP;

        /**
         * @var isSensorBusy
         *
         * True if the sensor is busy with an ongoing transaction
         */
        inline static bool isSensorBusy = false;

        /**
         * @enum PendingTransfer
         * @brief Tracks the in-flight SPI transaction phase
         */
        enum class PendingTransfer : std::uint8_t {
                NONE,
                READ_COMMAND,
                READ_ERRFL,
                WRITE_COMMAND,
                WRITE_DATA,
                WRITE_ERRFL,
        };

        /// Current pending transfer type/state
        PendingTransfer pendingTransfer = PendingTransfer::NONE;
        /// Pending write data frame (data stage of a write)
        std::array<std::uint8_t, 2> pendingWriteFrame{};

        /**
         * @enum TransactionState
         * @brief State machine for read/write operations
         */
        enum class TransactionState : std::uint8_t {
                IDLE,
                IN_PROGRESS,
                SUCCESS,
                ERROR,
        };

        /// Last read transaction state
        TransactionState lastReadState = TransactionState::IDLE;
        /// Last write transaction state
        TransactionState lastWriteState = TransactionState::IDLE;
        /// Last read transaction error code
        ReadError lastReadError = ReadError::NOT_READY;
        /// Last write transaction error code
        WriteError lastWriteError = WriteError::NOT_READY;
        /// Last ERRFL status flags
        ERRFL_Status errflStatus{};

        /**
         * @enum ERRFL_RegisterMask
         * @brief Represents the bitmask definitions for the ERRFL (Error Flag) register
         *
         * The ERRFL register provides diagnostic error flags that indicate specific sensor errors
         * Each bit in this enumeration corresponds to a particular type of error detected by the sensor
         */
        enum class ERRFL_RegisterMask : RegisterData_t {
                PARITY_ERROR = 0b0000'0000'0000'0100,
                INVALID_COMMAND = 0b0000'0000'0000'0010,
                FRAMING_ERROR = 0b0000'0000'0000'0001,
        };

        /**
         * @enum ReadWriteCommandMask
         * @brief Represents the command masks for read and write operations
         *
         * The mask is applied to the command frame to encode the type of transaction
         */
        enum class ReadWriteCommandMask : uint16_t {
                WRITE = 0x0000,
                READ = 0x4000,
        };

        /**
         * @enum ParityBit
         * @brief Represents the parity bit options for the SPI communication with the AS5047P sensor
         *
         * The AS5047P uses a single parity bit in its SPI protocol for error detection in communication
         */
        enum class ParityBit : uint16_t {
                PARITY_BIT_0 = 0b0000'0000'0000'0000,
                PARITY_BIT_1 = 0b1000'0000'0000'0000,
        };

        /**
         * @enum DIAAGC_RegisterMask
         * @brief Represents the bitmask definitions for the DIAAGC register
         *
         * The DIAAGC register provides diagnostic and automatic gain control (AGC) information
         * This enumeration defines specific flags and fields within the register that
         * indicate diagnostics or configuration status of the AS5047P sensor
         */
        enum class DIAAGC_RegisterMask : RegisterData_t {
                MAG_FIELD_TOO_LOW = 0b0000'1000'0000'0000,
                MAG_FIELD_TOO_HIGH = 0b0000'0100'0000'0000,
                CORDIC_OVF = 0b0000'0010'0000'0000,
                OFFSET_COMP = 0b0000'0001'0000'0000,
                AGC_VALUE = 0b0000'0000'1111'1111,
        };

        /// Mask for 14-bit register data values
        static constexpr RegisterData_t RegisterDataMask = 0b0011'1111'1111'1111;
        /// Mask for MSBs of read data (upper 6 bits of first byte)
        static constexpr uint8_t ReadDataMsbMask = 0b0011'1111;
        /// Mask for the error flag bit in read responses
        static constexpr uint8_t ReadErrorFlagMask = 0b0100'0000;

        /**
         * The SPI command frame size in bytes
         */
        static constexpr size_t CommandFrameSize = 2;

        /**
         * The SPI data frame size in bytes
         */
        static constexpr size_t DataFrameSize = 2;

        /**
         * AS5047P 14-bit angular resolution: number of discrete angle steps per 360° revolution
         * Range: 0 to 16383 (0x0000 to 0x3FFF), providing ~0.022° per step
         */
        static constexpr uint16_t AngleResolutionSPI = 16384;

        static_assert(AngleResolutionSPI <= std::numeric_limits<uint16_t>::max(), "AngleResolutionSPI must fit in 'int'");

        /**
         * The full rotation angle in degrees
         */
        static constexpr float FullRotationDegrees = 360.0f;

        /**
         * Helper function that adds an even parity bit to a 16-bit SPI frame
         *
         * @param frame The 16-bit SPI frame without parity
         * @return The frame with the parity bit set
         */
        [[nodiscard]] static constexpr uint16_t setEvenParity(uint16_t frame) {
                if (std::popcount(frame) % 2 == 0)
                        return frame | static_cast<uint16_t>(ParityBit::PARITY_BIT_0);

                return frame | static_cast<uint16_t>(ParityBit::PARITY_BIT_1);
        }

        /**
         * Callback function to be called after the completion of an SPI read transaction
         *
         * @param context
         */
        static void spiReadCallback(void* context);

        /**
         * Callback function to be called after the completion of an SPI write transaction
         *
         * @param context
         */
        static void spiWriteCallback(void* context);

        /**
         * Function that performs the SPI Read operation between MCU and AS5047P magnetic encoder
         *
         * @param registerAddress The address of the device register
         * @return Expected success or error code
         */
        [[nodiscard]] etl::expected<void, ReadError> readDeviceRegister(RegisterAddress registerAddress);

        /**
         * Function that performs the SPI Write operation between MCU and AS5047P magnetic encoder
         *
         * @param registerAddress The address of the device register
         * @param data The data that will be written into the specified device register
         */
        etl::expected<void, WriteError> writeDeviceRegister(RegisterAddress registerAddress, RegisterData_t data);

        /**
         * Operator | overload for the construction of the command
         *
         * @param commandMask Read/write command mask
         * @param registerAddress Register address to encode
         * @return Composed SPI command frame
         */
        friend constexpr uint16_t operator|(ReadWriteCommandMask commandMask, RegisterAddress registerAddress) {
                return static_cast<uint16_t>(commandMask) | static_cast<uint16_t>(registerAddress);
        };
};
