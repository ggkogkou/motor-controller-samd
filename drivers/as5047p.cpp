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
 * @file   as5047p.cpp
 * @brief  Device driver for the AS5047P magnetic encoder (implements only SPI mode)
 * @author Georgios Gkogkou <ggkogkou125@gmail.com>
 */

#include "as5047p.hpp"

AS5047P::AS5047P() {
        spiRequest.chipSelectPin = ENCODER_CS_PIN;
        isSensorBusy = false;
}

etl::expected<void, AS5047P::ReadError> AS5047P::readDeviceRegister(RegisterAddress registerAddress) {
        if (isSensorBusy) {
                lastReadState = TransactionState::ERROR;
                lastReadError = ReadError::BUSY;
                return etl::unexpected(ReadError::BUSY);
        }

        const auto CommandFrame = setEvenParity(ReadWriteCommandMask::READ | registerAddress);

        pendingTransfer = PendingTransfer::READ_COMMAND;
        lastRequestedRegister = registerAddress;
        lastReadState = TransactionState::IN_PROGRESS;

        spiRequest.txBuffer = {static_cast<uint8_t>(CommandFrame >> 8), static_cast<uint8_t>(CommandFrame & 0b1111'1111)};
        spiRequest.chipSelectPin = ENCODER_CS_PIN;
        spiRequest.callback = &AS5047P::spiReadCallback;
        spiRequest.context = this;

        isSensorBusy = true;

        if (SPI_Buffer::submit(spiRequest) != SPI_Buffer::TransactionState::PLACED) {
                pendingTransfer = PendingTransfer::NONE;
                isSensorBusy = false;
                lastReadState = TransactionState::ERROR;
                lastReadError = ReadError::SUBMIT_FAILED;
                return etl::unexpected(ReadError::SUBMIT_FAILED);
        }

        return {};
}

etl::expected<void, AS5047P::WriteError> AS5047P::writeDeviceRegister(RegisterAddress registerAddress, RegisterData_t data) {
        if (isSensorBusy) {
                lastWriteState = TransactionState::ERROR;
                lastWriteError = WriteError::BUSY;
                return etl::unexpected(WriteError::BUSY);
        }

        const auto CommandFrame = setEvenParity(ReadWriteCommandMask::WRITE | registerAddress);
        const auto DataFrame = setEvenParity(static_cast<uint16_t>(data & RegisterDataMask));

        pendingWriteFrame[0] = static_cast<uint8_t>(DataFrame >> 8);
        pendingWriteFrame[1] = static_cast<uint8_t>(DataFrame & 0b1111'1111);

        pendingTransfer = PendingTransfer::WRITE_COMMAND;
        lastWriteState = TransactionState::IN_PROGRESS;

        spiRequest.txBuffer = {static_cast<uint8_t>(CommandFrame >> 8), static_cast<uint8_t>(CommandFrame & 0b1111'1111)};
        spiRequest.chipSelectPin = ENCODER_CS_PIN;
        spiRequest.callback = &AS5047P::spiWriteCallback;
        spiRequest.context = this;

        isSensorBusy = true;

        if (SPI_Buffer::submit(spiRequest) != SPI_Buffer::TransactionState::PLACED) {
                pendingTransfer = PendingTransfer::NONE;
                isSensorBusy = false;
                lastWriteState = TransactionState::ERROR;
                lastWriteError = WriteError::SUBMIT_FAILED;
                return etl::unexpected(WriteError::SUBMIT_FAILED);
        }

        return {};
}

etl::expected<void, AS5047P::ReadError> AS5047P::request(RegisterAddress registerAddress) {
        return readDeviceRegister(registerAddress);
}

etl::expected<void, AS5047P::WriteError> AS5047P::writeRegister(RegisterAddress registerAddress, RegisterData_t data) {
        return writeDeviceRegister(registerAddress, data);
}

etl::expected<void, AS5047P::ReadError> AS5047P::lastReadResult() const {
        if (lastReadState == TransactionState::SUCCESS)
                return {};

        if (lastReadState == TransactionState::IN_PROGRESS)
                return etl::unexpected(ReadError::NOT_READY);

        return etl::unexpected(lastReadError);
}

etl::expected<void, AS5047P::WriteError> AS5047P::lastWriteResult() const {
        if (lastWriteState == TransactionState::SUCCESS)
                return {};

        if (lastWriteState == TransactionState::IN_PROGRESS)
                return etl::unexpected(WriteError::NOT_READY);

        return etl::unexpected(lastWriteError);
}

AS5047P::ERRFL_Status AS5047P::lastErrflStatus() const {
        return errflStatus;
}

void AS5047P::spiReadCallback(void* context) {
        auto* self = static_cast<AS5047P*>(context);

        const auto& RxBuffer = self->spiRequest.rxBuffer;
        const auto RxWord =
                static_cast<std::uint16_t>(static_cast<std::uint16_t>(RxBuffer[0]) << 8 | static_cast<std::uint16_t>(RxBuffer[1]));

        const auto failRead = [&](ReadError error) {
                self->pendingTransfer = PendingTransfer::NONE;
                self->lastReadState = TransactionState::ERROR;
                self->lastReadError = error;
                self->isSensorBusy = false;
        };

        if (self->pendingTransfer != PendingTransfer::READ_COMMAND && self->pendingTransfer != PendingTransfer::READ_ERRFL) {
                failRead(ReadError::UNEXPECTED_STATE);
                return;
        }

        const auto ParityCount = std::popcount(RxWord);

        if (ParityCount % 2 == 1) {
                self->lastRequestedRegister = RegisterAddress::NOP;
                failRead(ReadError::PARITY_ERROR);
                return;
        }

        if ((RxBuffer[0] & ReadErrorFlagMask) == ReadErrorFlagMask && self->pendingTransfer == PendingTransfer::READ_COMMAND) {
                constexpr auto ERRFL_Command = setEvenParity(ReadWriteCommandMask::READ | RegisterAddress::ERRFL);

                self->spiRequest.txBuffer = {static_cast<uint8_t>(ERRFL_Command >> 8),
                                             static_cast<uint8_t>(ERRFL_Command & 0b1111'1111)};
                self->spiRequest.chipSelectPin = ENCODER_CS_PIN;
                self->spiRequest.callback = &AS5047P::spiReadCallback;
                self->spiRequest.context = self;

                self->pendingTransfer = PendingTransfer::READ_ERRFL;
                self->lastRequestedRegister = RegisterAddress::ERRFL;
                self->lastReadState = TransactionState::ERROR;
                self->lastReadError = ReadError::ERROR_FLAG_SET;

                if (SPI_Buffer::submit(self->spiRequest) != SPI_Buffer::TransactionState::PLACED) {
                        failRead(ReadError::SUBMIT_FAILED);
                }

                return;
        }

        const auto Data = static_cast<RegisterData_t>(static_cast<RegisterData_t>(RxBuffer[0] & ReadDataMsbMask) << 8 |
                                                      static_cast<RegisterData_t>(RxBuffer[1]));

        if (self->pendingTransfer == PendingTransfer::READ_ERRFL) {
                self->registerCache.errflRaw = Data;
                self->errflStatus.raw = Data;
                self->errflStatus.parityError = (Data & static_cast<RegisterData_t>(ERRFL_RegisterMask::PARITY_ERROR)) != 0;
                self->errflStatus.invalidCommand = (Data & static_cast<RegisterData_t>(ERRFL_RegisterMask::INVALID_COMMAND)) != 0;
                self->errflStatus.framingError = (Data & static_cast<RegisterData_t>(ERRFL_RegisterMask::FRAMING_ERROR)) != 0;
        } else {
                switch (self->lastRequestedRegister) {
                case RegisterAddress::ANGLECOM:
                        self->registerCache.angleComRaw = Data;
                        break;
                case RegisterAddress::ANGLEUNC:
                        self->registerCache.angleUncRaw = Data;
                        break;
                case RegisterAddress::MAG:
                        self->registerCache.magRaw = Data;
                        break;
                case RegisterAddress::DIAAGC:
                        self->registerCache.diagRaw = Data;
                        self->registerCache.agcValue = static_cast<std::uint8_t>(Data & RegisterDataMask);
                        break;
                case RegisterAddress::ERRFL:
                        self->registerCache.errflRaw = Data;
                        self->errflStatus.raw = Data;
                        self->errflStatus.parityError = (Data & static_cast<RegisterData_t>(ERRFL_RegisterMask::PARITY_ERROR)) != 0;
                        self->errflStatus.invalidCommand =
                                (Data & static_cast<RegisterData_t>(ERRFL_RegisterMask::INVALID_COMMAND)) != 0;
                        self->errflStatus.framingError = (Data & static_cast<RegisterData_t>(ERRFL_RegisterMask::FRAMING_ERROR)) != 0;
                        break;
                default:
                        break;
                }

                self->lastReadState = TransactionState::SUCCESS;
        }

        self->pendingTransfer = PendingTransfer::NONE;
        self->isSensorBusy = false;
}

void AS5047P::spiWriteCallback(void* context) {
        auto* self = static_cast<AS5047P*>(context);

        const auto& RxBuffer = self->spiRequest.rxBuffer;
        const auto RxWord =
                static_cast<std::uint16_t>(static_cast<std::uint16_t>(RxBuffer[0]) << 8 | static_cast<std::uint16_t>(RxBuffer[1]));

        const auto failWrite = [&](WriteError error) {
                self->pendingTransfer = PendingTransfer::NONE;
                self->lastWriteState = TransactionState::ERROR;
                self->lastWriteError = error;
                self->isSensorBusy = false;
        };

        if (self->pendingTransfer == PendingTransfer::WRITE_COMMAND) {
                const auto ParityCount = std::popcount(RxWord);

                if (ParityCount % 2 == 1) {
                        failWrite(WriteError::PARITY_ERROR);
                        return;
                }

                if ((RxBuffer[0] & ReadErrorFlagMask) == ReadErrorFlagMask) {
                        constexpr auto ERRFL_Command = setEvenParity(ReadWriteCommandMask::READ | RegisterAddress::ERRFL);

                        self->spiRequest.txBuffer = {static_cast<uint8_t>(ERRFL_Command >> 8),
                                                     static_cast<uint8_t>(ERRFL_Command & 0b1111'1111)};
                        self->spiRequest.chipSelectPin = ENCODER_CS_PIN;
                        self->spiRequest.callback = &AS5047P::spiWriteCallback;
                        self->spiRequest.context = self;

                        self->pendingTransfer = PendingTransfer::WRITE_ERRFL;
                        self->lastWriteState = TransactionState::ERROR;
                        self->lastWriteError = WriteError::ERROR_FLAG_SET;

                        if (SPI_Buffer::submit(self->spiRequest) != SPI_Buffer::TransactionState::PLACED) {
                                failWrite(WriteError::SUBMIT_FAILED);
                        }

                        return;
                }

                self->pendingTransfer = PendingTransfer::WRITE_DATA;
                self->spiRequest.txBuffer = self->pendingWriteFrame;
                self->spiRequest.chipSelectPin = ENCODER_CS_PIN;
                self->spiRequest.callback = &AS5047P::spiWriteCallback;
                self->spiRequest.context = self;

                if (SPI_Buffer::submit(self->spiRequest) != SPI_Buffer::TransactionState::PLACED) {
                        failWrite(WriteError::SUBMIT_FAILED);
                }

                return;
        }

        if (self->pendingTransfer == PendingTransfer::WRITE_ERRFL) {
                const auto ParityCount = std::popcount(RxWord);

                if (ParityCount % 2 == 0) {
                        const auto Data = static_cast<RegisterData_t>(static_cast<RegisterData_t>(RxBuffer[0] & ReadDataMsbMask) << 8 |
                                                                      static_cast<RegisterData_t>(RxBuffer[1]));
                        self->registerCache.errflRaw = Data;
                        self->errflStatus.raw = Data;
                        self->errflStatus.parityError = (Data & static_cast<RegisterData_t>(ERRFL_RegisterMask::PARITY_ERROR)) != 0;
                        self->errflStatus.invalidCommand =
                                (Data & static_cast<RegisterData_t>(ERRFL_RegisterMask::INVALID_COMMAND)) != 0;
                        self->errflStatus.framingError = (Data & static_cast<RegisterData_t>(ERRFL_RegisterMask::FRAMING_ERROR)) != 0;
                }

                self->pendingTransfer = PendingTransfer::NONE;
                self->isSensorBusy = false;

                return;
        }

        if (self->pendingTransfer != PendingTransfer::WRITE_DATA) {
                failWrite(WriteError::UNEXPECTED_STATE);
                return;
        }

        self->pendingTransfer = PendingTransfer::NONE;
        self->lastWriteState = TransactionState::SUCCESS;
        self->isSensorBusy = false;
}

etl::expected<AS5047P::RegisterData_t, AS5047P::ReadError> AS5047P::measureAngleCompensatedRaw() const {
        const auto Status = lastReadResult();

        if (not Status.has_value())
                return etl::unexpected(Status.error());

        if (registerCache.angleComRaw > RegisterDataMask)
                return etl::unexpected(ReadError::OUT_OF_RANGE);

        return registerCache.angleComRaw;
}

etl::expected<AS5047P::RegisterData_t, AS5047P::ReadError> AS5047P::measureAngleUncompensatedRaw() const {
        const auto Status = lastReadResult();

        if (not Status.has_value())
                return etl::unexpected(Status.error());

        if (registerCache.angleUncRaw > RegisterDataMask)
                return etl::unexpected(ReadError::OUT_OF_RANGE);

        return registerCache.angleUncRaw;
}

etl::expected<AS5047P::FieldMagnitude_t, AS5047P::ReadError> AS5047P::measureFieldMagnitude() const {
        const auto Status = lastReadResult();

        if (not Status.has_value())
                return etl::unexpected(Status.error());

        if (registerCache.magRaw > RegisterDataMask)
                return etl::unexpected(ReadError::OUT_OF_RANGE);

        return registerCache.magRaw;
}

AS5047P::AngleDegrees_t AS5047P::rawAngleToDegrees(RegisterData_t rawAngle) {
        return static_cast<float>(rawAngle & RegisterDataMask) / static_cast<float>(AngleResolutionSPI) * FullRotationDegrees;
}

etl::expected<AS5047P::AGC_DiagnosticsReport, AS5047P::AGC_DiagnosticsReadError> AS5047P::readAGC_Diagnostics() const {
        if (isSensorBusy)
                return etl::unexpected(AGC_DiagnosticsReadError::SENSOR_BUSY);

        const auto DIAAGC_Data = registerCache.diagRaw;
        AGC_DiagnosticsReport report{};

        report.agcValue = registerCache.agcValue;

        const auto pushError = [&report](AGC_DiagnosticsError error) {
                if (report.errorCount < report.errors.size())
                        report.errors[report.errorCount++] = error;
        };

        if (DIAAGC_Data & static_cast<RegisterData_t>(DIAAGC_RegisterMask::MAG_FIELD_TOO_LOW))
                pushError(AGC_DiagnosticsError::MAGNETIC_FIELD_TOO_LOW);

        if (DIAAGC_Data & static_cast<RegisterData_t>(DIAAGC_RegisterMask::MAG_FIELD_TOO_HIGH))
                pushError(AGC_DiagnosticsError::MAGNETIC_FIELD_TOO_HIGH);

        if (DIAAGC_Data & static_cast<RegisterData_t>(DIAAGC_RegisterMask::CORDIC_OVF))
                pushError(AGC_DiagnosticsError::CORDIC_OVF);

        if (DIAAGC_Data & static_cast<RegisterData_t>(DIAAGC_RegisterMask::OFFSET_COMP))
                pushError(AGC_DiagnosticsError::OFFSET_COMP_NOT_READY);

        return report;
}

void AS5047P::readAndClearErrorFlags() const {
        const auto errflData = registerCache.errflRaw;
        (void)errflData;
}

bool AS5047P::sensorBusy() {
        return isSensorBusy;
}
