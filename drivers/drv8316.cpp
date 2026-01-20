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
 * @file   drv8316.cpp
 * @brief  Device driver for the DRV8316 Texas Instruments BLDC driver chip
 * @author Georgios Gkogkou <ggkogkou125@gmail.com>
 */

#include "drv8316.hpp"

void DRV8316::setPWMMode(PWM_Mode pwmMode) {
        constexpr auto PWM_ModeBitsMask = static_cast<RegisterMask_t>(Control_Register_2_Mask::PWM_MODE);
        constexpr uint8_t PWM_ModeFieldClearMask = ~static_cast<uint8_t>(PWM_ModeBitsMask);

        if (not readRegister(RegisterAddress::Control_Register_2))
                return;

        while (not deviceIsReady()) { }

        const uint8_t RegisterData = latestRegisterValueRead & PWM_ModeFieldClearMask;

        const uint8_t DataToWrite = RegisterData | (static_cast<uint8_t>(pwmMode) << 1);
        writeRegister(RegisterAddress::Control_Register_2, DataToWrite);

        while (not deviceIsReady()) { }
}

void DRV8316::setCurrentSenseAmplifierGain(CurrentSenseGain gain) {
        constexpr auto CurrentSenseGainBitsMask = static_cast<RegisterMask_t>(Control_Register_5_Mask::CSA_GAIN);
        constexpr uint8_t CurrentSenseGainFieldClearMask = ~static_cast<uint8_t>(CurrentSenseGainBitsMask);

        if (not readRegister(RegisterAddress::Control_Register_5))
                return;

        while (not deviceIsReady()) { }

        const uint8_t RegisterData = latestRegisterValueRead & CurrentSenseGainFieldClearMask;

        const uint8_t DataToWrite = RegisterData | static_cast<uint8_t>(gain);
        writeRegister(RegisterAddress::Control_Register_5, DataToWrite);

        while (not deviceIsReady()) { }

        if (gain == CurrentSenseGain::CSA_GAIN_0_15)
                csaGain = 0.15f;
        else if (gain == CurrentSenseGain::CSA_GAIN_0_30)
                csaGain = 0.30f;
        else if (gain == CurrentSenseGain::CSA_GAIN_0_6)
                csaGain = 0.6f;
        else if (gain == CurrentSenseGain::CSA_GAIN_1_2)
                csaGain = 1.2f;
}

void DRV8316::lockAllRegisters() {
        constexpr uint8_t RegistersLockBitsMask = static_cast<RegisterMask_t>(Control_Register_1_Mask::REG_LOCK);
        constexpr uint8_t RegistersLockFieldClearMask = ~static_cast<uint8_t>(RegistersLockBitsMask);

        const uint8_t RegisterData = readRegister(RegisterAddress::Control_Register_1) & RegistersLockFieldClearMask;

        const uint8_t DataToWrite = RegisterData | static_cast<uint8_t>(0b110);
        writeRegister(RegisterAddress::Control_Register_1, DataToWrite);
}

void DRV8316::unlockAllRegisters() {
        constexpr uint8_t RegistersLockBitsMask = static_cast<RegisterMask_t>(Control_Register_1_Mask::REG_LOCK);
        constexpr uint8_t RegistersLockFieldClearMask = ~static_cast<uint8_t>(RegistersLockBitsMask);

        if (not readRegister(RegisterAddress::Control_Register_1)) {
                return;
        }

        while (not deviceIsReady()) { }

        const uint8_t RegisterData = latestRegisterValueRead & RegistersLockFieldClearMask;

        const uint8_t DataToWrite = RegisterData | static_cast<uint8_t>(0b011);
        writeRegister(RegisterAddress::Control_Register_1, DataToWrite);

        while (not deviceIsReady()) { }
}

bool DRV8316::checkForFaults() {
        const auto icStatus = readRegister(RegisterAddress::IC_Status_Register);
        const auto status1 = readRegister(RegisterAddress::Status_Register_1);
        const auto status2 = readRegister(RegisterAddress::Status_Register_2);
        const auto ctrl1 = readRegister(RegisterAddress::Control_Register_1);
        const auto ctrl2 = readRegister(RegisterAddress::Control_Register_2);
        const auto ctrl5 = readRegister(RegisterAddress::Control_Register_5);

        if (icStatus != 0)
                return true;
        if (status1 != 0)
                return true;
        if (status2 != 0)
                return true;

        return false;
}

void DRV8316::calculateCurrents(float& IA, float& IB, float& IC) {
        const float IB_Sensed = (IB - offsetCorrectionVoltageB) / csaGain;
        const float IC_Sensed = (IC - offsetCorrectionVoltageC) / csaGain;

        constexpr float IBB = 0.971197f;
        constexpr float IBC = 0.068300f;
        constexpr float ICB = 0.020876f;
        constexpr float ICC = 0.994823f;

        IB = IBB * IB_Sensed - IBC * IC_Sensed;
        IC = ICB * IB_Sensed + ICC * IC_Sensed;
        IA = -(IB + IC);
}

void DRV8316::spiReadTransferCallback(void* context) {
        auto* self = static_cast<DRV8316*>(context);

        self->latestRegisterValueRead = self->spiRequest.rxBuffer[1];
        self->isReady = true;
}

void DRV8316::spiWriteTransferCallback(void* context) {
        auto* self = static_cast<DRV8316*>(context);
        self->isReady = true;
}

bool DRV8316::writeRegister(RegisterAddress registerAddress, uint8_t dataToWrite) {
        constexpr uint8_t WriteOperationBit = 0x0;

        const auto WriteOperationMSB = [&]() -> uint8_t {
                uint8_t cmd = WriteOperationBit | (static_cast<uint8_t>(registerAddress) << 1);

                const auto NumberOf1sMSB = std::popcount(cmd);
                const auto NumberOf1sLSB = std::popcount(dataToWrite);

                if ((NumberOf1sLSB + NumberOf1sMSB) % 2 == 1)
                        cmd |= 0b0000'0001;

                return cmd;
        }();

        const auto CommandFrame = std::array{WriteOperationMSB, dataToWrite};

        spiRequest.txBuffer = CommandFrame;
        spiRequest.chipSelectPin = DRV8316_CS_PIN;
        spiRequest.callback = &DRV8316::spiWriteTransferCallback;
        spiRequest.context = this;

        SYSTICK_DelayUs(1); /// At least 400ns between transactions

        isReady = false;

        if (SPI_Buffer::submit(spiRequest) != SPI_Buffer::TransactionState::PLACED) {
                isReady = true;
                SYSTICK_DelayUs(1); /// At least 400ns between transactions
                return false;
        }

        SYSTICK_DelayUs(1); /// At least 400ns between transactions

        return true;
}

bool DRV8316::readRegister(RegisterAddress registerAddress) {
        constexpr uint8_t ReadOperationLSB = 0x0;
        constexpr uint8_t ReadOperationBit = 0b1000'0000;

        const auto ReadOperationMSB = [&]() -> uint8_t {
                uint8_t cmd = ReadOperationBit | (static_cast<uint8_t>(registerAddress) << 1);

                if (std::popcount(cmd) % 2 == 1)
                        cmd |= 0b0000'0001;

                return cmd;
        }();

        const auto CommandFrame = std::array{ReadOperationMSB, ReadOperationLSB};

        spiRequest.txBuffer = CommandFrame;
        spiRequest.chipSelectPin = DRV8316_CS_PIN;
        spiRequest.callback = &DRV8316::spiReadTransferCallback;
        spiRequest.context = this;

        SYSTICK_DelayUs(1); /// At least 400ns between transactions

        isReady = false;

        if (SPI_Buffer::submit(spiRequest) != SPI_Buffer::TransactionState::PLACED) {
                isReady = true;
                SYSTICK_DelayUs(1); /// At least 400ns between transactions
                return false;
        }

        SYSTICK_DelayUs(1); /// At least 400ns between transactions

        return true;
}
