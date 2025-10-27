#include "drv8316.hpp"

void DRV8316::setPWMMode(PWM_Mode pwmMode)  {
    constexpr auto PWM_ModeBitsMask = static_cast<RegisterMask_t>(Control_Register_2_Mask::PWM_MODE);
    constexpr auto PWM_ModeFieldClearMask = ~ static_cast<uint8_t>(PWM_ModeBitsMask);

    const auto RegisterData = readRegister(RegisterAddress::Control_Register_2) & PWM_ModeFieldClearMask;

    const uint8_t DataToWrite = RegisterData | (static_cast<uint8_t>(pwmMode) << 1);
    writeRegister(RegisterAddress::Control_Register_2, DataToWrite);
}

void DRV8316::setCurrentSenseAmplifierGain(CurrentSenseGain gain)  {
    constexpr auto CurrentSenseGainBitsMask = static_cast<RegisterMask_t>(Control_Register_5_Mask::CSA_GAIN);
    constexpr auto CurrentSenseGainFieldClearMask = static_cast<uint8_t>(~CurrentSenseGainBitsMask);

    const auto RegisterData = readRegister(RegisterAddress::Control_Register_5) & CurrentSenseGainFieldClearMask;

    const uint8_t DataToWrite = RegisterData | static_cast<uint8_t>(gain);
    writeRegister(RegisterAddress::Control_Register_5, DataToWrite);
}

bool DRV8316::checkForFaults()  {
    // const auto icStatus = readRegister(RegisterAddress::IC_Status_Register);
    // const auto status1 = readRegister(RegisterAddress::Status_Register_1);
    // const auto status2 = readRegister(RegisterAddress::Status_Register_2);
    const auto ctrl2 = readRegister(RegisterAddress::Control_Register_2);

    // if (icStatus != 0)
        // return true;
    // if (status1 != 0)
        // return true;
    // if (status2 != 0)
        // return true;

    return false;
}

void DRV8316::writeRegister(RegisterAddress registerAddress, uint8_t dataToWrite) {
    constexpr uint8_t WriteOperationBit = 0x0;

    const auto WriteOperationMSB = [&]() -> uint8_t {
        uint8_t cmd = WriteOperationBit | static_cast<uint8_t>(registerAddress);

        if (std::popcount(cmd) % 2 == 1)
            cmd |= 0b0000'0001;

        return cmd;
    }();

    auto CommandFrame = std::array{WriteOperationMSB, dataToWrite};

    DRV8316_CS_Clear();
    SERCOM4_SPI_Write(&CommandFrame[0], CommandFrame.size());
    DRV8316_CS_Set();

    SYSTICK_DelayUs(1); /// At least 400ns between transactions
}

uint8_t DRV8316::readRegister(RegisterAddress registerAddress) {
    constexpr uint8_t ReadOperationLSB = 0x0;
    constexpr uint8_t ReadOperationBit = 0b1000'0000;

    const auto ReadOperationMSB = [&]() -> uint8_t {
        uint8_t cmd = ReadOperationBit | (static_cast<uint8_t>(registerAddress) << 1);

        if (std::popcount(cmd) % 2 == 1)
            cmd |= 0b0000'0001;

        return cmd;
    }();

    auto CommandFrame = std::array{ReadOperationMSB, ReadOperationLSB};
    std::array<uint8_t, 2> rxBuffer {0};

    DRV8316_CS_Clear();
    SERCOM4_SPI_WriteRead(&CommandFrame[0], CommandFrame.size(), &rxBuffer[0], rxBuffer.size());
    DRV8316_CS_Set();

    SYSTICK_DelayUs(1); /// At least 400ns between transactions

    return rxBuffer[1];
}
