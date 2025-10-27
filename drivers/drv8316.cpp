#include "drv8316.hpp"

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
        uint8_t cmd = ReadOperationBit | static_cast<uint8_t>(registerAddress);

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

void DRV8316::setPWMMode(PWM_Mode pwmMode) {
    constexpr auto PWM_ModeBitsMask = static_cast<RegisterMask_t>(Control_Register_2_Mask::PWM_MODE);
    const auto RegisterData = readRegister(RegisterAddress::Control_Register_2);

    const uint8_t DataToWrite = RegisterData | (static_cast<uint8_t>(pwmMode) << 1);
    writeRegister(RegisterAddress::Control_Register_2, DataToWrite);
}
