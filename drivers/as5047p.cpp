#include "as5047p.hpp"

AS5047P::AS5047P(AS5047P_Config config) {

}

AS5047P::RegisterData_t AS5047P::readDeviceRegister(RegisterAddress registerAddress) const {
    uint16_t commandFrame = ReadWriteCommandMask::READ | registerAddress;

    // Calculate parity bit -- ARM GCC built-in command for popcnt
    if (__builtin_popcount(commandFrame) % 2 == 0)
        commandFrame = commandFrame | static_cast<uint16_t>(ParityBit::PARITY_BIT_0);
    else
        commandFrame = commandFrame | static_cast<uint16_t>(ParityBit::PARITY_BIT_1);

    const auto CommandFrameMSB = static_cast<uint8_t>(commandFrame >> 8);
    const auto CommandFrameLSB = static_cast<uint8_t>(commandFrame & 0b1111'1111);

    std::array<uint8_t, CommandFrameSize> txBuffer {CommandFrameMSB, CommandFrameLSB};
    std::array<uint8_t, DataFrameSize> rxBuffer {0, 0};

    AS5047_CS_Clear();

    if(SERCOM1_SPI_Write(&txBuffer[0], 2))
        Logger_Info("SPI sent data\r\n");
    else
        Logger_Error("SPI failed to send data\r\n");

    AS5047_CS_Set();
    SYSTICK_DelayMs(1);
    AS5047_CS_Clear();

    if(SERCOM1_SPI_Read(&rxBuffer[0], 2))
        Logger_Info("SPI returned data\r\n");
    else
        Logger_Error("SPI failed to return data\r\n");

    AS5047_CS_Set();

    const decltype(rxBuffer)::value_type PARD_Bit = rxBuffer[0] & 0b0111'1111;
    const decltype(rxBuffer)::value_type EF_Bit = rxBuffer[0] & 0b1011'1111;

    if (( __builtin_popcount(rxBuffer[1]) + __builtin_popcount(rxBuffer[0]) ) % 2 == 1 && PARD_Bit == 0)
        Logger_Error("Parity Bit Error, PARD set incorrectly");

    if (EF_Bit == 1)
        Logger_Error("Command Frame Error Occured, EF Bit = 1");

    return (static_cast<RegisterData_t>(rxBuffer[0] & 0b0011'1111) << 8) | rxBuffer[1];
}

void AS5047P::writeDeviceRegister(RegisterAddress registerAddress, RegisterData_t data) const {
    const uint16_t CommandFrame = 0b1011'1111'1111'1101;
    const auto CommandFrameMSB = static_cast<uint8_t>(CommandFrame >> 8);
    const auto CommandFrameLSB = static_cast<uint8_t>(CommandFrame & 0b1111'1111);

    const auto DataFrameMSB = static_cast<uint8_t>(data >> 8);
    const auto DataFrameLSB = static_cast<uint8_t>(data & 0b1111'1111);

    std::array<uint8_t, CommandFrameSize> addressBuffer {CommandFrameMSB, CommandFrameLSB};
    std::array<uint8_t, DataFrameSize> dataBuffer {DataFrameMSB, DataFrameLSB};

    AS5047_CS_Clear();

    if(SERCOM1_SPI_Write(&addressBuffer[0], 2))
        Logger_Info("SPI sent data\r\n");
    else
        Logger_Error("SPI failed to send data\r\n");

    AS5047_CS_Set();
    SYSTICK_DelayMs(1);
    AS5047_CS_Clear();

    if(SERCOM1_SPI_Write(&dataBuffer[0], 2))
        Logger_Info("SPI sent data\r\n");
    else
        Logger_Error("SPI failed to send data\r\n");

}

AS5047P::Angle_t AS5047P::measureAngleUncompensated() const {
    const auto AngleUncData = readDeviceRegister(RegisterAddress::ANGLEUNC);

    return static_cast<Angle_t>(AngleUncData) / static_cast<Angle_t>(AngleResolutionSPI) * FullRotationDegrees;
}

AS5047P::Angle_t AS5047P::measureAngleCompensated() const {
    const auto AngleComData = readDeviceRegister(RegisterAddress::ANGLECOM);

    return static_cast<Angle_t>(AngleComData) / static_cast<Angle_t>(AngleResolutionSPI) * FullRotationDegrees;
}

AS5047P::FieldMagnitude_t AS5047P::measureFieldMagnitude() const {
    return readDeviceRegister(RegisterAddress::MAG);
}

void AS5047P::readAGC_Diagnostics() const {
    const auto DIAAGC_Data = readDeviceRegister(RegisterAddress::DIAAGC);

    if (DIAAGC_Data & static_cast<RegisterData_t>(DIAAGC_RegisterMask::MAG_FIELD_TOO_LOW))
        Logger_Info("AGC Diagnostics: The magnetic field is too low\r\n");

    if (DIAAGC_Data & static_cast<RegisterData_t>(DIAAGC_RegisterMask::MAG_FIELD_TOO_HIGH))
        Logger_Info("AGC Diagnostics: The magnetic field is too high\r\n");

    if (DIAAGC_Data & static_cast<RegisterData_t>(DIAAGC_RegisterMask::CORDIC_OVF))
        Logger_Info("AGC Diagnostics: CORDIC overflow\r\n");

    if (DIAAGC_Data & static_cast<RegisterData_t>(DIAAGC_RegisterMask::OFFSET_COMP))
        Logger_Info("AGC Diagnostics: Offset compensation\r\n");
}

void AS5047P::readAndClearErrorFlags() const {
    const auto ERRFL_Data = readDeviceRegister(RegisterAddress::ERRFL);

    if (ERRFL_Data & static_cast<RegisterData_t>(ERRFL_RegisterMask::PARITY_ERROR))
        Logger_Info("Error flag: Parity error\r\n");

    if (ERRFL_Data & static_cast<RegisterData_t>(ERRFL_RegisterMask::INVALID_COMMAND))
        Logger_Info("Error flag: Invalid command\r\n");

    if (ERRFL_Data & static_cast<RegisterData_t>(ERRFL_RegisterMask::FRAMING_ERROR))
        Logger_Info("Error flag: Framing error\r\n");

}
