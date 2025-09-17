#include "as5047p.hpp"

AS5047P::RegisterData_t AS5047P::readDeviceRegister(RegisterAddress address) {
    const uint16_t CommandFrame = 0b1111'1111'1111'1100;
    const auto CommandFrameMSB = static_cast<uint8_t>(CommandFrame >> 8);
    const auto CommandFrameLSB = static_cast<uint8_t>(CommandFrame & 0b1111'1111);

    std::array<uint8_t, 2> txBuffer {CommandFrameMSB, CommandFrameLSB};
    std::array<uint8_t, 2> rxBuffer {0, 0};

    AS5047P_CS_Clear();

    if(SERCOM1_SPI_Write(&txBuffer[0], 2))
        Logger_Info("SPI sent data\r\n");
    else
        Logger_Error("SPI failed to send data\r\n");

    AS5047P_CS_Set();
    SYSTICK_DelayMs(1);
    AS5047P_CS_Clear();

    if(SERCOM1_SPI_Read(&rxBuffer[0], 2))
        Logger_Info("SPI returned data\r\n");
    else
        Logger_Error("SPI failed to return data\r\n");

    AS5047P_CS_Set();

    return (static_cast<RegisterData_t>(rxBuffer[0]) << 8) | rxBuffer[1];
}
