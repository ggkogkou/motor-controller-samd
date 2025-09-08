#include "as5047p.hpp"

std::uint16_t AS5047P::read_angle() {
    Logger_Info("Suppose reading an angle\r\n");

    return 2;
}

void AS5047P::selectOutputMode(OutputMode output_mode) const {


    return;
}

uint8_t* AS5047P::angles() {
    return nullptr;
}

uint16_t AS5047P::readFromRegister() {
    uint16_t commandFrame = 0b1111'1111'1111'1100;

    uint8_t commandFrame1 = 0b1111'1111;
    uint8_t commandFrame2 = 0b1111'1100;
    uint8_t txBuffer[2] = {commandFrame1, commandFrame2};
    size_t txSize = 2;
    uint8_t rxBuffer[2] = {0, 0};
    size_t rxSize = 2;


    // if()
    // {
    //     Logger_Info("SPI transaction passed\r\n");
    // }
    // else
    // {
    //     Logger_Error("SPI transaction failed\r\n");
    // }

    AS5047P_CS_Clear();
    SERCOM1_SPI_Write(&txBuffer[0], txSize);

    SERCOM1_SPI_Read(&rxBuffer[0], rxSize);
    // SERCOM1_SPI_WriteRead(&txBuffer, txSize, &rxBuffer, rxSize);

    AS5047P_CS_Set();

    while (SERCOM1_SPI_IsTransmitterBusy()) {};

    SYSTICK_DelayMs(1000);

    return 0;
}
