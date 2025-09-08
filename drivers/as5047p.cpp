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




    // if()
    // {
    //     Logger_Info("SPI transaction passed\r\n");
    // }
    // else
    // {
    //     Logger_Error("SPI transaction failed\r\n");
    // }

    AS5047P_CS_Clear();
    // SERCOM1_SPI_Write(&txBuffer[0], txSize);

    nowWrite = true;
    if(SERCOM1_SPI_Write(&txBuffer, txSize))
    {
        Logger_Info("SPI sent data\r\n");
        nowWrite = false;
    }
    else
    {
        Logger_Error("SPI failed to send data\r\n");
    }

    // SERCOM1_SPI_Read(&rxBuffer[0], rxSize);
    // SERCOM1_SPI_WriteRead(&txBuffer, txSize, &rxBuffer, rxSize);

    nowRead = true;
    if(SERCOM1_SPI_Read(&rxBuffer, rxSize))
    {
        Logger_Info("SPI returned data\r\n");
        nowRead = false;
    }
    else
    {
        Logger_Error("SPI failed to return data\r\n");
    }

    AS5047P_CS_Set();

    // while (SERCOM1_SPI_IsTransmitterBusy()) {};

    SYSTICK_DelayMs(500);

    return 0;
}
