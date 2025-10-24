#include "drv8316.hpp"

void DRV8316::writeRegister(RegisterAddress address, uint8_t value) {

}

uint8_t DRV8316::readRegister(RegisterAddress address) {
    DRV8316Word_t cmdFrame = 0b1000'1000'0000'0000;

    std::array<uint8_t, 2> txBuffer {0b1000'1000, 0b0000'0000};
    std::array<uint8_t, 2> rxBuffer {0};

    AS5047_CS_Set();
    DRV8316_CS_Clear();

    // if(SERCOM4_SPI_Write(&txBuffer[0], txBuffer.size()))
    //     Logger_Info("SPI sent data\r\n");
    // else
    //     Logger_Error("SPI failed to send data\r\n");
    //
    // if(SERCOM4_SPI_Read(&rxBuffer[0], rxBuffer.size()))
    //     Logger_Info("SPI returned data\r\n");
    // else
    //     Logger_Error("SPI failed to return data\r\n");

    SERCOM4_SPI_WriteRead(&txBuffer[0], 2, &rxBuffer[0], 2);

    SYSTICK_DelayMs(1);
    DRV8316_CS_Set();


    txBuffer[0] = 0b1000'0100;
    txBuffer[1] = 0;

    DRV8316_CS_Clear();

    SERCOM4_SPI_WriteRead(&txBuffer[0], 2, &rxBuffer[0], 2);

    DRV8316_CS_Set();

    return rxBuffer[0];
}
