#include "as5047p.hpp"

AS5047P::AS5047P() {
        ENCODER_CS_Set();
        spiRequest.chipSelectPin = ENCODER_CS_PIN;
        isSensorBusy = false;
}

bool AS5047P::readDeviceRegister(RegisterAddress registerAddress) {
        uint16_t commandFrame = ReadWriteCommandMask::READ | registerAddress;

        // Calculate parity bit -- ARM GCC built-in command for popcnt
        if (std::popcount(commandFrame) % 2 == 0)
                commandFrame = commandFrame | static_cast<uint16_t>(ParityBit::PARITY_BIT_0);
        else
                commandFrame = commandFrame | static_cast<uint16_t>(ParityBit::PARITY_BIT_1);

        const auto CommandFrameMSB = static_cast<uint8_t>(commandFrame >> 8);
        const auto CommandFrameLSB = static_cast<uint8_t>(commandFrame & 0b1111'1111);

        spiRequest.txBuffer = {CommandFrameMSB, CommandFrameLSB};
        spiRequest.chipSelectPin = ENCODER_CS_PIN;
        spiRequest.callback = &AS5047P::spiTransferCallback;
        spiRequest.context = this;

        isSensorBusy = true;

        if (SPI_Buffer::submit(spiRequest) != SPI_Buffer::TransactionState::PLACED) {
                isSensorBusy = false;
                return false;
        }

        return true;
}

void AS5047P::writeDeviceRegister(RegisterAddress registerAddress, RegisterData_t data) {
        const uint16_t CommandFrame = 0b1011'1111'1111'1101;
        const auto CommandFrameMSB = static_cast<uint8_t>(CommandFrame >> 8);
        const auto CommandFrameLSB = static_cast<uint8_t>(CommandFrame & 0b1111'1111);

        const auto DataFrameMSB = static_cast<uint8_t>(data >> 8);
        const auto DataFrameLSB = static_cast<uint8_t>(data & 0b1111'1111);

        std::array<uint8_t, CommandFrameSize> addressBuffer{CommandFrameMSB, CommandFrameLSB};
        std::array<uint8_t, DataFrameSize> dataBuffer{DataFrameMSB, DataFrameLSB};
}

bool AS5047P::request(RegisterAddress registerAddress) {
        if (readDeviceRegister(registerAddress))
                return true;

        return false;
}

void AS5047P::spiTransferCallback(void* context) {
        auto* self = static_cast<AS5047P*>(context);

        // Hold reference? For 2 elements copy constructor is basically free
        // const auto& rxBuffer = self->spiJob.rxBuffer;
        const auto rxBuffer = self->spiRequest.rxBuffer;

        const auto PARD_Bit = static_cast<uint8_t>(rxBuffer[0] & 0b1000'0000);
        const auto ParityCount = std::popcount(rxBuffer[0]) + std::popcount(rxBuffer[1]);

        if (ParityCount % 2 == 1) {
                self->latestRegisterRequested = 0;
                return;
        }

        const auto EF_Bit = static_cast<uint8_t>(rxBuffer[0] & 0b0100'0000);

        if (EF_Bit == 0b0100'0000) {
                self->latestRegisterRequested = 0;
                return;
        }

        self->latestRegisterRequested = static_cast<RegisterData_t>(
                static_cast<RegisterData_t>(rxBuffer[0] & 0b0011'1111) << 8 | static_cast<RegisterData_t>(rxBuffer[1]));

        isSensorBusy = false;
}

AS5047P::Angle_t AS5047P::measureAngleUncompensated() const {
        // const auto AngleUncData = readDeviceRegister(RegisterAddress::ANGLEUNC);
        const auto AngleUncData = latestRegisterRequested;

        return static_cast<Angle_t>(AngleUncData) / static_cast<Angle_t>(AngleResolutionSPI) * FullRotationDegrees;
}

AS5047P::Angle_t AS5047P::measureAngleCompensated() const {
        // const auto AngleComData = readDeviceRegister(RegisterAddress::ANGLECOM);
        const auto AngleComData = latestRegisterRequested;

        return static_cast<Angle_t>(AngleComData) / static_cast<Angle_t>(AngleResolutionSPI) * FullRotationDegrees;
}

uint16_t AS5047P::measureAngleCompensatedRaw() const { return latestRegisterRequested; }

AS5047P::FieldMagnitude_t AS5047P::measureFieldMagnitude() const {
        // return readDeviceRegister(RegisterAddress::MAG);
        return latestRegisterRequested;
}

void AS5047P::readAGC_Diagnostics() const {
        const auto DIAAGC_Data = latestRegisterRequested;

        // if (DIAAGC_Data & static_cast<RegisterData_t>(DIAAGC_RegisterMask::MAG_FIELD_TOO_LOW))
        //         // Logger_Info("AGC Diagnostics: The magnetic field is too low\r\n");
        //
        // if (DIAAGC_Data & static_cast<RegisterData_t>(DIAAGC_RegisterMask::MAG_FIELD_TOO_HIGH))
        //         // Logger_Info("AGC Diagnostics: The magnetic field is too high\r\n");
        //
        // if (DIAAGC_Data & static_cast<RegisterData_t>(DIAAGC_RegisterMask::CORDIC_OVF))
        //         // Logger_Info("AGC Diagnostics: CORDIC overflow\r\n");
        //
        // if (DIAAGC_Data & static_cast<RegisterData_t>(DIAAGC_RegisterMask::OFFSET_COMP))
        //         // Logger_Info("AGC Diagnostics: Offset compensation\r\n");
}

void AS5047P::readAndClearErrorFlags() const {
        const auto ERRFL_Data = latestRegisterRequested;

        // if (ERRFL_Data & static_cast<RegisterData_t>(ERRFL_RegisterMask::PARITY_ERROR))
        //         // Logger_Info("Error flag: Parity error\r\n");
        //
        // if (ERRFL_Data & static_cast<RegisterData_t>(ERRFL_RegisterMask::INVALID_COMMAND))
        //         // Logger_Info("Error flag: Invalid command\r\n");
        //
        // if (ERRFL_Data & static_cast<RegisterData_t>(ERRFL_RegisterMask::FRAMING_ERROR))
        //         // Logger_Info("Error flag: Framing error\r\n");
}

bool AS5047P::sensorBusy() { return isSensorBusy; }
