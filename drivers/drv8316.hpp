#pragma once

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include "definitions.h"
#include "spi_buffer.hpp"

using namespace ATSAMD21_GGKOGKOU;

class DRV8316 {
public:
        using RegisterAddress_t = uint8_t;
        using RegisterMask_t = uint16_t;

        /**
         * @enum RegisterAddress
         *
         * A user-defined type containing all the registers and their memory-mapped address
         */
        enum class RegisterAddress : RegisterAddress_t {
                /// Status Registers
                IC_Status_Register = 0x00,
                Status_Register_1 = 0x01,
                Status_Register_2 = 0x02,

                /// Control Registers
                Control_Register_1 = 0x03,
                Control_Register_2 = 0x04,
                Control_Register_3 = 0x05,
                Control_Register_4 = 0x06,
                Control_Register_5 = 0x07,
                Control_Register_6 = 0x08,
                Control_Register_10 = 0xC0,
        };

        DRV8316() = default;
        ~DRV8316() = default;

        /**
         * The PWM modes supported by the DRV8316 (Control_Register_2)
         */
        enum class PWM_Mode : uint8_t {
                MODE_6x = 0x0,
                MODE_6x_CURRENT_LIM = 0x1,
                MODE_3x = 0x2,
                MODE_3x_CURRENT_LIM = 0x3,
        };

        /**
         * The PWM modes supported by the DRV8316 (Control_Register_5)
         */
        enum class CurrentSenseGain : uint8_t {
                CSA_GAIN_0_15 = 0x0,
                CSA_GAIN_0_30 = 0x1,
                CSA_GAIN_0_6 = 0x2,
                CSA_GAIN_1_2 = 0x3,
        };

        /**
         * SPI_Request object; passed between the SPI_Buffer layer and AS5047 driver
         */
        SPI_Request spiRequest;

        /**
         * Function that sets the PWM mode to the device
         *
         * @param pwmMode The user selected PWM mode
         */
        void setPWMMode(PWM_Mode pwmMode);

        /**
         * Function that sets the gain of the internal current sense amplifier
         *
         * @param gain The user selected gain (in V/A)
         */
        void setCurrentSenseAmplifierGain(CurrentSenseGain gain);

        /**
         * Function that locks all registers
         */
        void lockAllRegisters();

        /**
         * Function that unlocks all registers
         */
        void unlockAllRegisters();

        /**
         * Function that sets the offset voltages
         * @param offsetVoltageB
         * @param offsetVoltageC
         */
        void setOffsetVoltages(float offsetVoltageB, float offsetVoltageC) {
                offsetCorrectionVoltageB = offsetVoltageB;
                offsetCorrectionVoltageC = offsetVoltageC;
        }

        /**
         * Function that applies the offset correction equations
         * @see [Datasheet p.39](https://www.ti.com/lit/ds/symlink/drv8316.pdf)
         *
         * @param IA Voltage measured by the Analog to Digital Controller from the SOA pin
         * @param IB Voltage measured by the Analog to Digital Controller from the SOB pin
         * @param IC Voltage measured by the Analog to Digital Controller from the SOC pin
         */
        void calculateCurrents(float& IA, float& IB, float& IC);

        /**
         * Function that checks whether the DRV8316 device has an error condition that prevents it from operating
         *
         * @return True if there is a fault condition
         */
        [[nodiscard]] bool checkForFaults();

        /**
         * Function that flags whether the device is ready or there is an ongoing SPI transaction
         * @return True if there is no ongoing SPI transaction
         */
        [[nodiscard]] inline bool deviceIsReady() const { return isReady; }

private:
        /**
         * Type-alias to a register AND mask
         */
        enum class SPI_Operation : uint8_t {
                WRITE = 0,
                READ = 1,
        };

        /**
         * The IC_Status_Register masks
         */
        enum class IC_Status_Register_Mask : RegisterMask_t {
                BK_FLT = 0b0100'0000,
                SPI_FLT = 0b0010'0000,
                OCP = 0b0001'0000,
                NPOR = 0b0000'1000,
                OVP = 0b0000'0100,
                OT = 0b0000'0010,
                FAULT = 0b0000'0001,
        };

        enum class Status_Register_1_Mask : RegisterMask_t {
                OTW = 0b1000'0000,
                OTS = 0b0100'0000,
                OCP_HC = 0b0010'0000,
                OCL_LC = 0b0001'0000,
                OCP_HB = 0b0000'1000,
                OCP_LB = 0b0000'0100,
                OCP_HA = 0b0000'0010,
                OCP_LA = 0b0000'0001,
        };

        enum class Status_Register_2_Mask : RegisterMask_t {
                OTP_ERR = 0b0100'0000, /// One-Time-Programmability error
                BUCK_OCP = 0b0010'0000, /// Buck regulator overcurrent
                BUCK_UV = 0b0001'0000, /// Buck regulator undervoltage
                VCP_UV = 0b0000'1000, /// Charge pump undervoltage
                SPI_PARITY = 0b0000'0100, /// SPI parity error
                SPI_SCLK_FLT = 0b0000'0010, /// SPI clock framing error
                SPI_ADDR_FLT = 0b0000'0001, /// SPI address fault
        };

        enum class Control_Register_1_Mask : RegisterMask_t {
                REG_LOCK = 0b0000'0111,
        };

        enum class Control_Register_2_Mask : RegisterMask_t {
                SDO_MODE = 0b0010'0000,
                SLEW = 0b0001'1000,
                PWM_MODE = 0b0000'0110,
                CLR_FLT = 0b0000'0001,
        };

        enum class Control_Register_3_Mask : RegisterMask_t {
                PWM_100_DUTY_SEL = 0b0001'0000,
                OVP_SEL = 0b0000'1000,
                OVP_EN = 0b0000'0100,
                OTW_REP = 0b0000'0001,
        };

        enum class Control_Register_4_Mask : RegisterMask_t {
                DRV_OFF = 0b1000'0000,
                OCP_CBC = 0b0100'0000,
                OCP_DEG = 0b0011'0000,
                OCP_RETRY = 0b0000'1000,
                OCP_LVL = 0b0000'0100,
                OCP_MODE = 0b0000'0011,
        };

        enum class Control_Register_5_Mask : RegisterMask_t {
                ILIM_RECIR = 0b0100'0000,
                EN_AAR = 0b0000'1000,
                EN_ASR = 0b0000'0100,
                CSA_GAIN = 0b0000'0011,
        };

        enum class Control_Register_6_Mask : RegisterMask_t {
                BUCK_PS_DIS = 0b0001'0000,
                BUCK_CL = 0b0000'1000,
                BUCK_SEL = 0b0000'0110,
                BUCK_DIS = 0b0000'0001,
        };

        enum class Control_Register_10_Mask : RegisterMask_t {
                DLYCMP_EN = 0b0001'0000,
                DLY_TARGET = 0b0000'1111,
        };

        /**
         * Offset voltage for phase B -- the voltage when all the OUTx are GND (or Hi-z??)
         */
        float offsetCorrectionVoltageB = 0.0f;

        /**
         * Offset voltage for phase C -- the voltage when all the OUTx are GND (or Hi-z??)
         */
        float offsetCorrectionVoltageC = 0.0f;

        /**
         * The Current Sense Amplifier (CSA) gain as a float -- for usage in calculations
         */
        float csaGain = 0.6f;

        /**
         * Boolean variable that flags whether the device is ready or there is an ongoing SPI transaction
         */
        bool isReady = true;

        /**
         * The value of the register that was read last -- essentially a caching variable
         */
        uint8_t latestRegisterValueRead = 0;

        /**
         * Function that serves as the callback function for the SPI Read transaction completion
         *
         * @param context
         */
        static void spiReadTransferCallback(void* context);

        /**
         * Function that serves as the callback function for the SPI Write transaction completion
         *
         * @param context
         */
        static void spiWriteTransferCallback(void* context);

        /**
         * Function that writes SPI words to the DRV8316 device
         *
         * @note For DRV8316, 1 word equals 16-bit of data into one packet
         *
         * @param registerAddress The 6-bit address of the register to write to
         * @param dataToWrite The 8-bit data word to write to the register
         * @return True if SPI transaction was placed successfully
         */
        bool writeRegister(RegisterAddress registerAddress, uint8_t dataToWrite);

        /**
         * Function that reads from the DRV8316 via SPI protocol
         *
         * @note For READ operation the DATA[7:0] is ignored, so for simplicity we write 0x0
         * @param registerAddress The register to be read
         * @return True if SPI transaction was placed successfully
         */
        [[nodiscard]] bool readRegister(RegisterAddress registerAddress);
};
