#pragma once

#include <cstdint>
#include <array>
#include <cmath>
#include <algorithm>
#include "logger.h"

class DRV8316 {
public:

    using RegisterAddress_t = uint8_t;
    enum class RegisterAddress : RegisterAddress_t {
        /// Status Registers
        IC_Status_Register  = 0x00,
        Status_Register_1   = 0x01,
        Status_Register_2   = 0x02,

        /// Control Registers
        Control_Register_1  = 0x03,
        Control_Register_2  = 0x04,
        Control_Register_3  = 0x05,
        Control_Register_4  = 0x06,
        Control_Register_5  = 0x07,
        Control_Register_6  = 0x08,
        Control_Register_10 = 0xC0,
    };

    using RegisterMask_t = uint8_t;
    enum class RegisterMask : RegisterMask_t {

    };



private:
    enum class IC_Status_RegisterMask : uint8_t {
        BK_FLT  = 0b0100'0000,
        SPI_FLT = 0b0010'0000,
        OCP     = 0b0001'0000,
        NPOR    = 0b0000'1000,
        OVP     = 0b0000'0100,
        OT      = 0b0000'0010,
        FAULT   = 0b0000'0001,
    };

    enum class Status_Register_1_Mask : uint8_t {
        OTW    = 0b1000'0000,
        OTS    = 0b0100'0000,
        OCP_HC = 0b0010'0000,
        OCL_LC = 0b0001'0000,
        OCP_HB = 0b0000'1000,
        OCP_LB = 0b0000'0100,
        OCP_HA = 0b0000'0010,
        OCP_LA = 0b0000'0001,
    };

    enum class Status_Register_2_Mask : uint8_t {
        OTP_ERR      = 0b0100'0000, /// One-Time-Programmability error
        BUCK_OCP     = 0b0010'0000, /// Buck regulator overcurrent
        BUCK_UV      = 0b0001'0000, /// Buck regulator undervoltage
        VCP_UV       = 0b0000'1000, /// Charge pump undervoltage
        SPI_PARITY   = 0b0000'0100, /// SPI parity error
        SPI_SCLK_FLT = 0b0000'0010, /// SPI clock framing error
        SPI_ADDR_FLT = 0b0000'0001, /// SPI address fault
    };

    enum class Control_Register_2_Mask : uint8_t {
        SDO_MODE  = 0b0010'0000,
        SLEW      = 0b0001'1000,
        PWM_MODE  = 0b0000'0110,
        CLR_FLT   = 0b0000'0001,
    };

};
