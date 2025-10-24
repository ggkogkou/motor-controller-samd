#pragma once

#include <cstdint>
#include <array>
#include <cmath>
#include <algorithm>
#include "logger.h"
#include "plib_sercom1_spi_master.h"
#include "plib_sercom4_spi_master.h"

class DRV8316 {
public:

    DRV8316() = default;
    ~DRV8316() = default;

    enum class PWM_Mode : uint8_t {
        MODE_6x = 0x0,
        MODE_6x_CURRENT_LIM = 0x1,
        MODE_3x = 0x2,
        MODE_3x_CURRENT_LIM = 0x3,
    };

    /**
     * Function that sets the PWM mode to the device
     * @param pwmMode
     */
    void setPWMMode(PWM_Mode pwmMode);

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

private:
    /**
     * Function that writes SPI words to the DRV8316 device
     *
     * @note For DRV8316, 1 word equals 16-bit of data into one packet
     *
     * @param registerAddress The 6-bit address of the register to write to
     * @param dataToWrite The 8-bit data word to write to the register
     */
    void writeRegister(RegisterAddress registerAddress, uint8_t dataToWrite);

    /**
     * Function that reads from the DRV8316 via SPI protocol
     *
     * @note For READ operation the DATA[7:0] is ignored so for simplicity we write 0x0
     * @param registerAddress The register to be read
     * @return The 8-bit content of the register - ignore the status bits [15:8]
     */
    [[nodiscard]] uint8_t readRegister(RegisterAddress registerAddress);

    /**
     * Type-alias to a register AND mask
     */
    using RegisterMask_t = uint16_t;

    enum class SPI_Operation : uint8_t {
        WRITE = 0,
        READ = 1,
    };

    /**
     * The IC_Status_Register masks
     */
    enum class IC_Status_Register_Mask : RegisterMask_t {
        BK_FLT  = 0b0100'0000,
        SPI_FLT = 0b0010'0000,
        OCP     = 0b0001'0000,
        NPOR    = 0b0000'1000,
        OVP     = 0b0000'0100,
        OT      = 0b0000'0010,
        FAULT   = 0b0000'0001,
    };

    enum class Status_Register_1_Mask : RegisterMask_t {
        OTW    = 0b1000'0000,
        OTS    = 0b0100'0000,
        OCP_HC = 0b0010'0000,
        OCL_LC = 0b0001'0000,
        OCP_HB = 0b0000'1000,
        OCP_LB = 0b0000'0100,
        OCP_HA = 0b0000'0010,
        OCP_LA = 0b0000'0001,
    };

    enum class Status_Register_2_Mask : RegisterMask_t {
        OTP_ERR      = 0b0100'0000, /// One-Time-Programmability error
        BUCK_OCP     = 0b0010'0000, /// Buck regulator overcurrent
        BUCK_UV      = 0b0001'0000, /// Buck regulator undervoltage
        VCP_UV       = 0b0000'1000, /// Charge pump undervoltage
        SPI_PARITY   = 0b0000'0100, /// SPI parity error
        SPI_SCLK_FLT = 0b0000'0010, /// SPI clock framing error
        SPI_ADDR_FLT = 0b0000'0001, /// SPI address fault
    };

    enum class Control_Register_2_Mask : RegisterMask_t {
        SDO_MODE  = 0b0010'0000,
        SLEW      = 0b0001'1000,
        PWM_MODE  = 0b0000'0110,
        CLR_FLT   = 0b0000'0001,
    };

    enum class Control_Register_3_Mask : RegisterMask_t {
        PWM_100_DUTY_SEL = 0b0001'0000,
        OVP_SEL          = 0b0000'1000,
        OVP_EN           = 0b0000'0100,
        OTW_REP          = 0b0000'0001,
    };

    enum class Control_Register_4_Mask : RegisterMask_t {
        DRV_OFF   = 0b1000'0000,
        OCP_CBC   = 0b0100'0000,
        OCP_DEG   = 0b0011'0000,
        OCP_RETRY = 0b0000'1000,
        OCP_LVL   = 0b0000'0100,
        OCP_MODE  = 0b0000'0011,
    };

    enum class Control_Register_5_Mask : RegisterMask_t {
        ILIM_RECIR = 0b0100'0000,
        EN_AAR     = 0b0000'1000,
        EN_ASR     = 0b0000'0100,
        CSA_GAIN   = 0b0000'0011,
    };

    enum class Control_Register_6_Mask : RegisterMask_t {
        BUCK_PS_DIS = 0b0001'0000,
        BUCK_CL     = 0b0000'1000,
        BUCK_SEL    = 0b0000'0110,
        BUCK_DIS    = 0b0000'0001,
    };

    enum class Control_Register_10_Mask : RegisterMask_t {
        DLYCMP_EN  = 0b0001'0000,
        DLY_TARGET = 0b0000'1111,
    };


};
