#pragma once

#include <cstdint>
#include <array>
#include <logger.h>
#include "plib_sercom1_spi_master.h"

/**
 * @class AS5047P
 * @brief Represents an interface for interacting with the AS5047P magnetic rotary sensor.
 *
 * The AS5047P is a high-resolution magnetic rotary encoder. This class provides
 * functionality to read the current angle value reported by the sensor.
 *
 * SPI Transaction
 * ----------------
 * Consists of a 16-bit command frame follower by a 16-bit data frame
 *
 * Command frame => | PARC | RW | ADDR |
 *                  |  15  | 14 | 13:0 |
 *
 * Read data frame => | PARD | EF | DATA |
 *                    |  15  | 14 | 13:0 |
 */
class AS5047P {
public:
    AS5047P() = default;
    ~AS5047P() = default;

    enum class OutputMode {
        SPI,
        ABI,
        PWM,
        UVW,
    };

    void selectOutputMode(OutputMode output_mode) const;

    /**
     * Type aliases for the device registers addresses and data
     */
    using RegisterAddress_t = std::uint16_t;
    using RegisterData_t = std::uint16_t;
    using ReadWriteCommandMask_t = std::uint16_t;

    /**
     * @enum RegisterAddress
     * @brief Lists all the register addresses (both volatile and non-volatile) for the AS5047P magnetic rotary sensor
     *
     * This enumeration provides the complete list of both volatile and non-volatile register addresses
     *
     * Volatile registers are typically used for real-time data such as angle measurements or diagnostics
     *
     * Non-volatile registers are used for storing calibration or configuration settings that persist across power cycles
     *
     * Each register has a 14-bit address and holds 14-bit data
     */
    enum class RegisterAddress : RegisterAddress_t {
        /// Volatile registers
        NOP        = 0x0000,  // No operation
        ERRFL      = 0x0001,  // Error register
        PROG       = 0x0003,  // Programming register
        DIAAGC     = 0x3FFC,  // Diagnostic and AGC
        MAG        = 0x3FFD,  // CORDIC magnitude
        ANGLEUNC   = 0x3FFE,  // Measured angle without dynamic angle error compensation
        ANGLECOM   = 0x3FFF,  // Measured angle with dynamic angle error compensation

        /// Non-volatile registers
        ZPOSM     = 0x0016,  // Zero position MSB
        ZPOSL     = 0x0017,  // Zero position LSB/MAG diagnostic
        SETTINGS1 = 0x0018,  // Custom setting register 1
        SETTINGS2 = 0x0019   // Custom setting register 2
    };

    /**
     * @enum ReadWriteCommandMask
     * @brief Represents the command masks for read and write operations
     *
     * The mask is applied to the command frame to encode the type of transaction
     */
    enum class ReadWriteCommandMask : uint16_t {
        READ    = 0x0000,
        WRITE   = 0x4000,
    };

    /**
     * Function that performs the SPI transaction between MCU and AS5047P magnetic encoder
     *
     * @param address The address of the device register
     * @return The device register's data
     */
    RegisterData_t readDeviceRegister(RegisterAddress address);

    enum class ERRFL_RegisterMask : uint8_t {
        PARITY_ERROR = 0b0000'0100,
        INVALID_COMMAND = 0b0000'0010,
        FRAMING_ERROR = 0b0000'0001,
    };

    enum class DIAAGC_RegisterMask : uint16_t {
        MAGNETIC_FIELD_TOO_LOW = 0b0000'1000'0000'0000,
        MAGNETIC_FIELD_TOO_HIGH = 0b0000'0100'0000'0000,
        CORDIC_OVERFLOW = 0b0000'0010'0000'0000,
        OFFSET_COMPENSATION = 0b0000'0001'0000'0000,
        AGC_VALUE = 0b0000'0000'1111'1111,
    };

    enum class MAG_RegisterMask : uint16_t {
        CORDIC_MAGNITUDE = 0b0011'1111'1111'1111,
    };

    enum class ANGLEUNC_RegisterMask : uint16_t {
        ANGLE_INFORMATION = 0b0011'1111'1111'1111,
    };

    enum class ANGLECOM_RegisterMask : uint16_t {
        ANGLE_INFORMATION = 0b0011'1111'1111'1111,
    };

    enum class ABI_RotationDirection {
        CLOCKWISE,
        COUNTER_CLOCKWISE,
    };

    enum class PWM_OutputPin {
        PIN_W, // ABI is operating , I is used as PWM
        PIN_I, // UVW is operating , I is used as PWM
    };

    /**
     * Function that disables the Dynamic Angle Compensation (DAEC)
     *
     * @brief For disabling DAEC set Bit 4 of SETTINGS1 register to logic HIGH
     */
    void disableDynamicAngleCompensation();

    enum class ErrorFlag {
        PARITY_ERROR,
        INVALID_COMMAND,
        FRAMING_ERROR,
    };

    uint16_t measureAngleUncompensated();
    uint16_t measureAngleCompensated();
    uint16_t measureFieldMagnitude();

    void setZeroPosition(uint16_t zero_position);
    void configureSettings(uint16_t settings);

private:
    /**
     *
     * @param zero_position
     */
    void configureZeroPosition_Register(uint16_t zero_position);

    void configureSettings1_Register();

    void configureSettings2_Register();

};
