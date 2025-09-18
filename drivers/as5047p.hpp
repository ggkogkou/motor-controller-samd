#pragma once

#define _USE_MATH_DEFINES
#include <cstdint>
#include <array>
#include <type_traits>
#include <logger.h>
#include <limits>
#include <cmath>
#include "plib_sercom1_spi_master.h"

struct AS5047P_Config {
    enum class RotationDirection : uint16_t {
        CLOCKWISE           = 0b0000'0000,
        COUNTER_CLOCKWISE   = 0b0000'0100,
    };

    enum class PWM_OutputPin : uint16_t {
        PIN_W = 0b0000'0000, // ABI is operating , I is used as PWM
        PIN_I = 0b0000'1000, // UVW is operating , I is used as PWM
    };

    enum class DAEC_Status : uint16_t {
        ENABLED     = 0b0000'0000,
        DISABLED    = 0b0001'0000,
    };

    enum class DataSelect : uint16_t {
        DAEC_ANG    = 0b0000'0000,
        CORDIC_ANG  = 0b0100'0000,
    };

    enum class PWM_Status : uint16_t {
        ENABLED     = 0b1000'0000,
        DISABLED    = 0b0000'0000,
    };

    /**
     * The 14-bit zero position (default 0x0000)
     */
    uint16_t zeroPosition = 0x0000;

    /**
     * The rotation direction, bit DIR of SETTINGS1
     */
    RotationDirection rotationDirection = RotationDirection::CLOCKWISE;

    /**
     * Bit UVW_ABI of SETTINGS1
     */
    PWM_OutputPin pwmOutputPin = PWM_OutputPin::PIN_W;

    /**
     * Bit DAECDIS of SETTINGS1
     */
    DAEC_Status dynamicAngleCompensation = DAEC_Status::ENABLED;

    /**
     * Bit Dataselect of SETTINGS1
     */
    DataSelect dataSelect = DataSelect::DAEC_ANG;

    /**
     * Bit PWMon of SETTINGS1
     */
    PWM_Status pwmStatus = PWM_Status::DISABLED;

    enum class PolePairs : uint8_t {
        ONE = 0b000,
        TWO = 0b001,
        THREE = 0b010,
        FOUR = 0b011,
        FIVE = 0b100,
        SIX = 0b101,
        SEVEN = 0b110,
        EIGHT = 0b111,
    };

    enum class HysteresisBits : uint8_t {
        THREE = 0b00,
        TWO = 0b01,
        ONE = 0b10,
        ZERO = 0b11,
    };

    enum class ABI_Resolution : uint8_t {
        ABIRES_000 = 0b000,
        ABIRES_001 = 0b001,
        ABIRES_010 = 0b010,
        ABIRES_011 = 0b011,
        ABIRES_100 = 0b100,
        ABIRES_101 = 0b101,
        ABIRES_110 = 0b110,
        ABIRES_111 = 0b111,
    };

    enum class ABIBIN : uint8_t {
        DECIMAL = 0,
        BINARY = 1,
    };
};


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
 *
 * SPI clock frequency has a maximum of 10MHz
 */
class AS5047P {
public:
    /**
     * Default constructor of the driver class
     *
     * Meant to be used when default configurations are OK or device OTP is programmed
     */
    AS5047P() {
        AS5047P_CS_Set();
    }

    /**
     * Constructor that initializes the device with the user-defined configurations
     *
     * @param config The full user configuration of the device
     */
    explicit AS5047P(AS5047P_Config config);

    /**
     * Default destrcutor of the class
     */
    ~AS5047P() = default;

    /**
     * Type aliases for the device registers addresses and data
     */
    using RegisterAddress_t = std::uint16_t;
    using RegisterData_t = std::uint16_t;
    using ReadWriteCommandMask_t = std::uint16_t;
    using Angle_t = float;
    using FieldMagnitude_t = float;

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
        ERRFL      = 0x0001,  // Error register (Read-Only)
        PROG       = 0x0003,  // Programming register (Read/Write)
        DIAAGC     = 0x3FFC,  // Diagnostic and AGC (Read-Only)
        MAG        = 0x3FFD,  // CORDIC magnitude (Read-Only)
        ANGLEUNC   = 0x3FFE,  // Measured angle without dynamic angle error compensation (Read-Only)
        ANGLECOM   = 0x3FFF,  // Measured angle with dynamic angle error compensation (Read-Only)

        /// Non-volatile registers
        ZPOSM     = 0x0016,  // Zero position MSB (Read/Write/Program)
        ZPOSL     = 0x0017,  // Zero position LSB/MAG diagnostic (Read/Write/Program)
        SETTINGS1 = 0x0018,  // Custom setting register 1 (Read/Write/Program)
        SETTINGS2 = 0x0019   // Custom setting register 2 (Read/Write/Program)
    };

    /**
     * @enum ReadWriteCommandMask
     * @brief Represents the command masks for read and write operations
     *
     * The mask is applied to the command frame to encode the type of transaction
     */
    enum class ReadWriteCommandMask : uint16_t {
        WRITE   = 0x0000,
        READ    = 0x4000,
    };

    /**
     * @enum ParityBit
     * @brief Represents the parity bit options for the SPI communication with the AS5047P sensor.
     *
     * The AS5047P uses a single parity bit in its SPI protocol for error detection in communication.
     */
    enum class ParityBit : uint16_t {
        PARITY_BIT_0 = 0b0000'0000'0000'0000,
        PARITY_BIT_1 = 0b1000'0000'0000'0000,
    };

    /**
     * The SPI command frame size in bytes
     */
    static constexpr size_t CommandFrameSize = 2;

    /**
     * The SPI data frame size in bytes
     */
    static constexpr size_t DataFrameSize = 2;

    /**
     * Function that performs the SPI Read operation between MCU and AS5047P magnetic encoder
     *
     * @param registerAddress The address of the device register
     * @return The device register's data
     */
    [[nodiscard]] RegisterData_t readDeviceRegister(RegisterAddress registerAddress) const;

    /**
     * Function that performs the SPI Write operation between MCU and AS5047P magnetic encoder
     *
     * @param registerAddress The address of the device register
     * @param data The data that will be written into the specified device register
     */
    void writeDeviceRegister(RegisterAddress registerAddress, RegisterData_t data) const;

    /**
     * Function that reads the uncompensated angle (skip DAEC)
     *
     * @return The 14-bit measured angle uncompensated
     */
    [[nodiscard]] Angle_t measureAngleUncompensated() const;

    /**
     * Function that reads the compensated angle (DAEC output)
     *
     * @return The 14-bit measured angle compensated
     */
    [[nodiscard]] Angle_t measureAngleCompensated() const;

    /**
     * Function that reads the CORDIC magnetic field magnitude
     *
     * @return The 14-bit measured magnetic field magnitude
     */
    [[nodiscard]] FieldMagnitude_t measureFieldMagnitude() const;

    /**
     * @enum DIAAGC_RegisterMask
     * @brief Represents the bitmask definitions for the DIAAGC register.
     *
     * The DIAAGC register provides diagnostic and automatic gain control (AGC) information.
     * This enumeration defines specific flags and fields within the register that
     * indicate diagnostics or configuration status of the AS5047P sensor.
     */
    enum class DIAAGC_RegisterMask : RegisterData_t {
        MAG_FIELD_TOO_LOW   = 0b0000'1000'0000'0000,
        MAG_FIELD_TOO_HIGH  = 0b0000'0100'0000'0000,
        CORDIC_OVF          = 0b0000'0010'0000'0000,
        OFFSET_COMP         = 0b0000'0001'0000'0000,
        AGC_VALUE           = 0b0000'0000'1111'1111,
    };

    /**
     * Function that reads the DIAAGC register and logs the warnings
     *
     * TODO: Return the warnings properly in a data structure
     */
    void readAGC_Diagnostics() const;

    /**
     * @enum ERRFL_RegisterMask
     * @brief Represents the bitmask definitions for the ERRFL (Error Flag) register.
     *
     * The ERRFL register provides diagnostic error flags that indicate specific sensor errors.
     * Each bit in this enumeration corresponds to a particular type of error detected by the sensor.
     */
    enum class ERRFL_RegisterMask : RegisterData_t {
        PARITY_ERROR    = 0b0000'0000'0000'0100,
        INVALID_COMMAND = 0b0000'0000'0000'0010,
        FRAMING_ERROR   = 0b0000'0000'0000'0001,
    };

    /**
     * Function that reads and clears (by IC design) the error flags from ERRFL register
     */
    void readAndClearErrorFlags() const;

    /**
     * AS5047P 14-bit angular resolution: number of discrete angle steps per 360° revolution
     * Range: 0 to 16383 (0x0000 to 0x3FFF), providing ~0.022° per step
     */
    static constexpr uint16_t AngleResolutionSPI = 16384;
    static_assert(AngleResolutionSPI <= std::numeric_limits<uint16_t>::max(), "AngleResolutionSPI must fit in 'int'");

    /**
     * The full rotation angle in degrees
     */
    static constexpr float FullRotationDegrees = 360.0f;
};

constexpr uint16_t operator|(AS5047P::ReadWriteCommandMask commandMask, AS5047P::RegisterAddress registerAddress) {
    return static_cast<uint16_t>(commandMask) | static_cast<uint16_t>(registerAddress);
}
