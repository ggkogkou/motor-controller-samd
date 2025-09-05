#pragma once

#include <cstdint>
#include <logger.h>

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
    };

    std::uint16_t read_angle();

    void selectOutputMode(OutputMode output_mode) const;

private:

    using RegisterAddress_t = std::uint16_t;

    /**
     * @enum VolatileRegisters
     * @brief Enumerates the volatile register addresses of the AS5047P magnetic rotary sensor.
     *
     * Volatile registers are used for frequently updated or real-time data, such as
     * angle measurements or diagnostics. These registers are accessed to retrieve
     * or process sensor data dynamically.
     */
    enum class VolatileRegisters : RegisterAddress_t {
        NOP        = 0x0000,  // No operation
        ERRFL      = 0x0001,  // Error register
        PROG       = 0x0003,  // Programming register
        DIAAGC     = 0x3FFC,  // Diagnostic and AGC
        MAG        = 0x3FFD,  // CORDIC magnitude
        ANGLEUNC   = 0x3FFE,  // Measured angle without dynamic angle error compensation
        ANGLECOM   = 0x3FFF   // Measured angle with dynamic angle error compensation
    };

    /**
     * @enum NonVolatileRegisters
     * @brief Enumerates the non-volatile register addresses of the AS5047P magnetic rotary sensor.
     *
     * Non-volatile registers store user-configured settings or calibration values that
     * remain persistent across power cycles. These registers can be used to configure
     * the sensor's operational parameters or store calibration data for precise angle measurements.
     */
    enum class NonVolatileRegisters : RegisterAddress_t {
        ZPOSM     = 0x0016,  // Zero position MSB
        ZPOSL     = 0x0017,  // Zero position LSB/MAG diagnostic
        SETTINGS1 = 0x0018,  // Custom setting register 1
        SETTINGS2 = 0x0019   // Custom setting register 2
    };


};
