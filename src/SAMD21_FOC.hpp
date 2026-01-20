// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Georgios Gkogkou <ggkogkou125@gmail.com>

/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

/**
 * @file   SAMD21_FOC.hpp
 * @brief  SAMD21-specific FOC for BLDC
 * @author Georgios Gkogkou <ggkogkou125@gmail.com>
 */

#pragma once

#include <cstdint>
#include "as5047p.hpp"
#include "definitions.h"
#include "pmsm_controller.hpp"

namespace PermanentMagnetSynchronousMotor {

/**
 * Hardware-specific Field-Oriented Control implementation for brushless DC motors
 */
class SAMD21_FOC {
public:
        /**
         * Constructor of the class
         *
         * Initializes the motor control peripherals, registers the callback functions and primes encoder
         */
        SAMD21_FOC();

        /**
         * Compiler generated default destructor
         */
        ~SAMD21_FOC() = default;

        /**
         * Function that stops the motor from spinning by applying a zero-vector to the 3-phase inverter
         */
        void stop() const;

private:
        static void ADC_Callback(ADC_STATUS status, uintptr_t context);
        static void TC3_Callback(TC_TIMER_STATUS status, uintptr_t context);

        void ADC_Callback(ADC_STATUS status);
        void TC3_FOC_Handler(TC_TIMER_STATUS status);

        /**
         * Function that writes the cached PWM periods to the TCC0 registers
         */
        void setPWM_DutyCycles() const;

        /**
         * The ADC reference voltage in mV (1/1.48*Vdd)
         */
        static constexpr int32_t ADC_VREF_mV = 2'230;

        /**
         * The ADC resolution which is 12-bit thus 2^12
         */
        static constexpr int32_t ADC_Resolution = 4'096;

        /**
         * The maximum value that the ADC peripheral can read
         */
        static constexpr int32_t ADC_MaximumRawValue = ADC_Resolution - 1;

        /**
         * Helper function that converts the raw 12-bit ADC reading to the corresponding voltage (in mV)
         *
         * @param adcRawValue The 12-bit ADC raw word from REDRDY register
         * @return The corresponding voltage in mV
         */
        static inline int32_t rawToMilliVolts(int32_t adcRawValue) {
                if (adcRawValue >= 0)
                        return (adcRawValue * ADC_VREF_mV + ADC_MaximumRawValue / 2) / ADC_MaximumRawValue;

                return (adcRawValue * ADC_VREF_mV - ADC_MaximumRawValue / 2) / ADC_MaximumRawValue;
        }

        static_assert(ADC_MaximumRawValue * ADC_VREF_mV <= INT32_MAX);

        /**
         * Convert one ADC channel (raw) to phase current (mA) using the measured offset (raw)
         *
         * Assume bidirectional current sense centered at mid-supply
         *
         * @note The current equation is I_mA = (vSense_mV * 1000) / (Gain * R_shunt_mOhm)
         */
        static inline int32_t adcRawToCurrent(uint16_t adcRawValue, uint16_t adcOffsetRawValue) {
                const int32_t Raw = static_cast<int32_t>(adcRawValue) - static_cast<int32_t>(adcOffsetRawValue);
                const int32_t ShuntVoltage_mV = rawToMilliVolts(Raw);

                if (ShuntVoltage_mV >= 0)
                        return (ShuntVoltage_mV * 1000 + SenseGain * R_Shunt_mOhm / 2) / (SenseGain * R_Shunt_mOhm);

                return (ShuntVoltage_mV * 1000 - SenseGain * R_Shunt_mOhm / 2) / (SenseGain * R_Shunt_mOhm);
        }

        /**
         * Function that performs the initial zero-input offset calibration
         *
         * When motor is inactive and windings are de-energized, the positive input node of the op-amp is pulled down to GND through
         * the shunt resistor
         *
         * This is a typical process when using op-amps and MCU ADCs
         *
         * @note Make sure to stop the motor before running this function
         *
         * @see
         * https://onlinedocs.microchip.com/oxy/GUID-087A9847-6B26-452A-ABE4-5742B6A74CAF-en-US-1/GUID-F04AB974-02FC-49FB-B736-47235A36AE7B.html
         */
        void adcZeroOffsetCalibration();

        /**
         * The Brushless-DC motor object that contains the math of the FOC
         */
        PMSM_Controller motor;

        /**
         * The magnetic encoder (AS5047P) device driver object
         */
        AS5047P encoder;

        /**
         * The cached TCC periods that are written in the TCC0 register
         */
        uint32_t TCC_PeriodU = 0;
        uint32_t TCC_PeriodV = 0;
        uint32_t TCC_PeriodW = 0;

        /**
         * The duty cycles structure that is passed by reference as argument in the PMSM_Controller functions
         */
        PhaseDutyCycles dutyCycles{TCC_PeriodU, TCC_PeriodV, TCC_PeriodW};

        /**
         * The phase currents structure that is passed by reference as argument in the PMSM_Controller functions
         */
        PhaseCurrents currents{};

        /**
         * The ADC readings (raw values)
         */
        uint16_t adcResultU = 0;
        uint16_t adcResultV = 0;
        uint16_t adcResultW = 0;

        bool adcResultsReady = false;
        uint8_t adcScanIndex = 0;

        uint16_t adcOffsetU = 0;
        uint16_t adcOffsetV = 0;
        uint16_t adcOffsetW = 0;

        uint32_t offsetAccU = 0;
        uint32_t offsetAccV = 0;
        uint32_t offsetAccW = 0;
        uint16_t offsetCount = 0;
        bool offsetsReady = false;

        static constexpr int32_t R_Shunt_mOhm = 100;

        static constexpr int32_t SenseGain = 4;

        int32_t opAmpOffset_mV = 1'650;

        bool switchToCloseLoop = false;

        /**
         * Cached timing
         */
        float dT = 0.0f;
        bool encoderPrimed = false;
        /**
         * Cached PWM period duration in ticks (?) (according to SAMD21 TCC peripheral datasheet)
         */
        uint32_t pwmPeriodDuration = 0;
};

} // namespace PermanentMagnetSynchronousMotor
