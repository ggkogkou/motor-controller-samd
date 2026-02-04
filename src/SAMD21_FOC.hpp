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

using microseconds_t = uint32_t;
using frequency_kHz_t = float;
using counter_ticks_t = uint32_t;

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
        explicit SAMD21_FOC(frequency_kHz_t pwmFrequencyKHz);

        explicit SAMD21_FOC(frequency_kHz_t pwmFrequencyKHz, TelemetryLogger* telemetry = nullptr);

        /**
         * Compiler generated default destructor
         */
        ~SAMD21_FOC() = default;

        /**
         * Function that stops the motor from spinning by applying a zero-vector to the 3-phase inverter
         */
        void stop() const;

        /**
         * Command a target position in milliradians [0, 2π)
         *
         * @param targetAngle_mrad Target angle in milliradians
         * @param direction Path direction (CW, CCW, or SHORTEST)
         * @param revolutions Full turns to add in the given direction
         */
        void moveToAngle(int32_t targetAngle_mrad,
                         PMSM_Controller::PositionDirection direction = PMSM_Controller::PositionDirection::CCW,
                         int32_t revolutions = 0);

        /**
         * Encoder error monitoring (poll outside ISRs).
         */
        [[nodiscard]] etl::expected<void, AS5047P::ReadError> encoderLastReadResult() const {
                return encoder.lastReadResult();
        }

        /**
         * Last ERRFL snapshot (valid after ERROR_FLAG_SET).
         */
        [[nodiscard]] AS5047P::ERRFL_Status encoderLastErrflStatus() const {
                return encoder.lastErrflStatus();
        }

private:
        static void ADC_Callback(ADC_STATUS status, uintptr_t context);
        static void TC3_Callback(TC_TIMER_STATUS status, uintptr_t context);
        static void TC4_Callback(TC_TIMER_STATUS status, uintptr_t context);

        void ADC_Callback(ADC_STATUS status);
        void TC3_FOC_Handler(TC_TIMER_STATUS status);
        void TC4_FOC_Handler(TC_TIMER_STATUS status);

        /**
         * Function that writes the cached PWM periods to the TCC0 registers
         */
        void setPWM_DutyCycles() const;

        TelemetryLogger* telemetryLogger = nullptr;

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
         * @param adcRawValue The 12-bit ADC raw word from RESRDY register
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
        volatile uint16_t adcResultU = 0;
        volatile uint16_t adcResultV = 0;
        volatile uint16_t adcResultW = 0;

        volatile bool adcResultsReady = false;
        volatile uint8_t adcScanIndex = 0;

        uint16_t adcOffsetU = 0;
        uint16_t adcOffsetV = 0;
        uint16_t adcOffsetW = 0;

        uint32_t offsetAccU = 0;
        uint32_t offsetAccV = 0;
        uint32_t offsetAccW = 0;
        uint16_t offsetCount = 0;
        bool offsetsReady = false;

        static constexpr int32_t R_Shunt_mOhm = 100;

        static constexpr int32_t SenseGain = 20;

        int32_t opAmpOffset_mV = 1'650;

        enum class TC3_State : uint8_t {
                PrimeEncoder,
                CalibrateOffsets,
                StartupCalibration,
                ClosedLoop,
                Fault,
        };

        enum class TC4_State : uint8_t {
                Idle,
                RunCurrentLoop,
                Fault,
        };

        TC3_State tc3State = TC3_State::PrimeEncoder;
        TC4_State tc4State = TC4_State::Idle;

        bool switchToCloseLoop = false;
        bool encoderFaulted = false;
        uint32_t encoderErrorCode = 0;

        /**
         * Cached timing
         */
        float dT = 0.0f;


        /**
         * @enum TC_InputClockPrescaler
         *
         * A list of available prescalers for the TC peripheral input clock
         */
        enum class TC_InputClockPrescaler : uint16_t {
                DIV_1 = 1,
                DIV_2 = 2,
                DIV_4 = 4,
                DIV_16 = 16,
                DIV_64 = 64,
        };

        /**
         * The input clock of the TC3 peripheral
         */
        static constexpr float TC3_InputClockPrescaler = static_cast<float>(TC_InputClockPrescaler::DIV_64);

        /**
         * The input clock of the TC3 peripheral in KHz
         */
        static constexpr float TC3_InputClock_KHz = 48'000;

        /**
         * The Timer count frequency
         */
        static constexpr float TC3_TimerFrequency_KHz = TC3_InputClock_KHz / TC3_InputClockPrescaler;

        static constexpr uint16_t TC3_TimerFrequency = static_cast<uint16_t>(TC3_TimerFrequency_KHz);

        static constexpr float VelocityLoopFrequencyKHz = 3.0f;

        static constexpr uint32_t VelocityLoopFrequencyHz = static_cast<uint32_t>(VelocityLoopFrequencyKHz * 1000.0f);

        uint32_t TC3_TimerFrequencyHz = 0;

        uint32_t TC3_TOP_RegisterValue = 0;

        static constexpr uint32_t VelocityLoopPeriod_us = 1'000'000 / VelocityLoopFrequencyHz;

        /**
         * TC4 (Current loop) timings
         */
        static constexpr float CurrentLoopFrequencyKHz = 16.0f;
        static constexpr uint32_t CurrentLoopFrequencyHz = static_cast<uint32_t>(CurrentLoopFrequencyKHz * 1000.0f);
        static constexpr uint32_t CurrentLoopPeriod_us = 1'000'000u / CurrentLoopFrequencyHz;

        uint32_t TC4_TimerFrequencyHz = 0;
        uint32_t TC4_TOP_RegisterValue = 0;

        static constexpr uint32_t TelemetryHz = 200;
        static constexpr uint32_t TelemetryDivider = (CurrentLoopFrequencyHz + TelemetryHz / 2) / TelemetryHz;

        static_assert(TelemetryDivider >= 1);

        uint32_t telemetryDividerCounter = 0;

        /**
         * Cache rotor position for the current loop
         */
        volatile uint16_t rotorPositionCached = 0;
        volatile bool rotorPositionValid = false;

        volatile uint16_t adcU_latest = 0;
        volatile uint16_t adcV_latest = 0;
        uint32_t adcPairSeq = 0;
        uint32_t lastUsedAdcPairSeq = 0;

        uint32_t missedPairs = 0;

        /**
         * Position loop
         */
        static constexpr uint32_t PositionLoopFrequencyHz = 1'000;
        static constexpr uint32_t PositionLoopDivider =
                (VelocityLoopFrequencyHz + PositionLoopFrequencyHz / 2) / PositionLoopFrequencyHz;

        static_assert(PositionLoopDivider >= 1);

        uint32_t positionLoopDividerCounter = 0;

        /**
         * The period of the PWM driving the 3-phase inverters, just a default value
         */
        static constexpr uint32_t PWM_Period_us = 1000;

        /**
         * The prescaler for the clock of the TCC peripheral; used to calculate the period
         */
        static constexpr uint16_t TCC_ClockPrescaler_N = 1;

        /**
         * The input clock of the TCC0 peripheral
         */
        static constexpr uint16_t TCC_InputClock_MHz = 48;

        /**
         * Cached PWM period duration in timer counts
         *
         * The PWM period for dual-slope PWM generation is given by the formula:
         * f(pwm_ds) = f(gclk_tcc) / (2*N*PER), where PER is the * value to write in the corresponding register TCC0_REGS->TCC_PER
         *
         * Assuming that the TCC0 is clocked by the main 48MHz clock and the prescaler is N, then if Tpwm is the period in μs, the
         * value written in PER is PER = 48 / (2 * N) * Tpwm
         */
        uint32_t tccPeriod_PER = 1'000;

        /**
         * Helper function that converts the period (μs) to a value appropriate for the PER register of the TCC peripheral
         *
         * @param pwmPeriod_us The PWM period in μs
         */
        [[nodiscard]] static __attribute__((always_inline)) counter_ticks_t microsecondsToTimerTicks(microseconds_t pwmPeriod_us) {
                return TCC_InputClock_MHz / (2 * TCC_ClockPrescaler_N) * pwmPeriod_us;
        }

        /**
         * Calculate the period to the PER register of the TCC peripheral * * @param pwmPeriod_us The PWM period in μs
         */
        __attribute__((always_inline)) void setPWM_Period_us(microseconds_t pwmPeriod_us) {
                tccPeriod_PER = microsecondsToTimerTicks(pwmPeriod_us);
                TCC0_PWM24bitPeriodSet(tccPeriod_PER);
        }

        /**
         * Calculate the period to the PER register of the TCC peripheral
         *
         * @note Floating-point calculations are not a problem here, it's an operation performed at startup, not part of the hot-path
         *
         * @param pwmFrequency_khz The PWM frequency in kHz
         */
        [[nodiscard]] static __attribute__((always_inline)) counter_ticks_t
        calculatePWM_PeriodFromFrequency(frequency_kHz_t pwmFrequency_khz) {
                const auto PER_Value = std::lroundf(static_cast<float>(TCC_InputClock_MHz) /
                                                    (2.0f * static_cast<float>(TCC_ClockPrescaler_N)) / pwmFrequency_khz * 1000.0f);

                static_assert(sizeof(long) <= sizeof(PER_Value), "PER_Value is too big to fit in a long");

                return static_cast<counter_ticks_t>(PER_Value);
        }

        /**
         * Set the period to the PER register of the TCC peripheral
         *
         * @param pwmFrequency_khz The PWM frequency in kHz
         */
        __attribute__((always_inline)) void setPWM_Frequency(float pwmFrequency_khz) {
                tccPeriod_PER = calculatePWM_PeriodFromFrequency(pwmFrequency_khz);
                TCC0_PWM24bitPeriodSet(tccPeriod_PER);
        }
};

} // namespace PermanentMagnetSynchronousMotor
