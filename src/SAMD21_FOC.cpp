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
 * @file   SAMD21_FOC.cpp
 * @brief  SAMD21-specific FOC for BLDC
 * @author Georgios Gkogkou <ggkogkou125@gmail.com>
 */

#include "SAMD21_FOC.hpp"

namespace PermanentMagnetSynchronousMotor {

SAMD21_FOC::SAMD21_FOC(frequency_kHz_t pwmFrequencyKHz) : SAMD21_FOC(pwmFrequencyKHz, nullptr) {}

SAMD21_FOC::SAMD21_FOC(frequency_kHz_t pwmFrequencyKHz, TelemetryLogger* telemetry) :
    telemetryLogger(telemetry), motor(calculatePWM_PeriodFromFrequency(pwmFrequencyKHz), VelocityLoopPeriod_us, CurrentLoopPeriod_us),
    tccPeriod_PER(calculatePWM_PeriodFromFrequency(pwmFrequencyKHz)) {
        __disable_irq();

        TC3_TimerFrequencyHz = TC3_TimerFrequencyGet();
        TC3_TOP_RegisterValue = TC3_TimerFrequencyHz / VelocityLoopFrequencyHz - 1;
        TC3_Timer16bitPeriodSet(TC3_TOP_RegisterValue);

        TC4_TimerFrequencyHz = TC4_TimerFrequencyGet();
        TC4_TOP_RegisterValue = TC4_TimerFrequencyHz / CurrentLoopFrequencyHz - 1u;
        TC4_Timer16bitPeriodSet(TC4_TOP_RegisterValue);

        TCC0_PWM24bitPeriodSet(tccPeriod_PER);

        static constexpr uint32_t SampleOffsetTicks = 1;
        static constexpr bool SampleOnFallingRamp = false;
        uint32_t ccValue = SampleOffsetTicks & 0x7FFFFF;

        if (SampleOnFallingRamp)
                ccValue |= (1u << 23);

        TCC0_REGS->TCC_CC[3] = ccValue;

        while (TCC0_REGS->TCC_SYNCBUSY != 0U) {
                /* Wait for sync */
        }

        ADC_CallbackRegister(&SAMD21_FOC::ADC_Callback, reinterpret_cast<uintptr_t>(this));
        TC3_TimerCallbackRegister(&SAMD21_FOC::TC3_Callback, reinterpret_cast<uintptr_t>(this));
        TC4_TimerCallbackRegister(&SAMD21_FOC::TC4_Callback, reinterpret_cast<uintptr_t>(this));

        ADC_Enable();
        TCC0_PWMStart();
        TC3_TimerStart();
        TC4_TimerStart();

        setPWM_DutyCycles();

        __enable_irq();

        if (not AS5047P::sensorBusy()) {
                (void)encoder.request(AS5047P::RegisterAddress::ANGLECOM);
        }
}

void SAMD21_FOC::stop() const {
        __disable_irq();

        motor.stopMotor(dutyCycles);
        setPWM_DutyCycles();

        __enable_irq();
}

void SAMD21_FOC::moveToAngle(int32_t targetAngle_mrad, PMSM_Controller::PositionDirection direction, int32_t revolutions) {
        motor.setTargetPosition(targetAngle_mrad, direction, revolutions);
}

void SAMD21_FOC::ADC_Callback(ADC_STATUS status, uintptr_t context) {
        auto* self = reinterpret_cast<SAMD21_FOC*>(context);

        if (self)
                self->ADC_Callback(status);
}

void SAMD21_FOC::TC3_Callback(TC_TIMER_STATUS status, uintptr_t context) {
        auto* self = reinterpret_cast<SAMD21_FOC*>(context);

        if (self)
                self->TC3_FOC_Handler(status);
}

void SAMD21_FOC::TC4_Callback(TC_TIMER_STATUS status, uintptr_t context) {
        auto* self = reinterpret_cast<SAMD21_FOC*>(context);

        if (self)
                self->TC4_FOC_Handler(status);
}

void SAMD21_FOC::ADC_Callback(ADC_STATUS status) {
        if (status & ADC_INTFLAG_RESRDY_Msk) {
                // BENCHMARK_IO_Set();
                // BENCHMARK_IO_Clear();

                const uint16_t sample = ADC_ConversionResultGet();

                if (adcScanIndex == 0u)
                        adcResultU = sample;
                else
                        adcResultV = sample;

                adcScanIndex = (adcScanIndex + 1u) & 1u;

                if (adcScanIndex == 0u) {
                        adcU_latest = adcResultU;
                        adcV_latest = adcResultV;
                        adcPairSeq++;

                        adcResultsReady = true;
                }
        }

        if (status & ADC_INTFLAG_OVERRUN_Msk) {

                ADC_InterruptsClear(ADC_INTFLAG_OVERRUN_Msk);
        }
}

void SAMD21_FOC::TC3_FOC_Handler(TC_TIMER_STATUS status) {
        (void)status;

        const auto runClosedLoop = [&](uint16_t rotorPosition) {
                rotorPositionCached = rotorPosition;
                rotorPositionValid = true;

                if (++positionLoopDividerCounter >= PositionLoopDivider) {
                        positionLoopDividerCounter = 0;
                        motor.runPositionLoop(rotorPosition);
                }

                motor.runVelocityLoop(rotorPosition);

                if (not AS5047P::sensorBusy()) {
                        (void)encoder.request(AS5047P::RegisterAddress::ANGLECOM);
                }
        };

        switch (tc3State) {
        case TC3_State::PrimeEncoder:
                (void)encoder.request(AS5047P::RegisterAddress::ANGLECOM);
                tc3State = TC3_State::CalibrateOffsets;
                return;
        case TC3_State::CalibrateOffsets:
                if (not offsetsReady) {
                        motor.stopMotor(dutyCycles);
                        setPWM_DutyCycles();
                        adcZeroOffsetCalibration();
                        return;
                }
                tc3State = TC3_State::StartupCalibration;
                [[fallthrough]];
        case TC3_State::StartupCalibration:
                {
                        static constexpr uint16_t EncoderMask = 0x3FFF;
                        const auto rotorResult = encoder.measureAngleCompensatedRaw();
                        if (!rotorResult.has_value()) {
                                const auto error = rotorResult.error();
                                encoderErrorCode = static_cast<uint32_t>(error);
                                if (error == AS5047P::ReadError::PARITY_ERROR || error == AS5047P::ReadError::ERROR_FLAG_SET) {
                                        encoderFaulted = true;
                                        motor.stopMotor(dutyCycles);
                                        setPWM_DutyCycles();
                                        tc3State = TC3_State::Fault;
                                        return;
                                }
                        } else {
                                encoderErrorCode = 0;
                        }

                        const auto rotorPosition = rotorResult.has_value() ? static_cast<uint16_t>(rotorResult.value() & EncoderMask)
                                                                           : static_cast<uint16_t>(rotorPositionCached & EncoderMask);

                        if (not switchToCloseLoop) {
                                switchToCloseLoop = motor.startupCalibration(dutyCycles, rotorPosition);
                                setPWM_DutyCycles();
                                if (switchToCloseLoop)
                                        tc3State = TC3_State::ClosedLoop;
                                return;
                        }

                        tc3State = TC3_State::ClosedLoop;
                        runClosedLoop(rotorPosition);
                        return;
                }
        case TC3_State::ClosedLoop:
                {
                        static constexpr uint16_t EncoderMask = 0x3FFF;
                        const auto rotorResult = encoder.measureAngleCompensatedRaw();
                        if (not rotorResult.has_value()) {
                                const auto error = rotorResult.error();
                                encoderErrorCode = static_cast<uint32_t>(error);
                                if (error == AS5047P::ReadError::PARITY_ERROR || error == AS5047P::ReadError::ERROR_FLAG_SET) {
                                        encoderFaulted = true;
                                        motor.stopMotor(dutyCycles);
                                        setPWM_DutyCycles();
                                        tc3State = TC3_State::Fault;
                                        return;
                                }
                        } else {
                                encoderErrorCode = 0;
                        }

                        const auto rotorPosition = rotorResult.has_value() ? static_cast<uint16_t>(rotorResult.value() & EncoderMask)
                                                                           : static_cast<uint16_t>(rotorPositionCached & EncoderMask);

                        runClosedLoop(rotorPosition);
                        return;
                }
        case TC3_State::Fault:
        default:
                motor.stopMotor(dutyCycles);
                setPWM_DutyCycles();
                return;
        }
}

void SAMD21_FOC::TC4_FOC_Handler(TC_TIMER_STATUS status) {
        (void)status;

        if (not AS5047P::sensorBusy()) {
                (void)encoder.request(AS5047P::RegisterAddress::ANGLECOM);
        }

        switch (tc4State) {
        case TC4_State::Idle:
                if (not switchToCloseLoop || !offsetsReady || not rotorPositionValid)
                        return;

                tc4State = TC4_State::RunCurrentLoop;
                [[fallthrough]];
        case TC4_State::RunCurrentLoop:
                break;
        case TC4_State::Fault:
        default:
                motor.stopMotor(dutyCycles);
                setPWM_DutyCycles();
                return;
        }

        uint32_t seq;
        uint16_t U, V;

        NVIC_DisableIRQ(ADC_IRQn);
        seq = adcPairSeq;

        if (seq == lastUsedAdcPairSeq) {
                missedPairs++;
                NVIC_EnableIRQ(ADC_IRQn);
                return;
        }

        U = adcU_latest;
        V = adcV_latest;
        lastUsedAdcPairSeq = seq;
        NVIC_EnableIRQ(ADC_IRQn);

        static constexpr uint16_t EncoderMask = 0x3FFF;
        const auto rotorResult = encoder.measureAngleCompensatedRaw();
        if (not rotorResult.has_value()) {
                const auto error = rotorResult.error();
                encoderErrorCode = static_cast<uint32_t>(error);
                if (error == AS5047P::ReadError::PARITY_ERROR || error == AS5047P::ReadError::ERROR_FLAG_SET) {
                        encoderFaulted = true;
                        motor.stopMotor(dutyCycles);
                        setPWM_DutyCycles();
                        tc4State = TC4_State::Fault;
                        return;
                }
        } else {
                encoderErrorCode = 0;
        }

        const auto rotorPosition = rotorResult.has_value() ? static_cast<uint16_t>(rotorResult.value() & EncoderMask)
                                                           : static_cast<uint16_t>(rotorPositionCached & EncoderMask);
        rotorPositionCached = rotorPosition;

        currents.Ia_mA = adcRawToCurrent(U, adcOffsetU);
        currents.Ib_mA = adcRawToCurrent(V, adcOffsetV);
        currents.Ic_mA = -(currents.Ia_mA + currents.Ib_mA);

        motor.updateTelemetryHardware(adcPairSeq, missedPairs, U, V, adcOffsetU, adcOffsetV, encoderErrorCode);

        TelemetryLogger* t = telemetryLogger; // force logging every time

        if (telemetryLogger) {
                if (++telemetryDividerCounter >= TelemetryDivider) {
                        telemetryDividerCounter = 0;
                        t = telemetryLogger;
                }
        }

        motor.runCurrentLoop(currents, dutyCycles, rotorPosition, t);
        setPWM_DutyCycles();

        if (not AS5047P::sensorBusy()) {
                (void)encoder.request(AS5047P::RegisterAddress::ANGLECOM);
        }

        // BENCHMARK_IO_Clear();
}

void SAMD21_FOC::setPWM_DutyCycles() const {
        TCC0_PWM24bitDutySet(TCC0_CHANNEL0, TCC_PeriodU);
        TCC0_PWM24bitDutySet(TCC0_CHANNEL1, TCC_PeriodV);
        TCC0_PWM24bitDutySet(TCC0_CHANNEL2, TCC_PeriodW);
}

void SAMD21_FOC::adcZeroOffsetCalibration() {
        static constexpr uint16_t ADC_ScanCount = 1024;

        if (not adcResultsReady)
                return;

        NVIC_DisableIRQ(ADC_IRQn);
        const uint16_t U = adcResultU;
        const uint16_t V = adcResultV;
        adcResultsReady = false;
        NVIC_EnableIRQ(ADC_IRQn);

        if (offsetsReady)
                return;

        offsetAccU += U;
        offsetAccV += V;
        offsetCount++;

        if (offsetCount >= ADC_ScanCount) {
                adcOffsetU = static_cast<uint16_t>((offsetAccU + offsetCount / 2) / offsetCount);
                adcOffsetV = static_cast<uint16_t>((offsetAccV + offsetCount / 2) / offsetCount);

                offsetsReady = true;

                offsetAccU = 0;
                offsetAccV = 0;
                offsetCount = 0;
        }
}

} // namespace PermanentMagnetSynchronousMotor
