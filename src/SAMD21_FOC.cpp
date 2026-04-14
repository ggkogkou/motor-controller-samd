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

constinit ApplicationCriticalVariables appCriticalVariables1{
        .focState = FOC_State::PRIME_ENCODER,
        .switchToCloseLoop = false,
        .encoderFaulted = false,
        .encoderErrorCode = 0,
        .adcOffsetU = 0,
        .adcOffsetV = 0,
        .offsetsReady = false,
        .positionLoopDivider = 1,
};

constinit ApplicationCriticalVariables appCriticalVariables2{
        .focState = FOC_State::PRIME_ENCODER,
        .switchToCloseLoop = false,
        .encoderFaulted = false,
        .encoderErrorCode = 0,
        .adcOffsetU = 0,
        .adcOffsetV = 0,
        .offsetsReady = false,
        .positionLoopDivider = 1,
};

constinit ApplicationCriticalVariables appCriticalVariables3{
        .focState = FOC_State::PRIME_ENCODER,
        .switchToCloseLoop = false,
        .encoderFaulted = false,
        .encoderErrorCode = 0,
        .adcOffsetU = 0,
        .adcOffsetV = 0,
        .offsetsReady = false,
        .positionLoopDivider = 1,
};

SAMD21_FOC::SAMD21_FOC(frequency_kHz_t pwmFrequencyKHz) : SAMD21_FOC(pwmFrequencyKHz, nullptr) {}

SAMD21_FOC::SAMD21_FOC(frequency_kHz_t pwmFrequencyKHz, TelemetryLogger<TelemetryPayload12>* telemetry) :
    telemetryLogger(telemetry), motor(calculatePWM_PeriodFromFrequency(pwmFrequencyKHz), velocityLoopPeriodUsFromPwm(pwmFrequencyKHz),
                                      currentLoopPeriodUsFromPwm(pwmFrequencyKHz)),
    adcOffsetU(appCriticalVariables1.adcOffsetU, appCriticalVariables2.adcOffsetU, appCriticalVariables3.adcOffsetU),
    adcOffsetV(appCriticalVariables1.adcOffsetV, appCriticalVariables2.adcOffsetV, appCriticalVariables3.adcOffsetV),
    offsetsReady(appCriticalVariables1.offsetsReady, appCriticalVariables2.offsetsReady, appCriticalVariables3.offsetsReady),
    focState(appCriticalVariables1.focState, appCriticalVariables2.focState, appCriticalVariables3.focState),
    switchToCloseLoop(appCriticalVariables1.switchToCloseLoop, appCriticalVariables2.switchToCloseLoop,
                      appCriticalVariables3.switchToCloseLoop),
    encoderFaulted(appCriticalVariables1.encoderFaulted, appCriticalVariables2.encoderFaulted, appCriticalVariables3.encoderFaulted),
    encoderErrorCode(appCriticalVariables1.encoderErrorCode, appCriticalVariables2.encoderErrorCode,
                     appCriticalVariables3.encoderErrorCode),
    positionLoopDivider(appCriticalVariables1.positionLoopDivider, appCriticalVariables2.positionLoopDivider,
                        appCriticalVariables3.positionLoopDivider),
    tccPeriod_PER(calculatePWM_PeriodFromFrequency(pwmFrequencyKHz)) {
        __disable_irq();

        const uint32_t velocityLoopFrequencyHz = velocityLoopFrequencyHzFromPwm(pwmFrequencyKHz);
        positionLoopDivider.write((velocityLoopFrequencyHz + PositionLoopFrequencyHz / 2) / PositionLoopFrequencyHz);
        if (positionLoopDivider.read() == 0u)
                positionLoopDivider.write(1u);

        positionLoopDividerCounter = 0u;

        adcOffsetU.write(0u);
        adcOffsetV.write(0u);
        offsetsReady.write(false);
        focState.write(FOC_State::PRIME_ENCODER);
        switchToCloseLoop.write(false);
        encoderFaulted.write(false);
        encoderErrorCode.write(0u);

        TC3_TimerFrequencyHz = TC3_TimerFrequencyGet();
        TC3_TOP_RegisterValue = TC3_TimerFrequencyHz / velocityLoopFrequencyHz - 1;
        TC3_Timer16bitPeriodSet(TC3_TOP_RegisterValue);

        TCC0_PWM24bitPeriodSet(tccPeriod_PER);

        static constexpr uint32_t SampleOffsetTicksFromPeriodEnd = 40;
        const uint32_t sampleTicks =
                (tccPeriod_PER > SampleOffsetTicksFromPeriodEnd) ? (tccPeriod_PER - SampleOffsetTicksFromPeriodEnd) : 0U;
        const uint32_t ccValue = sampleTicks & 0xFFFFFFU;

        TCC0_REGS->TCC_CC[3] = ccValue;

        while (TCC0_REGS->TCC_SYNCBUSY != 0U) {
                /* Wait for sync */
        }

        ADC_CallbackRegister(&SAMD21_FOC::ADC_Callback, reinterpret_cast<uintptr_t>(this));
        TC3_TimerCallbackRegister(&SAMD21_FOC::TC3_Callback, reinterpret_cast<uintptr_t>(this));

        ADC_Enable();
        TCC0_PWMStart();
        TC3_TimerStart();

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

void __attribute__((section(".ramfunc"))) SAMD21_FOC::ADC_Callback(ADC_STATUS status) {
        if (status & ADC_INTFLAG_RESRDY_Msk) {
                const uint16_t Sample = ADC_ConversionResultGet();

                adcResult[adcScanIndex] = Sample;
                adcScanIndex ^= 1u;

                if (adcScanIndex != 0) {
                        ADC_ConversionStart();
                        return;
                }

                adcResultsReady = true;

                if (focState.read() == FOC_State::FAULT_DETECTED || not switchToCloseLoop.read() || not offsetsReady.read() ||
                    not rotorPositionValid)
                        return;

                currents.Ia_mA = adcRawToCurrent(adcResult[PhaseIndexU], adcOffsetU.read());
                currents.Ib_mA = adcRawToCurrent(adcResult[PhaseIndexV], adcOffsetV.read());

                motor.updateTelemetryHardware(0, missedPairs, adcResult[PhaseIndexU], adcResult[PhaseIndexV], adcOffsetU.read(),
                                              adcOffsetV.read(), encoderErrorCode.read());

                motor.runCurrentLoop(currents, dutyCycles, rotorPositionCached);
                setPWM_DutyCycles();
                // BENCHMARK_IO_Clear();
        }

        if (status & ADC_INTFLAG_OVERRUN_Msk)
                ADC_InterruptsClear(ADC_INTFLAG_OVERRUN_Msk);
}

void __attribute__((section(".ramfunc"))) SAMD21_FOC::TC3_FOC_Handler(TC_TIMER_STATUS status) {
        (void)status;

        const auto updateEncoder = [&](uint16_t& rotorPosition) -> bool {
                static constexpr uint16_t EncoderMask = 0x3FFF;

                const auto RotorResult = encoder.measureAngleCompensatedRaw();
                auto angle = static_cast<uint16_t>(rotorPositionCached & EncoderMask);

                if (not RotorResult.has_value()) {
                        const auto EncoderError = RotorResult.error();
                        encoderErrorCode.write(static_cast<uint32_t>(EncoderError));
                        if (EncoderError == AS5047P::ReadError::PARITY_ERROR || EncoderError == AS5047P::ReadError::ERROR_FLAG_SET) {
                                encoderFaulted.write(true);
                                motor.stopMotor(dutyCycles);
                                setPWM_DutyCycles();
                                focState.write(FOC_State::FAULT_DETECTED);
                                HardwareDiagnostics::diagnosticsLogger.writeLiteral("ENCODER FAUL DETECTED\r\n");
                                return false;
                        }
                } else {
                        encoderErrorCode.write(0u);
                        angle = static_cast<uint16_t>(RotorResult.value() & EncoderMask);
                        rotorPositionCached = angle;
                        rotorPositionValid = true;
                }

                rotorPosition = angle;

                return true;
        };

        if (focState.read() == FOC_State::CLOSED_LOOP) {
                uint16_t rotorPosition = 0;

                if (updateEncoder(rotorPosition)) {
                        motor.updateEncoderErrorCode(encoderErrorCode.read());

                        const uint32_t nextDividerCounter = positionLoopDividerCounter + 1u;
                        positionLoopDividerCounter = nextDividerCounter;

                        if (positionLoopDividerCounter >= positionLoopDivider.read()) {
                                const auto ALU_HealthCheckPrevious = aluHealthCheckCounter;
                                aluHealthCheckCounter = aluHealthCheckCounter + 1;

                                if (aluHealthCheckCounter != static_cast<uint32_t>(ALU_HealthCheckPrevious + 1))
                                        NVIC_SystemReset();

                                positionLoopDividerCounter = 0u;
                                motor.runPositionLoop(rotorPosition);
                        }

                        motor.runVelocityLoop(rotorPosition);

                        if (telemetryLogger) {
                                TelemetryPayload12 tp{};
                                motor.fillTelemetryPayload(tp, rotorPosition);
                                telemetryLogger->updateLatest(tp);
                        }
                }

                if (not AS5047P::sensorBusy())
                        (void)encoder.request(AS5047P::RegisterAddress::ANGLECOM);

                return;
        }

        switch (focState.read()) {
        case FOC_State::PRIME_ENCODER:
                (void)encoder.request(AS5047P::RegisterAddress::ANGLECOM);
                focState.write(FOC_State::CALIBRATE_ADC_ZERO_OFFSETS);
                break;
        case FOC_State::CALIBRATE_ADC_ZERO_OFFSETS:
                if (not offsetsReady.read()) {
                        motor.stopMotor(dutyCycles);
                        setPWM_DutyCycles();
                        adcZeroOffsetCalibration();
                        break;
                }
                focState.write(FOC_State::STARTUP_CALIBRATIONS);
                [[fallthrough]];
        case FOC_State::STARTUP_CALIBRATIONS:
                {
                        uint16_t rotorPosition = 0;
                        if (not updateEncoder(rotorPosition))
                                break;

                        if (not switchToCloseLoop.read()) {
                                switchToCloseLoop.write(motor.startupCalibration(dutyCycles, rotorPosition));
                                setPWM_DutyCycles();
                                if (switchToCloseLoop.read())
                                        focState.write(FOC_State::CLOSED_LOOP);
                                break;
                        }

                        focState.write(FOC_State::CLOSED_LOOP);
                        break;
                }
        case FOC_State::FAULT_DETECTED:
        default:
                motor.stopMotor(dutyCycles);
                setPWM_DutyCycles();
                break;
        }

        if (not AS5047P::sensorBusy())
                (void)encoder.request(AS5047P::RegisterAddress::ANGLECOM);
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

        const uint16_t U = adcResult[PhaseIndexU];
        const uint16_t V = adcResult[PhaseIndexV];
        adcResultsReady = false;

        if (offsetsReady.read())
                return;

        offsetAccU += U;
        offsetAccV += V;
        offsetCount++;

        if (offsetCount >= ADC_ScanCount) {
                adcOffsetU.write(static_cast<uint16_t>((offsetAccU + offsetCount / 2) / offsetCount));
                adcOffsetV.write(static_cast<uint16_t>((offsetAccV + offsetCount / 2) / offsetCount));

                offsetsReady.write(true);

                offsetAccU = 0;
                offsetAccV = 0;
                offsetCount = 0;
        }
}

} // namespace PermanentMagnetSynchronousMotor
