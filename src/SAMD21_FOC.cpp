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
    telemetryLogger(telemetry), motor(calculatePWM_PeriodFromFrequency(pwmFrequencyKHz), VelocityLoopPeriod_us),
    tccPeriod_PER(calculatePWM_PeriodFromFrequency(pwmFrequencyKHz)) {
        __disable_irq();

        TC3_TimerFrequencyHz = TC3_TimerFrequencyGet();

        TC3_TOP_RegisterValue = TC3_TimerFrequencyHz / VelocityLoopFrequencyHz - 1;
        TC3_Timer16bitPeriodSet(TC3_TOP_RegisterValue);

        TCC0_PWM24bitPeriodSet(tccPeriod_PER);

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

void SAMD21_FOC::ADC_Callback(ADC_STATUS status) {
        if (status & ADC_INTFLAG_RESRDY_Msk) {
                const uint16_t sample = ADC_ConversionResultGet();

                if (adcScanIndex == 0u) {
                        adcResultU = sample; // AIN10
                } else { // adcScanIndex == 1
                        adcResultV = sample; // AIN11
                }

                adcScanIndex = (adcScanIndex + 1u) % 2u;

                if (adcScanIndex == 0u) {
                        adcResultsReady = true;
                }
        }

        if (status & ADC_INTFLAG_OVERRUN_Msk) {
                adcResultU = 0;
                adcResultV = 0;
                ADC_InterruptsClear(ADC_INTFLAG_OVERRUN_Msk);
        }
}

void SAMD21_FOC::TC3_FOC_Handler(TC_TIMER_STATUS status) {
        (void)status;

        if (not encoderPrimed) {
                (void)encoder.request(AS5047P::RegisterAddress::ANGLECOM);
                encoderPrimed = true;
                return;
        }

        if (not offsetsReady) {
                motor.stopMotor(dutyCycles);
                setPWM_DutyCycles();
                adcZeroOffsetCalibration();
                return;
        }

        static constexpr uint16_t EncoderMask = 0x3FFF;

        const auto rotorPosition = static_cast<uint16_t>(encoder.measureAngleCompensatedRaw() & EncoderMask);

        if (not AS5047P::sensorBusy()) {
                (void)encoder.request(AS5047P::RegisterAddress::ANGLECOM);
        }

        if (not switchToCloseLoop) {
                switchToCloseLoop = motor.startupCalibration(dutyCycles, rotorPosition);
                setPWM_DutyCycles();
                return;
        }

        if (adcResultsReady) {
                NVIC_DisableIRQ(ADC_IRQn);
                const uint16_t U = adcResultU;
                const uint16_t V = adcResultV;
                adcResultsReady = false;
                NVIC_EnableIRQ(ADC_IRQn);

                currents.Ia_mA = adcRawToCurrent(U, adcOffsetU);
                currents.Ib_mA = adcRawToCurrent(V, adcOffsetV);
                currents.Ic_mA = -(currents.Ia_mA + currents.Ib_mA);
        }

        TelemetryLogger* t = nullptr;

        static uint32_t telemetryDivider = 0;

        if (telemetryLogger) {
                telemetryDivider++;
                if (telemetryDivider >= 50) {
                        telemetryDivider = 0;
                        t = telemetryLogger;
                }
        }

        motor.updateVelocity(currents, dutyCycles, rotorPosition, t);
        setPWM_DutyCycles();
        // stop();
}

void SAMD21_FOC::setPWM_DutyCycles() const {
        TCC0_PWM24bitDutySet(TCC0_CHANNEL0, TCC_PeriodU);
        TCC0_PWM24bitDutySet(TCC0_CHANNEL1, TCC_PeriodV);
        TCC0_PWM24bitDutySet(TCC0_CHANNEL2, TCC_PeriodW);
}

void SAMD21_FOC::adcZeroOffsetCalibration() {
        static constexpr uint16_t ADC_ScanCount = 512;

        if (!adcResultsReady)
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
