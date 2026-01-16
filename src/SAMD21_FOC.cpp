#include "SAMD21_FOC.hpp"

namespace PermanentMagnetSynchronousMotor {

SAMD21_FOC::SAMD21_FOC() {
        __disable_irq();

        const uint32_t f_tc = TC3_TimerFrequencyGet();
        const uint32_t top = TC3_Timer16bitPeriodGet();
        dT = (1.0f + static_cast<float>(top)) / static_cast<float>(f_tc);
        pwmPeriodDuration = TCC0_PWM24bitPeriodGet();

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
                        adcResultU = sample;
                } else if (adcScanIndex == 7u) {
                        adcResultV = sample;
                } else if (adcScanIndex == 8u) {
                        adcResultW = sample;
                }

                adcScanIndex = (adcScanIndex + 1u) % 9u;

                if (adcScanIndex == 0u) {
                        adcResultsReady = true;
                }
        }

        if (status & ADC_INTFLAG_OVERRUN_Msk) {
                ADC_InterruptsClear(ADC_INTFLAG_OVERRUN_Msk);
        }
}

void SAMD21_FOC::TC3_FOC_Handler(TC_TIMER_STATUS status) {
        (void)status;

        if (!encoderPrimed) {
                (void)encoder.request(AS5047P::RegisterAddress::ANGLECOM);
                encoderPrimed = true;
                return;
        }

        const auto rotorPosition = static_cast<uint16_t>(encoder.measureAngleCompensatedRaw() & 0x3FFFu);

        if (not AS5047P::sensorBusy()) {
                (void)encoder.request(AS5047P::RegisterAddress::ANGLECOM);
        }

        if (!switchToCloseLoop) {
                switchToCloseLoop = motor.startupCalibration(dutyCycles, rotorPosition);
                setPWM_DutyCycles();
                return;
        }

        motor.updateVelocity(currents, dutyCycles, rotorPosition);
        setPWM_DutyCycles();
}

void SAMD21_FOC::setPWM_DutyCycles() const {
        TCC0_PWM24bitDutySet(TCC0_CHANNEL0, TCC_PeriodU);
        TCC0_PWM24bitDutySet(TCC0_CHANNEL1, TCC_PeriodV);
        TCC0_PWM24bitDutySet(TCC0_CHANNEL2, TCC_PeriodW);
}

} // namespace PermanentMagnetSynchronousMotor
