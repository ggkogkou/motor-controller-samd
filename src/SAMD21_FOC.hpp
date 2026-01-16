#pragma once

#include <cstdint>
#include "as5047p.hpp"
#include "definitions.h"
#include "pmsm_controller.hpp"

namespace PermanentMagnetSynchronousMotor {

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
