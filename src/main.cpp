#pragma once

#include <cstdlib>
#include "definitions.h"
#include "logger.h"
#include "as5047p.hpp"

/***************************************
 * Check PWM outputs on pins
 * Channel 0 PWMH - PA08
 * Channel 0 PWML - PB10
 * Channel 1 PWMH - PA09
 * Channel 1 PWML - PB11
 * Channel 2 PWMH - PA10
 * Channel 2 PWML - PB12
***************************************/

/**
 * Duty cycle increment value
 */
inline constexpr std::uint32_t DUTY_INCREMENT = 10;

/**
 * Save PWM period
 */
static std::uint32_t period;

/**
 * This function is called after TCC period event
 */
void TCC_PeriodEventHandler(uint32_t status, uintptr_t context) {
    static std::uint32_t duty0 = 800U;
    static std::uint32_t duty1 = 800U;
    static std::uint32_t duty2 = 1600U;

    TCC0_PWM24bitDutySet(TCC0_CHANNEL0, duty0);
    TCC0_PWM24bitDutySet(TCC0_CHANNEL1, duty1);
    TCC0_PWM24bitDutySet(TCC0_CHANNEL2, duty2);
}

volatile bool tc_buffer_ready = false;

void capture_handler( TC_CAPTURE_STATUS status, uintptr_t context) {
    if ((status  & TC_CAPTURE_STATUS_CAPTURE0_READY) == TC_CAPTURE_STATUS_CAPTURE0_READY)
    {
        tc_buffer_ready = true;
    }
}

[[noreturn]] int main ( ) {
    /* Initialize all modules */
    SYS_Initialize ( nullptr);

    SYSTICK_TimerStart();

    TC4_CaptureStart();

    TC4_CaptureCallbackRegister(capture_handler, (uintptr_t)NULL);

    Logger_Initialize();

    /* Register callback function for period event */
    TCC0_PWMCallbackRegister(TCC_PeriodEventHandler, (uintptr_t)NULL);

    /* Read the period */
    period = TCC0_PWM24bitPeriodGet();
    Logger_Info("PWM period configured\r\n");

    /* Start PWM*/
    TCC0_PWMStart();
    Logger_Info("PWM started\r\n");

    AS5047P as5047p;

    SYSTICK_DelayMs(500);

    while ( true )     {
        /* Maintain state machines of all polled MPLAB Harmony modules. */
        // SYS_Tasks ( );

        const uint32_t pwm_full_period = TC4_Capture16bitChannel0Get();
        const uint32_t pwm_on_time = TC4_Capture16bitChannel1Get();

        while(tc_buffer_ready != true)
        {}
        // auto angle = as5047p.measureAngleUncompensated();

        uint32_t duty = ((pwm_on_time) * 100U) / pwm_full_period;
        uint32_t frequency = (TC4_CaptureFrequencyGet() / pwm_full_period);

        SYSTICK_DelayMs(500);

        Logger_Info("Running...\r\n");
    }

}
