#pragma once

#include <cstdint>
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
    static std::uint32_t duty0 = 0U;
    static std::uint32_t duty1 = 800U;
    static std::uint32_t duty2 = 1600U;

    TCC0_PWM24bitDutySet(TCC0_CHANNEL0, duty0);
    TCC0_PWM24bitDutySet(TCC0_CHANNEL1, duty1);
    TCC0_PWM24bitDutySet(TCC0_CHANNEL2, duty2);

    duty0 += DUTY_INCREMENT;
    duty1 += DUTY_INCREMENT;
    duty2 += DUTY_INCREMENT;

    if (duty0 > period)
        duty0 = 0U;
    if (duty1 > period)
        duty1 = 0U;
    if (duty2 > period)
        duty2 = 0U;
}

[[noreturn]] int main ( ) {
    /* Initialize all modules */
    SYS_Initialize ( nullptr);

    SYSTICK_TimerStart();

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

        SYSTICK_DelayMs(500);

        as5047p.readDeviceRegister(AS5047P::RegisterAddress::DIAAGC);
        auto angle = as5047p.measureAngleUncompensated();

        Logger_Info("Running...\r\n");
    }

}
