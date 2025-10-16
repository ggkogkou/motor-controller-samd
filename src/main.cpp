#include <cstdlib>
#include "definitions.h"
#include "logger.h"
#include "as5047p.hpp"
#include "svpwm.hpp"
#include "math_utils.hpp"

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
 * PWM period ISR: sweep angle at fixed magnitude
 * Keep ISR minimal: compute duties and write CC
 */
void TCC_PeriodEventHandler(uint32_t status, uintptr_t context) {

    // TCC0_PWM24bitDutySet(TCC0_CHANNEL0, duties.dutyA);
    // TCC0_PWM24bitDutySet(TCC0_CHANNEL1, duties.dutyB);
    // TCC0_PWM24bitDutySet(TCC0_CHANNEL2, duties.dutyC);

}

/**
 * This function is called after TCC period event
 */
// void TCC_PeriodEventHandler(uint32_t status, uintptr_t context) {
//     static std::uint32_t duty0 = 800U;
//     static std::uint32_t duty1 = 800U;
//     static std::uint32_t duty2 = 1600U;
//
//     TCC0_PWM24bitDutySet(TCC0_CHANNEL0, duty0);
//     TCC0_PWM24bitDutySet(TCC0_CHANNEL1, duty1);
//     TCC0_PWM24bitDutySet(TCC0_CHANNEL2, duty2);
// }

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
    Logger_Initialize();

    TC4_CaptureStart();
    TC4_CaptureCallbackRegister(capture_handler, (uintptr_t)NULL);

    /* Register callback function for period event */
    TCC0_PWMCallbackRegister(TCC_PeriodEventHandler, (uintptr_t)NULL);

    /* Read the period */
    period = TCC0_PWM24bitPeriodGet();
    SYSTICK_DelayMs(100);
    Logger_Info("PWM period configured\r\n");

    /* Start PWM*/
    // TCC0_PWMStart();
    SYSTICK_DelayMs(100);
    Logger_Info("PWM started\r\n");

    AS5047P as5047p;

    while ( true )     {
        // Keep application alive; ISR updates PWM each period
        SYSTICK_DelayMs(500);
        Logger_Info("Running...\r\n");
    }

}
