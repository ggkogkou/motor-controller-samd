#pragma once

#include <cstdlib>
#include "definitions.h"
#include "logger.h"
#include "as5047p.hpp"
#include "svpwm.hpp"

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

// SVPWM instance and test vector params (volatile if updated in main)
static ZeroSequenceModulation::SVPWM g_svm;
static volatile float g_magnitude = 0.30f;   // 0..~0.9 (normalized)
static volatile float g_theta     = 0.0f;    // angle [rad]
static volatile float g_dtheta    = 0.002f;  // per-period increment

/**
 * PWM period ISR: sweep angle at fixed magnitude
 * Keep ISR minimal: compute duties and write CC
 */
void TCC_PeriodEventHandler(uint32_t status, uintptr_t context) {
    // Compute alpha-beta from current angle and magnitude
    const float c = cosf(g_theta);
    const float s = sinf(g_theta);
    const float v_alpha = g_magnitude * c;
    const float v_beta  = g_magnitude * s;

    // Compute duties
    const auto duties = g_svm.computeFromAB(v_alpha, v_beta);

    // Apply to PWM channels
    TCC0_PWM24bitDutySet(TCC0_CHANNEL0, duties.dutyA);
    TCC0_PWM24bitDutySet(TCC0_CHANNEL1, duties.dutyB);
    TCC0_PWM24bitDutySet(TCC0_CHANNEL2, duties.dutyC);

    // Advance angle
    g_theta += g_dtheta;
    if (g_theta >= 6.28318531f) g_theta -= 6.28318531f;
    if (g_theta < 0.0f)         g_theta += 6.28318531f;
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

    // Initialize SVPWM with timer period ticks and normalized bus (vbus=1.0f)
    g_svm.setPeriod(period);
    g_svm.setVbus(1.0f);
    g_svm.setOnTimeClamp(0U, period);
    g_magnitude = 0.30f;  // visible modulation
    g_theta = 0.0f;
    g_dtheta = 0.002f;    // slow rotation; increase for faster sweep


    /* Start PWM*/
    TCC0_PWMStart();
    SYSTICK_DelayMs(100);
    Logger_Info("PWM started\r\n");

    AS5047P as5047p;

    while ( true )     {
        // Keep application alive; ISR updates PWM each period
        SYSTICK_DelayMs(500);
        Logger_Info("Running...\r\n");
    }

}
