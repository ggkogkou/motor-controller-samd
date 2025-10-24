#include <cstdlib>
#include "definitions.h"
#include "logger.h"
#include "as5047p.hpp"
#include "drv8316.hpp"
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

    auto prd = TCC0_PWM24bitPeriodGet();

    if (status & TCC_INTFLAG_MC2_Msk)
        ADC_ConversionStart();

    if (status & TCC_INTFLAG_OVF_Msk) {
        TCC0_PWM24bitDutySet(TCC0_CHANNEL1, prd/2);
        TCC1_PWM24bitDutySet(TCC1_CHANNEL1, prd/2);
        TCC2_PWM16bitDutySet(TCC2_CHANNEL0, prd/2);
    }

}

// static volatile uint32_t adcResult = -1;

void ADC_Callback( ADC_STATUS status, uintptr_t context ) {
    uint32_t adcResult = -1;

    if (status & ADC_INTFLAG_RESRDY_Msk)
        adcResult = ADC_ConversionResultGet();

    return;
}

static volatile bool debug_led_state = false;

void debug_led_task() {
    if (debug_led_state) {
        DEBUG_LED_Clear();
        debug_led_state = false;
    }
    else {
        DEBUG_LED_Set();
        debug_led_state = true;
    }

    SYSTICK_DelayMs(1000);
}

[[noreturn]] int main ( ) {
    /* Initialize all modules */
    SYS_Initialize ( nullptr);

    SYSTICK_TimerStart();
    Logger_Initialize();

    /* Register callback function for period event */
    // TCC0_PWMCallbackRegister(TCC_PeriodEventHandler, (uintptr_t)NULL);
    // ADC_Enable();
    // ADC_CallbackRegister(ADC_Callback, NULL);

    /* Read the period */
    // ADC_ConversionStart();
    period = TCC0_PWM24bitPeriodGet();
    SYSTICK_DelayMs(100);
    Logger_Info("PWM period configured\r\n");

    // TCC0_PWMStart();
    // TCC1_PWMStart();
    // TCC2_PWMStart();

    SYSTICK_DelayMs(1000);
    Logger_Info("PWM started\r\n");

    AS5047P as5047p;
    DRV8316 drv8316;

    while ( true ) {
        // SYSTICK_DelayMs(1000);
        drv8316.readRegister(DRV8316::RegisterAddress::Control_Register_1);
        // SYSTICK_DelayMs(100);
        // auto y = as5047p.measureAngleUncompensated();
        Logger_Info("Running...\r\n");

        // debug_led_task();
    }

}
