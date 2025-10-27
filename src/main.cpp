#include <cstdlib>
#include "definitions.h"
#include "logger.h"
#include "as5047p.hpp"
#include "drv8316.hpp"
#include "svpwm.hpp"

/**
 * The PWM end-of-period Interrupt Service Routine (ISR) Callback function
 *
 * @param status
 * @param context
 */
void PWM_IRQ_Callback(uint32_t status, uintptr_t context) {
    const auto Period = TCC0_PWM24bitPeriodGet();

    if (status & TCC_INTFLAG_MC2_Msk)
        ADC_ConversionStart();

    if (status & TCC_INTFLAG_OVF_Msk) {
        // TCC0_PWM24bitDutySet(TCC0_CHANNEL1, Period/2);
        // TCC0_PWM24bitDutySet(TCC0_CHANNEL2, Period/2);
        // TCC1_PWM24bitDutySet(TCC1_CHANNEL1, Period/2);
        // TCC2_PWM16bitDutySet(TCC2_CHANNEL0, Period/2);
    }
}

void ADC_Callback( ADC_STATUS status, uintptr_t context ) {
    uint32_t adcResult = -1;

    if (status & ADC_INTFLAG_RESRDY_Msk)
        adcResult = ADC_ConversionResultGet();

    return;
}

static volatile bool debug_led_state = false;

/**
 * A periodic LED blinking task for visual debugging purposes
 */
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
    SYS_Initialize ( nullptr);

    SYSTICK_TimerStart();
    Logger_Initialize();

    TCC0_PWMCallbackRegister(PWM_IRQ_Callback, 0);
    ADC_Enable();
    ADC_CallbackRegister(ADC_Callback, 0);

    SYSTICK_DelayMs(10);
    Logger_Info("PWM configured\r\n");

    TCC0_PWMStart();
    TCC1_PWMStart();

    SYSTICK_DelayMs(10);

    const AS5047P Encoder;

    DRV8316 BrushlessDriver;
    // BrushlessDriver.setPWMMode(DRV8316::PWM_Mode::MODE_3x);
    // BrushlessDriver.setCurrentSenseAmplifierGain(DRV8316::CurrentSenseGain::CSA_GAIN_0_15);

    while (true) {
        auto x = BrushlessDriver.checkForFaults();
        auto y = Encoder.measureAngleUncompensated();

        Logger_Info("Running...\r\n");
        debug_led_task();
    }

}
