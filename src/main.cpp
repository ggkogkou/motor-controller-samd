#include <cstdlib>
#include "definitions.h"
#include "logger.h"
#include "as5047p.hpp"
#include "drv8316.hpp"
#include "svpwm.hpp"

using namespace SpaceVectorModulation;
using namespace MathUtilities;

static volatile uint32_t g_pwm_isr_cycles_last = 0;
static volatile uint32_t g_pwm_isr_cycles_max  = 0;
static volatile uint32_t g_pwm_isr_cycles_min  = 0x00FFFFFF;

/**
 * The PWM end-of-period Interrupt Service Routine (ISR) Callback function
 *
 * @param status
 * @param context
 */
void PWM_IRQ_Callback(uint32_t status, uintptr_t context) {
    const auto Period = TCC0_PWM24bitPeriodGet();

    uint32_t start = SYSTICK_TimerCounterGet();

    if (status & TCC_INTFLAG_MC2_Msk)
        ADC_ConversionStart();

    if (status & TCC_INTFLAG_OVF_Msk) {
        TCC0_PWM24bitDutySet(TCC0_CHANNEL1, Period/2);
        TCC0_PWM24bitDutySet(TCC0_CHANNEL2, Period/2);
        TCC1_PWM24bitDutySet(TCC1_CHANNEL1, Period/2);
    }

    uint32_t end = SYSTICK_TimerCounterGet();

    uint32_t dur = (start - end) & 0x00FFFFFF;

    g_pwm_isr_cycles_last = dur;
    if (dur > g_pwm_isr_cycles_max)
        g_pwm_isr_cycles_max = dur;

    if (dur < g_pwm_isr_cycles_min)
        g_pwm_isr_cycles_min = dur;
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

static uint32_t CyclesToUs(uint32_t cycles) {
    uint32_t f = SYSTICK_TimerFrequencyGet(); // Hz
    return (uint32_t)((cycles * 1000000ULL + (f/2)) / f);
}

[[noreturn]] int main ( ) {
    SYS_Initialize ( nullptr);

    SYSTICK_TimerStart();
    SYSTICK_TimerPeriodSet(0x00FFFFFF);

    Logger_Initialize();

    // TCC0_PWMCallbackRegister(PWM_IRQ_Callback, 0);
    // ADC_Enable();
    // ADC_CallbackRegister(ADC_Callback, 0);

    SYSTICK_DelayMs(10);
    Logger_Info("PWM configured\r\n");

    // TCC0_PWMStart();
    // TCC1_PWMStart();

    SYSTICK_DelayMs(10);

    const AS5047P Encoder;

    SPARE_GPIO_Clear();
    DRV8316 BrushlessDriver;
    BrushlessDriver.unlockAllRegisters();
    BrushlessDriver.setPWMMode(DRV8316::PWM_Mode::MODE_3x);
    BrushlessDriver.setCurrentSenseAmplifierGain(DRV8316::CurrentSenseGain::CSA_GAIN_0_30);

    uint32_t start = 0;
    uint32_t end = 0;
    uint32_t dur = 0;

    SVPWM pwm {5.0f, ZeroSequenceModulationType::MIDPOINT_CLAMP};

    while (true) {
        auto x = BrushlessDriver.checkForFaults();
        auto y = Encoder.measureAngleUncompensated();

        Logger_Info("Running...\r\n");
        auto prd = TCC0_PWM24bitPeriodGet();

        start = SYSTICK_TimerCounterGet();

        auto alphaBetaFrame = performInverseParkTransform(1, 1, 1);
        auto duties = pwm.compute(alphaBetaFrame[0], alphaBetaFrame[1]);

        auto calcPeriodA = prd - prd * duties.dutyCycleA;
        auto calcPeriodB = prd - prd * duties.dutyCycleB;
        auto calcPeriodC = prd - prd * duties.dutyCycleC;

        end = SYSTICK_TimerCounterGet();

        dur = (start - end) & 0x00FFFFFF;

        g_pwm_isr_cycles_last = dur;
        if (dur > g_pwm_isr_cycles_max)
            g_pwm_isr_cycles_max = dur;

        if (dur < g_pwm_isr_cycles_min)
            g_pwm_isr_cycles_min = dur;

        debug_led_task();

        auto last_ticks = g_pwm_isr_cycles_last;
        auto last_us = CyclesToUs(g_pwm_isr_cycles_last);

        auto max_ticks = g_pwm_isr_cycles_max;
        auto max_us = CyclesToUs(g_pwm_isr_cycles_max);

        auto min_ticks = g_pwm_isr_cycles_min;
        auto min_us = CyclesToUs(g_pwm_isr_cycles_min);

        SYSTICK_DelayUs(1);
    }

}
