#include <cstdlib>
#include "as5047p.hpp"
#include "definitions.h"
#include "drv8316.hpp"
#include "logger.h"
#include "svpwm.hpp"

using namespace SpaceVectorModulation;
using namespace MathUtilities;

static volatile uint32_t TCC_Period1;
static volatile uint32_t TCC_Period2;
static volatile uint32_t TCC_Period3;

inline constexpr float GlobalVoltageLimit = 3.0f;
inline constexpr float DCLinkVoltage = 20.0f;
inline constexpr float TargetVelocity = 3.0f;

/**
 * What must be the initial θm value?
 */
static volatile float theta_m = 0.0f;

/**
 * The dT periodic update of the angle for open-loop control
 */
static volatile float dT = 0;

SVPWM svpwm{DCLinkVoltage, ZeroSequenceModulationType::MIDPOINT_CLAMP};

uint32_t period = 0;

/**
 * The PWM end-of-period Interrupt Service Routine (ISR) Callback function
 *
 * @param status
 * @param context
 */
void PWM_IRQ_Callback(uint32_t status, uintptr_t context) {
    // const auto Period = TCC0_PWM24bitPeriodGet();

    if (status & TCC_INTFLAG_MC2_Msk)
        ADC_ConversionStart();

    if (status & TCC_INTFLAG_OVF_Msk) {
        TCC0_PWM24bitDutySet(TCC0_CHANNEL1, TCC_Period1);
        TCC0_PWM24bitDutySet(TCC0_CHANNEL2, TCC_Period2);
        TCC1_PWM24bitDutySet(TCC1_CHANNEL1, TCC_Period3);
    }
}

void ADC_Callback(ADC_STATUS status, uintptr_t context) {
    uint32_t adcResult = -1;

    if (status & ADC_INTFLAG_RESRDY_Msk)
        adcResult = ADC_ConversionResultGet();

    return;
}

void TC3_FOC_Handler(TC_TIMER_STATUS status, uintptr_t context) {
    auto wrap = [](float x){
        while (x < 0.0f)    x += TWO_PI;
        while (x >= TWO_PI) x -= TWO_PI;
        return x;
    };

    const float theta_m_next = wrap(theta_m + dT * TargetVelocity);
    theta_m = theta_m_next;

    constexpr float pole_pairs = 11.0f;
    const float theta_e = wrap(theta_m * pole_pairs);

    const auto inv_park = performInverseParkTransform(0.0f, GlobalVoltageLimit, theta_e);

    const auto [dutyCycleA, dutyCycleB, dutyCycleC] = svpwm.compute(inv_park[0], inv_park[1]);

    TCC_Period1 = period - static_cast<uint32_t>(static_cast<float>(period) * dutyCycleA);
    TCC_Period2 = period - static_cast<uint32_t>(static_cast<float>(period) * dutyCycleB);
    TCC_Period3 = period - static_cast<uint32_t>(static_cast<float>(period) * dutyCycleC);
}

static volatile bool debug_led_state = false;

/**
 * A periodic LED blinking task for visual debugging purposes
 */
void debug_led_task() {
    if (debug_led_state) {
        DEBUG_LED_Clear();
        debug_led_state = false;
    } else {
        DEBUG_LED_Set();
        debug_led_state = true;
    }

    SYSTICK_DelayMs(1000);
}

[[noreturn]] int main() {
    SYS_Initialize(nullptr);

    SYSTICK_TimerStart();
    Logger_Initialize();

    TCC0_PWMCallbackRegister(PWM_IRQ_Callback, 0);
    // ADC_Enable();
    // ADC_CallbackRegister(ADC_Callback, 0);

    SYSTICK_DelayMs(10);
    Logger_Info("PWM configured\r\n");

    period = TCC0_PWM24bitPeriodGet();

    const uint32_t f_tc = TC3_TimerFrequencyGet();
    const uint32_t top  = TC3_Timer16bitPeriodGet();
    dT = (1.0f + static_cast<float>(top)) / static_cast<float>(f_tc);

    TCC_Period1 = period / 2;
    TCC_Period2 = period / 2;
    TCC_Period3 = period / 2;

    TC3_TimerCallbackRegister(TC3_FOC_Handler, 0);

    SYSTICK_DelayMs(10);

    TCC0_PWMStart();
    TCC1_PWMStart();
    TC3_TimerStart();

    SYSTICK_DelayMs(10);

    const AS5047P Encoder;

    SPARE_GPIO_Clear();
    DRV8316 BrushlessDriver;
    BrushlessDriver.unlockAllRegisters();
    BrushlessDriver.setPWMMode(DRV8316::PWM_Mode::MODE_3x);
    BrushlessDriver.setCurrentSenseAmplifierGain(DRV8316::CurrentSenseGain::CSA_GAIN_0_30);

    while (true) {
        auto x = BrushlessDriver.checkForFaults();
        auto y = Encoder.measureAngleUncompensated();

        Logger_Info("Running...\r\n");
        debug_led_task();
    }
}
