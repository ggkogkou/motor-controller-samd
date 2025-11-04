#include <cstdlib>
#include "as5047p.hpp"
#include "definitions.h"
#include "drv8316.hpp"
#include "logger.h"
#include "logger.hpp"
#include "pid.hpp"
#include "svpwm.hpp"
#include "etl/string.h"

using namespace SpaceVectorModulation;
using namespace MathUtilities;

AS5047P Encoder;
DRV8316 BrushlessDriver;

inline constexpr float GlobalVoltageLimit = 8.0f;
inline constexpr float DCLinkVoltage = 20.0f;
inline constexpr float TargetVelocity = 10.0f;

enum class Direction : uint8_t {
    CLOCKWISE,
    COUNTERCLOCKWISE,
};

struct VelocityEstimator {
    float prev_mod = 0.0f;     // previous modulo angle in rad [0, 2π)
    float prev_unw = 0.0f;     // previous unwrapped angle in rad (−∞, ∞)
    float omega = 0.0f;        // filtered velocity (rad/s)
    bool  init = false;
};

static volatile VelocityEstimator vel;

// wrap to (−π, π]
static inline float wrap_pi(float x) {
    while (x <= -TWO_PI) x += TWO_PI;
    while (x >   TWO_PI) x -= TWO_PI;
    if (x >  PI) x -= TWO_PI;
    if (x <= -PI) x += TWO_PI;
    return x;
}

// unwrap current modulo angle using shortest signed delta
static inline float unwrap(float theta_mod, float prev_mod, float prev_unw) {
    const float d = wrap_pi(theta_mod - prev_mod);  // shortest diff
    return prev_unw + d;
}


/**
 * What must be the initial θm value?
 */
static volatile float theta_m = 0.0f;

/**
 * The dT periodic update of the angle for open-loop control
 */
static volatile float dT = 0;

/**
 *
 * Brushless DC GM4108H-120T Gimbal Motor
 * --------------------------------------
 * Pole pairs: 11
 * No-load current: 0.07±0.1A
 * No-load voltage: 20 V
 * Load torque: 1200-1800 g*cm
 * Motor internal resistance: 11.1±5% Ω
 * No-load RPM: 513-567 RPM @ 20 V => calculate its Kv rating as approximately 25.65-28.35 RPM/V
 *
 */
inline constexpr float MotorKV_Rating = 26.0f;
inline constexpr float MotorPolePairs = 11.0f;
inline constexpr float MotorInternalResistance = 11.0f;

static volatile float ZeroElectricalAngle = 0.0f;

/**
 *
 * DRV8316 Pinout Matching
 * -----------------------
 * Phase U: TCC0_WO5, ADC_AIN2
 * Phase V: TCC0_WO6, ADC_AIN3
 * Phase W: TCC1_WO1, ADC_AIN4
 *
 */
static volatile uint32_t TCC_PeriodU = 0;
static volatile uint32_t TCC_PeriodV = 0;
static volatile uint32_t TCC_PeriodW = 0;

inline constexpr ADC_POSINPUT ADC_InputU = ADC_POSINPUT_PIN2;
inline constexpr ADC_POSINPUT ADC_InputV = ADC_POSINPUT_PIN3;
inline constexpr ADC_POSINPUT ADC_InputW = ADC_POSINPUT_PIN4;

static volatile uint16_t adcResultU = 0;
static volatile uint16_t adcResultV = 0;
static volatile uint16_t adcResultW = 0;

static volatile uint8_t adcCounter = 0;
static volatile bool adcResultsReady = false;

inline constexpr ADC_NEGINPUT NegativeInput = ADC_NEGINPUT_GND;
inline constexpr uint16_t ADC_VREF = 2230; // mV

inline constexpr float Vq_Align = 6.0f;
inline constexpr float Vd_Align = 0.0f;
inline constexpr float TargetCalibrationVelocity = TWO_PI;

static volatile uint16_t angleEncoder = 0;

SVPWM svpwm{DCLinkVoltage, ZeroSequenceModulationType::MIDPOINT_CLAMP};

PID pid_controller {0.5f, 10.0f, 0.0f, 6.0f, 0.00100000005};

Logger logger;

uint32_t period = 0;

/**
 * The PWM end-of-period Interrupt Service Routine (ISR) Callback function
 *
 * @param status
 * @param context
 */
void PWM_IRQ_Callback(uint32_t status, uintptr_t context) {
    // const auto Period = TCC0_PWM24bitPeriodGet();

    if (status & TCC_INTFLAG_MC2_Msk && adcCounter == 0) {
        ADC_ChannelSelect(ADC_InputU, NegativeInput);
        ADC_ConversionStart();
    }

    if (status & TCC_INTFLAG_OVF_Msk) {
        TCC0_PWM24bitDutySet(TCC0_CHANNEL1, TCC_PeriodU);
        TCC0_PWM24bitDutySet(TCC0_CHANNEL2, TCC_PeriodV);
        TCC1_PWM24bitDutySet(TCC1_CHANNEL1, TCC_PeriodW);
        // TCC0_PWM24bitDutySet(TCC0_CHANNEL1, period);
        // TCC0_PWM24bitDutySet(TCC0_CHANNEL2, period);
        // TCC1_PWM24bitDutySet(TCC1_CHANNEL1, period);
    }
}

void ADC_Callback(ADC_STATUS status, uintptr_t context) {
    (void)context;

    if (status & ADC_INTFLAG_OVERRUN_Msk)
        ADC_InterruptsClear(ADC_INTFLAG_OVERRUN_Msk);

    if (status & ADC_INTFLAG_RESRDY_Msk) {
        if (adcCounter == 0) {
            adcResultU = ADC_ConversionResultGet();
            ADC_ChannelSelect(ADC_InputV, NegativeInput);
            ADC_ConversionStart();
            adcCounter = 1;
        }
        else if (adcCounter == 1) {
            adcResultV = ADC_ConversionResultGet();
            ADC_ChannelSelect(ADC_InputW, NegativeInput);
            ADC_ConversionStart();
            adcCounter = 2;
        }
        else if (adcCounter == 2) {
            adcResultW = ADC_ConversionResultGet();
            adcCounter = 3;
            adcResultsReady = true;
        }
    }
}

static volatile bool ongoingOffsetCalibration = true;
static volatile bool ongoingDirectionCalibration = true;
static volatile bool ongoingDirectionCalibration2 = true;
static volatile bool closedLoopControl = true;

static uint32_t timer_counter = 0;
static uint32_t needed_ticks = 0;

static float encoder_angle_init = 0;
static float encoder_angle_mid = 0;
static float encoder_angle_end = 0;

constexpr float deg2rad(float deg) {
    return deg * (3.14159265358979323846f / 180.0f);
}

static float filt_alpha(float dt, float tau) {
    // equivalent to tau/(tau+dt)
    return (tau / (tau + dt));
}

void TC3_FOC_HandlerOpenLoop(TC_TIMER_STATUS status, uintptr_t context) {
    auto wrap = [](float x) {
        while (x < 0.0f)
            x += TWO_PI;
        while (x >= TWO_PI)
            x -= TWO_PI;
        return x;
    };

    if (adcResultsReady) {
        __disable_irq();
        uint16_t currentPhaseU = ADC_VREF * adcResultU / static_cast<uint16_t>(4095);
        uint16_t currentPhaseV = ADC_VREF * adcResultV / static_cast<uint16_t>(4095);
        uint16_t currentPhaseW = ADC_VREF * adcResultW / static_cast<uint16_t>(4095);
        auto the_sum = static_cast<int32_t>(currentPhaseU) + static_cast<int32_t>(currentPhaseV) + static_cast<int32_t>(currentPhaseW) - 3*1650;
        adcCounter = 0;
        adcResultsReady = false;
        __enable_irq();
    }

    if (ongoingDirectionCalibration) {
        const float theta_m_next = wrap(theta_m + dT * TargetCalibrationVelocity);
        theta_m = theta_m_next;

        const float theta_e = wrap(theta_m * MotorPolePairs);
        const auto inv_park = performInverseParkTransform(0.0f, GlobalVoltageLimit, theta_e);
        const auto [dutyCycleA, dutyCycleB, dutyCycleC] = svpwm.compute(inv_park[0], inv_park[1]);

        TCC_PeriodU = period - static_cast<uint32_t>(static_cast<float>(period) * dutyCycleA);
        TCC_PeriodV = period - static_cast<uint32_t>(static_cast<float>(period) * dutyCycleB);
        TCC_PeriodW = period - static_cast<uint32_t>(static_cast<float>(period) * dutyCycleC);

        if (++timer_counter == needed_ticks) {
            __disable_irq();
            encoder_angle_mid = Encoder.measureAngleUncompensated();
            timer_counter = 0;
            ongoingDirectionCalibration = false;
            __enable_irq();
        }

    } else if (ongoingDirectionCalibration2) {
        const float theta_m_next = wrap(theta_m + dT * TargetCalibrationVelocity);
        theta_m = theta_m_next;

        const float theta_e = wrap(- theta_m * MotorPolePairs);
        const auto inv_park = performInverseParkTransform(0.0f, GlobalVoltageLimit, theta_e);
        const auto [dutyCycleA, dutyCycleB, dutyCycleC] = svpwm.compute(inv_park[0], inv_park[1]);

        TCC_PeriodU = period - static_cast<uint32_t>(static_cast<float>(period) * dutyCycleA);
        TCC_PeriodV = period - static_cast<uint32_t>(static_cast<float>(period) * dutyCycleB);
        TCC_PeriodW = period - static_cast<uint32_t>(static_cast<float>(period) * dutyCycleC);

        if (++timer_counter == needed_ticks) {
            __disable_irq();
            encoder_angle_end = Encoder.measureAngleUncompensated();
            timer_counter = 0;
            ongoingDirectionCalibration2 = false;
            __enable_irq();
        }

    } else if (ongoingOffsetCalibration) {
        constexpr float Vq = GlobalVoltageLimit;
        constexpr float Vd = 0.0f;
        constexpr float ThetaElectrical = 4.71238898038f;

        const auto AlphaBetaFrame = performInverseParkTransform(Vd, Vq, ThetaElectrical);
        const auto DutyCycles = svpwm.compute(AlphaBetaFrame[0], AlphaBetaFrame[1]);
        const auto [dutyCycleA, dutyCycleB, dutyCycleC] = DutyCycles;

        TCC_PeriodU = period - static_cast<uint32_t>(static_cast<float>(period) * dutyCycleA);
        TCC_PeriodV = period - static_cast<uint32_t>(static_cast<float>(period) * dutyCycleB);
        TCC_PeriodW = period - static_cast<uint32_t>(static_cast<float>(period) * dutyCycleC);

        if (++timer_counter == needed_ticks) {
            __disable_irq();
            ZeroElectricalAngle = deg2rad(Encoder.measureAngleUncompensated());
            timer_counter = 0;
            theta_m = deg2rad(Encoder.measureAngleUncompensated());
            ongoingOffsetCalibration = false;
            __enable_irq();
        }

    }
    else if (closedLoopControl) {
        __disable_irq();
        const float theta_mod = deg2rad(Encoder.measureAngleUncompensated());
        __enable_irq();

        if (!vel.init) {
            vel.prev_mod = theta_mod;
            vel.prev_unw = theta_mod;
            vel.omega    = 0.0f;
            vel.init     = true;
        } else {
            const float theta_unw = unwrap(theta_mod, vel.prev_mod, vel.prev_unw);
            const float deriv     = (theta_unw - vel.prev_unw) / dT;

            const float alpha = filt_alpha(dT, 0.010f);
            vel.omega = alpha * vel.omega + (1.0f - alpha) * deriv;

            vel.prev_unw = theta_unw;
            vel.prev_mod = theta_mod;
        }

        const float error_factor = TargetVelocity - std::fabs(vel.omega);
        const float Isp = pid_controller.compute(error_factor);

        const float theta_m_next = wrap(theta_m + dT * TargetVelocity);
        theta_m = theta_m_next;

        const float theta_e = wrap(theta_m * MotorPolePairs - ZeroElectricalAngle);
        const auto inv_park = performInverseParkTransform(0.0f, Isp, theta_e);
        const auto [dutyCycleA, dutyCycleB, dutyCycleC] = svpwm.compute(inv_park[0], inv_park[1]);

        TCC_PeriodU = period - static_cast<uint32_t>(static_cast<float>(period) * dutyCycleA);
        TCC_PeriodV = period - static_cast<uint32_t>(static_cast<float>(period) * dutyCycleB);
        TCC_PeriodW = period - static_cast<uint32_t>(static_cast<float>(period) * dutyCycleC);

    } else if (not closedLoopControl) {
        const float theta_m_next = wrap(theta_m + dT * TargetVelocity);
        theta_m = theta_m_next;

        const float theta_e = wrap(theta_m * MotorPolePairs - ZeroElectricalAngle);
        const auto inv_park = performInverseParkTransform(0.0f, GlobalVoltageLimit, theta_e);
        const auto [dutyCycleA, dutyCycleB, dutyCycleC] = svpwm.compute(inv_park[0], inv_park[1]);

        TCC_PeriodU = period - static_cast<uint32_t>(static_cast<float>(period) * dutyCycleA);
        TCC_PeriodV = period - static_cast<uint32_t>(static_cast<float>(period) * dutyCycleB);
        TCC_PeriodW = period - static_cast<uint32_t>(static_cast<float>(period) * dutyCycleC);
    }

}

void TC3_FOC_HandlerClosedLoop(TC_TIMER_STATUS status, uintptr_t context) {

}

void SPI_Callback(uintptr_t context) {
    (void)context;


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

void devices_init() {
    SPARE_GPIO_Clear();
    BrushlessDriver.unlockAllRegisters();
    BrushlessDriver.setPWMMode(DRV8316::PWM_Mode::MODE_3x);
    BrushlessDriver.setCurrentSenseAmplifierGain(DRV8316::CurrentSenseGain::CSA_GAIN_0_30);
}

void peripherals_init() {
    __disable_irq();

    const uint32_t f_tc = TC3_TimerFrequencyGet();
    const uint32_t top = TC3_Timer16bitPeriodGet();
    dT = (1.0f + static_cast<float>(top)) / static_cast<float>(f_tc);
    needed_ticks = static_cast<uint32_t>(1.0f / dT);
    period = TCC0_PWM24bitPeriodGet();

    SYSTICK_TimerStart();
    Logger_Initialize();
    ADC_Enable();

    ADC_CallbackRegister(ADC_Callback, 0);
    TCC0_PWMCallbackRegister(PWM_IRQ_Callback, 0);
    TC3_TimerCallbackRegister(TC3_FOC_HandlerOpenLoop, 0);

    TCC0_PWMStart();
    TCC1_PWMStart();
    TC3_TimerStart();

    __enable_irq();
}

[[noreturn]] int main() {
    SYS_Initialize(nullptr);

    devices_init();

    encoder_angle_init = Encoder.measureAngleUncompensated();

    peripherals_init();

    while (true) {
        // auto x = BrushlessDriver.checkForFaults();
        // auto y = Encoder.measureAngleUncompensated();

        // logger << "[INFO] Running";
        // float angleee = 12.345675f;
        // logger << "[INFO] Running, angle = " << angleee; // sends once, with "\r\n" appended

        debug_led_task();
    }
}
