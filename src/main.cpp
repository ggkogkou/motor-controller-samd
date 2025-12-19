#include <cstdlib>
#include "USART_TxStream.hpp"
#include "as5047p.hpp"
#include "definitions.h"
#include "drv8316.hpp"
#include "logger.hpp"
#include "pmsm_controller.hpp"
#include "svpwm.hpp"

using namespace SpaceVectorModulation;
using namespace MathUtilities;
using namespace PermanentMagnetSynchronousMotor;

PMSM_Controller brushlessMotor;
AS5047P encoder;
DRV8316 drv8316;

/**
 * The dT periodic update of the angle for open-loop control
 */
static volatile float dT = 0;

/**
 *
 * DRV8316 Pinout Matching
 * -----------------------
 * Phase U: TCC0_WO5, ADC_AIN2
 * Phase V: TCC0_WO6, ADC_AIN3
 * Phase W: TCC1_WO1, ADC_AIN4
 *
 */
uint32_t TCC_PeriodU = 0;
uint32_t TCC_PeriodV = 0;
uint32_t TCC_PeriodW = 0;

inline constexpr ADC_POSINPUT ADC_InputU = ADC_POSINPUT_PIN3;
inline constexpr ADC_POSINPUT ADC_InputV = ADC_POSINPUT_PIN10;
inline constexpr ADC_POSINPUT ADC_InputW = ADC_POSINPUT_PIN11;

static volatile uint16_t adcResultU = 0;
static volatile uint16_t adcResultV = 0;
static volatile uint16_t adcResultW = 0;

static volatile bool adcResultsReady = false;

inline constexpr ADC_NEGINPUT NegativeInput = ADC_NEGINPUT_GND;
inline constexpr uint16_t ADC_VREF = 2230; // mV

SVPWM svpwm{20'000, ZeroSequenceModulationType::MIDPOINT_CLAMP};

USART_TxStream logging;

uint32_t period = 0;

static volatile uint8_t adcScanIndex = 0;

void PWM_IRQ_Callback(uint32_t status, uintptr_t context) {
        (void)context;

        if (status & TCC_INTFLAG_OVF_Msk) {
                TCC0_PWM24bitDutySet(TCC0_CHANNEL0, TCC_PeriodU);
                TCC0_PWM24bitDutySet(TCC0_CHANNEL1, TCC_PeriodV);
                TCC0_PWM24bitDutySet(TCC0_CHANNEL2, TCC_PeriodW);
                TCC0_PWM24bitDutySet(TCC0_CHANNEL3, 1000);
        }
}

void ADC_Callback(ADC_STATUS status, uintptr_t context) {
        (void)context;

        if (status & ADC_INTFLAG_RESRDY_Msk) {
                const uint16_t sample = ADC_ConversionResultGet();

                if (adcScanIndex == 0u) {
                        adcResultU = sample;
                } else if (adcScanIndex == 7u) {
                        adcResultV = sample;
                } else if (adcScanIndex == 8u) {
                        adcResultW = sample;
                }

                adcScanIndex = (adcScanIndex + 1u) % 9u;

                if (adcScanIndex == 0u) {
                        adcResultsReady = true;
                }
        }

        if (status & ADC_INTFLAG_OVERRUN_Msk) {
                ADC_InterruptsClear(ADC_INTFLAG_OVERRUN_Msk);
        }
}

static volatile bool closedLoopControlCurrent = true;
static volatile bool ongoingSOOffsetCal = true;

static volatile float soW_off = 1.65f;
static volatile float soV_off = 1.65f;

static void get_currents_BC(float& iA, float& iB, float& iC) {
        const uint16_t rv = adcResultV;
        const uint16_t rw = adcResultW;
        adcResultsReady = false;

        iB = static_cast<float>(rv) * 3.3f / 4095.0f;
        iC = static_cast<float>(rw) * 3.3f / 4095.0f;

        drv8316.calculateCurrents(iA, iB, iC);
}

static void calibrateDRV8316_SOx(PhaseDutyCycles& cycles) {
        brushlessMotor.stopMotor(cycles);

        static uint32_t n = 0;
        static float sumV = 0.0, sumW = 0.0;

        if (adcResultsReady) {
                const uint16_t rv = adcResultV;
                const uint16_t rw = adcResultW;
                adcResultsReady = false;

                const float soV = static_cast<float>(rv) * 3.3f / 4095.0f;
                const float soW = static_cast<float>(rw) * 3.3f / 4095.0f;

                sumV += soV;
                sumW += soW;
                ++n;

                if (n >= 512) {
                        soV_off = static_cast<float>(sumV / static_cast<float>(n));
                        soW_off = static_cast<float>(sumW / static_cast<float>(n));
                        sumV = sumW = 0.0;
                        n = 0;
                        drv8316.setOffsetVoltages(soV_off, soW_off);
                        ongoingSOOffsetCal = false;
                }
        }
}

bool switchToCloseLoop = false;

volatile uint8_t counter = 0;

void TC3_FOC_HandlerOpenLoop(TC_TIMER_STATUS, uintptr_t) {

        // PORT_PinWrite(PORT_PIN_PA17, true);
        BENCHMARK_IO_Set();

        static PhaseDutyCycles duty{TCC_PeriodU, TCC_PeriodV, TCC_PeriodW};

        static bool primed = false;
        if (!primed) {
                (void)encoder.request(AS5047P::RegisterAddress::ANGLECOM);
                primed = true;
                return;
        }

        // ---- NEW: raw 14-bit encoder angle (0..16383) ----
        const auto theta14 = static_cast<uint16_t>(encoder.measureAngleCompensatedRaw() & 0x3FFFu);

        if (not AS5047P::sensorBusy()) {
                (void)encoder.request(AS5047P::RegisterAddress::ANGLECOM);
        }

        if (!switchToCloseLoop) {
                switchToCloseLoop = brushlessMotor.startupCalibration(duty, theta14);

                TCC0_PWM24bitDutySet(TCC0_CHANNEL0, TCC_PeriodU);
                TCC0_PWM24bitDutySet(TCC0_CHANNEL1, TCC_PeriodV);
                TCC0_PWM24bitDutySet(TCC0_CHANNEL2, TCC_PeriodW);

                return;
        }

        PhaseCurrents temp{};
        brushlessMotor.updateVelocity(temp, duty, theta14);

        // PORT_PinWrite(PORT_PIN_PA17, false);

        BENCHMARK_IO_Clear();

        TCC0_PWM24bitDutySet(TCC0_CHANNEL0, TCC_PeriodU);
        TCC0_PWM24bitDutySet(TCC0_CHANNEL1, TCC_PeriodV);
        TCC0_PWM24bitDutySet(TCC0_CHANNEL2, TCC_PeriodW);
}

void peripherals_init() {
        __disable_irq();

        const uint32_t f_tc = TC3_TimerFrequencyGet();
        const uint32_t top = TC3_Timer16bitPeriodGet();
        dT = (1.0f + static_cast<float>(top)) / static_cast<float>(f_tc);
        period = TCC0_PWM24bitPeriodGet();

        SYSTICK_TimerStart();
        // ADC_Enable();

        // ADC_CallbackRegister(ADC_Callback, 0);
        // TCC0_PWMCallbackRegister(PWM_IRQ_Callback, 0);
        TC3_TimerCallbackRegister(TC3_FOC_HandlerOpenLoop, 0);

        TCC0_PWMStart();
        TC3_TimerStart();

        TCC0_PWM24bitDutySet(TCC0_CHANNEL0, TCC_PeriodU);
        TCC0_PWM24bitDutySet(TCC0_CHANNEL1, TCC_PeriodV);
        TCC0_PWM24bitDutySet(TCC0_CHANNEL2, TCC_PeriodW);
        TCC0_PWM24bitDutySet(TCC0_CHANNEL3, 1000);

        __enable_irq();
}

[[noreturn]] int main() {
        SYS_Initialize(nullptr);
        SYSTICK_TimerStart();

        SPI_Buffer::init();
        logging.init();
        peripherals_init();

        encoder.request(AS5047P::RegisterAddress::ANGLECOM);

        while (true) {
                static uint8_t c = 'A';
                logging.write(std::span(&c, 1));
                if (++c > 'Z')
                        c = 'A';

                uint8_t eol[2] = {'\r', '\n'};
                logging.write(std::span(eol, 2));

                SYSTICK_DelayMs(200);
        }
}
