#include "pmsm_controller.hpp"

#include <cmath>

namespace PermanentMagnetSynchronousMotor {

namespace {
        static constexpr uint16_t ENC_MASK = 0x3FFFu;
        static constexpr uint32_t ENC_RES  = 16384u;

        [[nodiscard]] inline uint16_t wrap14(uint32_t x) noexcept {
                return static_cast<uint16_t>(x) & ENC_MASK;
        }

        [[nodiscard]] inline int32_t encoder14_to_mrad(uint16_t theta14) noexcept {
                // mrad = theta14 * 2*pi(mrad) / 16384 ; 2*pi(mrad) ~= 6283
                constexpr int32_t TWO_PI_MRAD = AngleVelocityEstimator::TWO_PI_MRAD;
                return static_cast<int32_t>(
                        (static_cast<int64_t>(theta14) * static_cast<int64_t>(TWO_PI_MRAD)) / static_cast<int64_t>(ENC_RES));
        }

        [[nodiscard]] inline uint16_t electricalAngle14(uint16_t thetaMech14,
                                                        uint8_t polePairs,
                                                        uint16_t zeroOffsetEl14,
                                                        int8_t dirSign) noexcept
        {
                uint16_t thetaEl14 = wrap14(static_cast<uint32_t>(thetaMech14) * static_cast<uint32_t>(polePairs));

                if (dirSign < 0)
                        thetaEl14 = wrap14(static_cast<uint32_t>(ENC_RES) - thetaEl14);

                thetaEl14 = wrap14(static_cast<uint32_t>(thetaEl14) - static_cast<uint32_t>(zeroOffsetEl14));
                return thetaEl14;
        }

        inline void limitCircle_mV(int32_t& Vd_mV, int32_t& Vq_mV, int32_t VLim_mV) noexcept {
                const int64_t vd = static_cast<int64_t>(Vd_mV);
                const int64_t vq = static_cast<int64_t>(Vq_mV);
                const int64_t mag2 = vd * vd + vq * vq;

                const int64_t lim = static_cast<int64_t>(VLim_mV);
                const int64_t lim2 = lim * lim;

                if (mag2 > lim2) {
                        // Bare-minimum behavior matching your old code (divide by 2).
                        Vd_mV /= 2;
                        Vq_mV /= 2;
                }
        }

        [[nodiscard]] inline int16_t clamp_i16(int32_t x) noexcept {
                if (x > 32767)  return 32767;
                if (x < -32768) return -32768;
                return static_cast<int16_t>(x);
        }

        [[nodiscard]] inline uint32_t duty_q15_to_period(uint32_t pwmPeriod, uint16_t duty_q15) noexcept {
                const uint64_t prod = static_cast<uint64_t>(pwmPeriod) * static_cast<uint64_t>(duty_q15);
                return pwmPeriod - static_cast<uint32_t>(prod >> 15);
        }

        [[nodiscard]] inline int32_t iabs32(int32_t x) noexcept { return (x < 0) ? -x : x; }

} // namespace

PMSM_Controller::PMSM_Controller() {
        // kept from your existing constructor (not used here)
        q31pidVelocity.setGainsFloat(0.5f, 10.0f, 0.0f, 6.0f, 0.00100000005f);
        q31pidId.setGainsFloat(0.25f, 20.0f, 0.0f, PMSM_Config::CloseLoopVoltageLimit, 0.00100000005f);
        q31pidIq.setGainsFloat(0.35f, 50.0f, 0.0f, PMSM_Config::CloseLoopVoltageLimit, 0.00100000005f);

        // Compute once (startup only) — removes float from the loop.
        targetVelocity_mrad_s = static_cast<int32_t>(std::lroundf(PMSM_Config::TargetVelocity * 1000.0f));

        const float step_counts =
                (PMSM_Config::TargetCalibrationVelocity * dT) *
                (static_cast<float>(ENC_RES) / (2.0f * 3.14159265358979323846f));

        const int32_t step_i32 = static_cast<int32_t>(std::lroundf(step_counts));
        openLoopStepCounts14 = (step_i32 <= 0) ? 1u : static_cast<uint16_t>(step_i32);
}

void PMSM_Controller::directionCalibration(const PhaseDutyCycles& dutyCycles, uint16_t thetaEncoder) {
        updateOpenLoop(dutyCycles);

        timerCounter++;

        if (timerCounter > neededTicks) {
                // (pi/2 .. 3pi/2) => (4096 .. 12288)
                if (thetaEncoder > 4096u && thetaEncoder < 12288u)
                        dirSign = 1;
                else
                        dirSign = -1;

                timerCounter = 0;
                calibrationDirection = Direction::COUNTERCLOCKWISE;
                calibrationState = CalibrationState::OFFSET_CALIBRATION;
                directionCalibrationState = DirectionCalibrationState::DONE;
        }
}

void PMSM_Controller::encoderOffsetCalibration(const PhaseDutyCycles& dutyCycles, uint16_t thetaEncoder) {
        (void)thetaEncoder;

        constexpr int32_t Vq_mV = 0;
        const int32_t Vd_mV = static_cast<int32_t>(std::lroundf(PMSM_Config::InitialCalibrationVoltageLimit * 1000.0f));

        // Lock electrical angle at 0 => thetaEl14 = 0
        constexpr uint16_t ThetaElectricalLock14 = 0u;

        const auto ab_mV = MathUtils::performInverseParkTransform(Vd_mV, Vq_mV, ThetaElectricalLock14);

        const auto dc = pwm.computeQ15(clamp_i16(ab_mV[0]), clamp_i16(ab_mV[1]));

        const uint32_t tmpPeriodA = duty_q15_to_period(pwmPeriod, dc.dutyA_q15);
        const uint32_t tmpPeriodB = duty_q15_to_period(pwmPeriod, dc.dutyB_q15);
        const uint32_t tmpPeriodC = duty_q15_to_period(pwmPeriod, dc.dutyC_q15);

        __disable_irq();
        dutyCycles.perA = tmpPeriodA;
        dutyCycles.perB = tmpPeriodB;
        dutyCycles.perC = tmpPeriodC;
        __enable_irq();

        timerCounter++;

        if (timerCounter > neededTicks) {
                __disable_irq();
                thetaMechanical14 = thetaEncoder;

                // offset so ThetaEl14 == 0 at this locked position
                uint16_t tmpEl14 = wrap14(static_cast<uint32_t>(thetaMechanical14) * static_cast<uint32_t>(PMSM_Config::MotorPolePairs));
                if (dirSign < 0)
                        tmpEl14 = wrap14(static_cast<uint32_t>(ENC_RES) - tmpEl14);

                ZeroOffsetElectricalAngle14 = tmpEl14;

                __enable_irq();

                timerCounter = 0;
                calibrationState = CalibrationState::DONE;
        }
}

bool PMSM_Controller::startupCalibration(const PhaseDutyCycles& dutyCycles, uint16_t thetaEncoder) {
        if (calibrationState == CalibrationState::PREPARING) {
                updateOpenLoop(dutyCycles);

                // old: thetaEncoder in (0 .. pi/4) => raw counts (0 .. 2048)
                if (thetaEncoder > 0u && thetaEncoder < 2048u)
                        calibrationState = CalibrationState::DIRECTION_CALIBRATION;
                else
                        return false;
        }

        if (calibrationState == CalibrationState::DONE) {
                if (!velocityEstimator) {
                        const int32_t wrapped_mrad = encoder14_to_mrad(thetaEncoder & ENC_MASK);
                        velocityEstimator.emplace(wrapped_mrad, 10'000u); // tau = 10ms
                }
                return true;
        }

        if (calibrationState == CalibrationState::DIRECTION_CALIBRATION)
                directionCalibration(dutyCycles, thetaEncoder);
        else if (calibrationState == CalibrationState::OFFSET_CALIBRATION)
                encoderOffsetCalibration(dutyCycles, thetaEncoder);
        else
                return true;

        return false;
}

void PMSM_Controller::updateOpenLoop(const PhaseDutyCycles& dutyCycles) {
        thetaMechanical14 = wrap14(static_cast<uint32_t>(thetaMechanical14) + static_cast<uint32_t>(openLoopStepCounts14));

        const uint16_t thetaElectrical14 =
                wrap14(static_cast<uint32_t>(thetaMechanical14) * static_cast<uint32_t>(PMSM_Config::MotorPolePairs));

        const int32_t Vd_mV = 0;
        const int32_t Vq_mV = static_cast<int32_t>(std::lroundf(PMSM_Config::OpenLoopVoltageLimit * 1000.0f));

        const auto ab_mV = MathUtils::performInverseParkTransform(Vd_mV, Vq_mV, thetaElectrical14);

        const auto dc = pwm.computeQ15(clamp_i16(ab_mV[0]), clamp_i16(ab_mV[1]));

        const uint32_t tmpPeriodA = duty_q15_to_period(pwmPeriod, dc.dutyA_q15);
        const uint32_t tmpPeriodB = duty_q15_to_period(pwmPeriod, dc.dutyB_q15);
        const uint32_t tmpPeriodC = duty_q15_to_period(pwmPeriod, dc.dutyC_q15);

        __disable_irq();
        dutyCycles.perA = tmpPeriodA;
        dutyCycles.perB = tmpPeriodB;
        dutyCycles.perC = tmpPeriodC;
        __enable_irq();
}

void PMSM_Controller::update(const PhaseCurrents& phaseCurrents, const PhaseDutyCycles& dutyCycles, uint16_t thetaEncoder) {
        const uint16_t ThetaEl14 =
                electricalAngle14(thetaEncoder, PMSM_Config::MotorPolePairs, ZeroOffsetElectricalAngle14, dirSign);

        // currents: A -> mA (still float source, but fixed-point from here)
        const int32_t Ia_mA = static_cast<int32_t>(std::lroundf(phaseCurrents.Ia * 1000.0f));
        const int32_t Ib_mA = static_cast<int32_t>(std::lroundf(phaseCurrents.Ib * 1000.0f));

        const auto dq_mA = MathUtils::performClarkeParkTransforms(Ia_mA, Ib_mA, ThetaEl14);

        constexpr int32_t id_ref_mA = 0;
        constexpr int32_t iq_ref_mA = 500; // 0.5A

        const int32_t id_err_mA = id_ref_mA - dq_mA[0];
        const int32_t iq_err_mA = iq_ref_mA - dq_mA[1];

        int32_t Vd_mV = pidId.compute(id_err_mA);
        int32_t Vq_mV = pidIq.compute(iq_err_mA);

        limitCircle_mV(Vd_mV, Vq_mV, static_cast<int32_t>(PMSM_Config::CloseLoopVoltageLimit * 1000.0f));

        const auto ab_mV = MathUtils::performInverseParkTransform(Vd_mV, Vq_mV, ThetaEl14);

        const auto dc = pwm.computeQ15(clamp_i16(ab_mV[0]), clamp_i16(ab_mV[1]));

        const uint32_t tmpPeriodA = duty_q15_to_period(pwmPeriod, dc.dutyA_q15);
        const uint32_t tmpPeriodB = duty_q15_to_period(pwmPeriod, dc.dutyB_q15);
        const uint32_t tmpPeriodC = duty_q15_to_period(pwmPeriod, dc.dutyC_q15);

        __disable_irq();
        dutyCycles.perA = tmpPeriodA;
        dutyCycles.perB = tmpPeriodB;
        dutyCycles.perC = tmpPeriodC;
        __enable_irq();
}

void PMSM_Controller::updateVelocity(const PhaseCurrents& phaseCurrents, const PhaseDutyCycles& dutyCycles, uint16_t thetaEncoder) {
        if (!velocityEstimator)
                return;

        const int32_t wrapped_mrad = encoder14_to_mrad(thetaEncoder & ENC_MASK);

        constexpr uint32_t deltaTime = 1000; // us
        velocityEstimator->update(wrapped_mrad, deltaTime);

        // velocityEstimator->angularVelocity is mrad/s
        const int32_t omega_abs_mrad_s = iabs32(velocityEstimator->angularVelocity);

        // velocity error in mrad/s
        const int32_t VelocityError_mrad_s = targetVelocity_mrad_s - omega_abs_mrad_s;

        const uint16_t ThetaEl14 =
                electricalAngle14(thetaEncoder, PMSM_Config::MotorPolePairs, ZeroOffsetElectricalAngle14, dirSign);

        // currents: A -> mA
        const auto Ia_mA = static_cast<int32_t>(std::lroundf(phaseCurrents.Ia * 1000.0f));
        const auto Ib_mA = static_cast<int32_t>(std::lroundf(phaseCurrents.Ib * 1000.0f));

        const auto dq_mA = MathUtils::performClarkeParkTransforms(Ia_mA, Ib_mA, ThetaEl14);

        // velocity PI: output is IqRef in mA
        const int32_t IqRef_mA = pidVelocity.compute(VelocityError_mrad_s);

        constexpr int32_t IdRef_mA = 0;

        const int32_t IdError_mA = IdRef_mA - dq_mA[0];
        const int32_t IqError_mA = IqRef_mA - dq_mA[1];

        int32_t Vd_mV = pidId.compute(IdError_mA);
        int32_t Vq_mV = pidIq.compute(IqError_mA);

        limitCircle_mV(Vd_mV, Vq_mV, static_cast<int32_t>(PMSM_Config::CloseLoopVoltageLimit * 1000.0f));

        const auto ab_mV = MathUtils::performInverseParkTransform(Vd_mV, Vq_mV, ThetaEl14);

        const auto dc = pwm.computeQ15(clamp_i16(ab_mV[0]), clamp_i16(ab_mV[1]));

        const uint32_t tmpPeriodA = duty_q15_to_period(pwmPeriod, dc.dutyA_q15);
        const uint32_t tmpPeriodB = duty_q15_to_period(pwmPeriod, dc.dutyB_q15);
        const uint32_t tmpPeriodC = duty_q15_to_period(pwmPeriod, dc.dutyC_q15);

        __disable_irq();
        dutyCycles.perA = tmpPeriodA;
        dutyCycles.perB = tmpPeriodB;
        dutyCycles.perC = tmpPeriodC;
        __enable_irq();
}

void PMSM_Controller::stopMotor(const PhaseDutyCycles& dutyCycles) const {
        constexpr int16_t Valpha_mV = 0;
        constexpr int16_t Vbeta_mV  = 0;

        const auto dc = pwm.computeQ15(Valpha_mV, Vbeta_mV);

        const uint32_t tmpPeriodA = duty_q15_to_period(pwmPeriod, dc.dutyA_q15);
        const uint32_t tmpPeriodB = duty_q15_to_period(pwmPeriod, dc.dutyB_q15);
        const uint32_t tmpPeriodC = duty_q15_to_period(pwmPeriod, dc.dutyC_q15);

        __disable_irq();
        dutyCycles.perA = tmpPeriodA;
        dutyCycles.perB = tmpPeriodB;
        dutyCycles.perC = tmpPeriodC;
        __enable_irq();
}

} // namespace PermanentMagnetSynchronousMotor
