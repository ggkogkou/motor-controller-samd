#include "pmsm_controller.hpp"

namespace PermanentMagnetSynchronousMotor {

namespace {

[[nodiscard]] uint32_t duty_q15_to_period(uint32_t pwmPeriod, uint16_t duty_q15) noexcept {
        const uint64_t prod = static_cast<uint64_t>(pwmPeriod) * static_cast<uint64_t>(duty_q15);
        return pwmPeriod - static_cast<uint32_t>(prod >> 15);
}

} // namespace

PMSM_Controller::PMSM_Controller() {
        // Compute once (startup only) — removes float from the loop.
        targetVelocity_mrad_s = std::lroundf(PMSM_Config::TargetVelocity * 1000.0f);

        const float step_counts =
                PMSM_Config::TargetCalibrationVelocity * dT * (static_cast<float>(16384) / (2.0f * MathUtilities::PI));

        const int32_t step_i32 = std::lroundf(step_counts);
        openLoopStepCounts14 = (step_i32 <= 0) ? 1u : static_cast<uint16_t>(step_i32);
}

bool PMSM_Controller::startupCalibration(const PhaseDutyCycles& dutyCycles, uint16_t thetaEncoder) {
        if (calibrationState == CalibrationState::PREPARING) {
                updateOpenLoop(dutyCycles);

                if (thetaEncoder > 0 && thetaEncoder < 2048)
                        calibrationState = CalibrationState::DIRECTION_CALIBRATION;
                else
                        return false;
        }

        if (calibrationState == CalibrationState::DONE) {
                if (!velocityEstimator) {
                        const int32_t wrapped_mrad = rawToMilliRad(thetaEncoder);
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

void PMSM_Controller::directionCalibration(const PhaseDutyCycles& dutyCycles, uint16_t thetaEncoder) {
        updateOpenLoop(dutyCycles);

        static uint16_t thetaStart = 0;
        static bool ongoingCalibration = false;

        if (not ongoingCalibration) {
                thetaStart = wrapAngle(thetaEncoder);
                ongoingCalibration = true;
                timerCounter = 0;
                return;
        }

        timerCounter++;

        if (timerCounter > MoveDuringCalibrationTicks) {
                const uint16_t thetaFinal = wrapAngle(thetaEncoder);

                int32_t d = static_cast<int32_t>(thetaFinal) - static_cast<int32_t>(thetaStart);

                if (d > 8192)
                        d -= 16384;
                if (d < -8192)
                        d += 16384;

                dirSign = d >= 0 ? +1 : -1;

                ongoingCalibration = false;
                timerCounter = 0;
                calibrationState = CalibrationState::OFFSET_CALIBRATION;
        }
}

void PMSM_Controller::encoderOffsetCalibration(const PhaseDutyCycles& dutyCycles, uint16_t thetaEncoder) {
        constexpr int32_t Uq = 0; /// in mV
        const int32_t Ud = std::lroundf(PMSM_Config::InitialCalibrationVoltageLimit * 1000.0f); /// in mV
        constexpr uint16_t ThetaElectricalLock = 0;

        const auto AlphaBetaFrame = MathUtils::performInverseParkTransform(Ud, Uq, ThetaElectricalLock);
        const auto DutyCycles = pwm.computeQ15(clamp16Bits(AlphaBetaFrame[0]), clamp16Bits(AlphaBetaFrame[1]));

        const uint32_t tmpPeriodA = duty_q15_to_period(pwmPeriod, DutyCycles.dutyA_q15);
        const uint32_t tmpPeriodB = duty_q15_to_period(pwmPeriod, DutyCycles.dutyB_q15);
        const uint32_t tmpPeriodC = duty_q15_to_period(pwmPeriod, DutyCycles.dutyC_q15);

        dutyCycles.perA = tmpPeriodA;
        dutyCycles.perB = tmpPeriodB;
        dutyCycles.perC = tmpPeriodC;

        timerCounter++;

        if (timerCounter > MoveDuringCalibrationTicks) {
                thetaMechanical = thetaEncoder;

                // offset so ThetaEl14 == 0 at this locked position
                uint16_t tmpEl14 = wrapAngle(thetaMechanical * PMSM_Config::MotorPolePairs);

                if (dirSign < 0)
                        tmpEl14 = wrapAngle(16384u - tmpEl14);

                ZeroOffsetElectricalAngle = tmpEl14;

                timerCounter = 0;
                calibrationState = CalibrationState::DONE;
        }
}

void PMSM_Controller::updateOpenLoop(const PhaseDutyCycles& dutyCycles) {
        thetaMechanical = wrapAngle(thetaMechanical + static_cast<uint32_t>(openLoopStepCounts14));
        // thetaMechanical = wrapAngle(thetaMechanical + ConvertRadToRaw(PMSM_Config::TargetCalibrationVelocity));

        const uint16_t thetaElectrical = wrapAngle(thetaMechanical * PMSM_Config::MotorPolePairs);

        const int32_t Uq = std::lroundf(PMSM_Config::OpenLoopVoltLimit_mV); /// in mV
        constexpr int32_t Ud = 0; /// in mV

        const auto InvPark = MathUtils::performInverseParkTransform(Ud, Uq, thetaElectrical);
        const auto [dA, dB, dC] = pwm.computeQ15(clamp16Bits(InvPark[0]), clamp16Bits(InvPark[1]));

        const uint32_t tmpPeriodA = duty_q15_to_period(pwmPeriod, dA);
        const uint32_t tmpPeriodB = duty_q15_to_period(pwmPeriod, dB);
        const uint32_t tmpPeriodC = duty_q15_to_period(pwmPeriod, dC);

        dutyCycles.perA = tmpPeriodA;
        dutyCycles.perB = tmpPeriodB;
        dutyCycles.perC = tmpPeriodC;
}

void PMSM_Controller::updateVelocity(const PhaseCurrents& phaseCurrents, const PhaseDutyCycles& dutyCycles, uint16_t thetaEncoder) {
        if (not velocityEstimator)
                return;

        BENCHMARK_IO_Set();

        const int32_t wrapped_mrad = rawToMilliRad(thetaEncoder);

        constexpr uint32_t deltaTime = 1000; // us
        velocityEstimator->update(wrapped_mrad, deltaTime);

        const auto VelocityAbsoluteValue = [](int32_t x) -> int32_t {
                if (x < 0)
                        return -x;

                return x;
        }(velocityEstimator->angularVelocity);

        const int32_t VelocityError = targetVelocity_mrad_s - VelocityAbsoluteValue; /// in mrad/s

        const uint16_t ThetaEl = calculateElectricalAngle(thetaEncoder);

        const int32_t Iq_Ref = pidVelocity.compute(VelocityError); /// output is Iq,ref in mA

        const auto Ia_mA = static_cast<int32_t>(phaseCurrents.Ia * 1000.0f); /// in mA
        const auto Ib_mA = static_cast<int32_t>(phaseCurrents.Ib * 1000.0f); /// in mA

        const auto dqFrame = MathUtils::performClarkeParkTransforms(Ia_mA, Ib_mA, ThetaEl);

        constexpr int32_t Id_Ref = 0; /// in mA

        const int32_t Id_Error = Id_Ref - dqFrame[0]; /// in mA
        const int32_t Iq_Error = Iq_Ref - dqFrame[1]; /// in mA

        const int32_t Uq_mV = pidIq.compute(Iq_Error); /// in mV
        const int32_t Ud_mV = pidId.compute(Id_Error); /// in mV

        /// TODO: Create a limit circle function that takes also care of overmodulation etc

        const auto AlphaBetaFrame = MathUtils::performInverseParkTransform(Ud_mV, Uq_mV, ThetaEl);
        const auto [dA, dB, dC] = pwm.computeQ15(clamp16Bits(AlphaBetaFrame[0]), clamp16Bits(AlphaBetaFrame[1]));

        const uint32_t tmpPeriodA = duty_q15_to_period(pwmPeriod, dA);
        const uint32_t tmpPeriodB = duty_q15_to_period(pwmPeriod, dB);
        const uint32_t tmpPeriodC = duty_q15_to_period(pwmPeriod, dC);

        BENCHMARK_IO_Clear();

        dutyCycles.perA = tmpPeriodA;
        dutyCycles.perB = tmpPeriodB;
        dutyCycles.perC = tmpPeriodC;
}

void PMSM_Controller::stopMotor(const PhaseDutyCycles& dutyCycles) const {
        constexpr int16_t V_Alpha = 0;
        constexpr int16_t V_Beta = 0;

        const auto [dA, dB, dC] = pwm.computeQ15(V_Alpha, V_Beta);

        const uint32_t tmpPeriodA = duty_q15_to_period(pwmPeriod, dA);
        const uint32_t tmpPeriodB = duty_q15_to_period(pwmPeriod, dB);
        const uint32_t tmpPeriodC = duty_q15_to_period(pwmPeriod, dC);

        dutyCycles.perA = tmpPeriodA;
        dutyCycles.perB = tmpPeriodB;
        dutyCycles.perC = tmpPeriodC;
}

} // namespace PermanentMagnetSynchronousMotor
