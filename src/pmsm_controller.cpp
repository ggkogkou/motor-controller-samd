#include "pmsm_controller.hpp"

namespace PermanentMagnetSynchronousMotor {

void PMSM_Controller::directionCalibration(const PhaseDutyCycles& dutyCycles, float thetaEncoder) {
        updateOpenLoop(dutyCycles);

        timerCounter++;

        if (timerCounter > neededTicks) {
                if (thetaEncoder > HALF_PI && thetaEncoder < 3.0f * HALF_PI)
                        dirSign = 1.0f;
                else if (thetaEncoder > 3.0f * HALF_PI || thetaEncoder < HALF_PI)
                        dirSign = -1.0f;

                timerCounter = 0;
                calibrationDirection = Direction::COUNTERCLOCKWISE;
                calibrationState = CalibrationState::OFFSET_CALIBRATION;
                directionCalibrationState = DirectionCalibrationState::DONE;
        }

}

void PMSM_Controller::encoderOffsetCalibration(const PhaseDutyCycles& dutyCycles, float thetaEncoder) {
        constexpr float Vq = 0.0f;
        constexpr float Vd = PMSM_Config::InitialCalibrationVoltageLimit;
        constexpr float ThetaElectricalLock = 0.0f;

        const auto AlphaBetaFrame = performInverseParkTransform(Vd, Vq, ThetaElectricalLock);
        const auto DutyCycles = pwm.compute(AlphaBetaFrame[0], AlphaBetaFrame[1]);
        const auto [dA, dB, dC] = DutyCycles;

        const auto tmpPeriodA = pwmPeriod - static_cast<uint32_t>(static_cast<float>(pwmPeriod) * dA);
        const auto tmpPeriodB = pwmPeriod - static_cast<uint32_t>(static_cast<float>(pwmPeriod) * dB);
        const auto tmpPeriodC = pwmPeriod - static_cast<uint32_t>(static_cast<float>(pwmPeriod) * dC);

        __disable_irq();
        dutyCycles.perA = tmpPeriodA;
        dutyCycles.perB = tmpPeriodB;
        dutyCycles.perC = tmpPeriodC;
        __enable_irq();

        timerCounter++;

        if (timerCounter > neededTicks) {
                __disable_irq();
                thetaMechanical = thetaEncoder;
                ZeroOffsetElectricalAngle = wrapAngle(dirSign * PMSM_Config::MotorPolePairs * thetaMechanical);
                __enable_irq();
                timerCounter = 0;
                calibrationState = CalibrationState::DONE;
        }
}

bool PMSM_Controller::startupCalibration(const PhaseDutyCycles& dutyCycles, float thetaEncoder) {

        if (calibrationState == CalibrationState::PREPARING) {
                updateOpenLoop(dutyCycles);

                if (thetaEncoder > 0.0f && thetaEncoder < HALF_PI / 2.0f)
                        calibrationState = CalibrationState::DIRECTION_CALIBRATION;
                else
                        return false;
        }

        if (calibrationState == CalibrationState::DONE) {
                if (!velocityEstimator) {
                        float wrapped = std::fmod(thetaEncoder, TWO_PI);
                        if (wrapped < 0.0f) wrapped += TWO_PI;

                        const auto wrappedAngleMilliRad = std::lroundf(wrapped * 1000.0f);
                        velocityEstimator.emplace(wrappedAngleMilliRad, 10'000u); // tau = 10ms = 10000us
                }

                return true;
        }

        if (calibrationState == CalibrationState::DIRECTION_CALIBRATION)
                directionCalibration(dutyCycles, thetaEncoder);
        else if (calibrationState == CalibrationState::OFFSET_CALIBRATION)
                encoderOffsetCalibration(dutyCycles, thetaEncoder);
        else
                return true;
        // else if (controlType == ControlType::OPEN_LOOP)
        //         updateOpenLoop(perA, perB, perC);
        // else if (controlType == ControlType::CLOSED_LOOP)
        //         update(perA, perB, perC);

        return false;
        /// Statistically the CLOSED_LOOP will run most; branching should be the opposite to reduce if-else-if checks
}

void PMSM_Controller::updateOpenLoop(const PhaseDutyCycles& dutyCycles) {
        thetaMechanical = wrapAngle(thetaMechanical + 0.001f * PMSM_Config::TargetCalibrationVelocity);

        const auto thetaElectrical = wrapAngle(thetaMechanical * PMSM_Config::MotorPolePairs);

        constexpr float Vd = 0.0f;
        constexpr float Vq = PMSM_Config::OpenLoopVoltageLimit;

        const auto InvPark = performInverseParkTransform(Vd, Vq, thetaElectrical);
        const auto [dA, dB, dC] = pwm.compute(InvPark[0], InvPark[1]);

        const auto tmpPeriodA = pwmPeriod - static_cast<uint32_t>(static_cast<float>(pwmPeriod) * dA);
        const auto tmpPeriodB = pwmPeriod - static_cast<uint32_t>(static_cast<float>(pwmPeriod) * dB);
        const auto tmpPeriodC = pwmPeriod - static_cast<uint32_t>(static_cast<float>(pwmPeriod) * dC);

        __disable_irq();
        dutyCycles.perA = tmpPeriodA;
        dutyCycles.perB = tmpPeriodB;
        dutyCycles.perC = tmpPeriodC;
        __enable_irq();
}

void PMSM_Controller::update(const PhaseCurrents& phaseCurrents, const PhaseDutyCycles& dutyCycles, float thetaEncoder) {
        const auto ThetaEl =
                wrapAngle(dirSign * PMSM_Config::MotorPolePairs * thetaEncoder - ZeroOffsetElectricalAngle);

        const auto dqFrameCurrents = performClarkeParkTransforms(phaseCurrents.Ia, phaseCurrents.Ib, ThetaEl);

        constexpr float id_ref = 0.0f;
        constexpr float iq_ref = 0.5f;

        const float id_err = id_ref - dqFrameCurrents[0];
        const float iq_err = iq_ref - dqFrameCurrents[1];

        float Vd = pidId.compute(id_err);
        float Vq = pidIq.compute(iq_err);
        limitCircle(Vd, Vq, PMSM_Config::CloseLoopVoltageLimit);

        const auto AlphaBetaFrame = performInverseParkTransform(Vd, Vq, ThetaEl);
        const auto [dA, dB, dC] = pwm.compute(AlphaBetaFrame[0], AlphaBetaFrame[1]);

        const auto tmpPeriodA = pwmPeriod - static_cast<uint32_t>(static_cast<float>(pwmPeriod) * dA);
        const auto tmpPeriodB = pwmPeriod - static_cast<uint32_t>(static_cast<float>(pwmPeriod) * dB);
        const auto tmpPeriodC = pwmPeriod - static_cast<uint32_t>(static_cast<float>(pwmPeriod) * dC);

        __disable_irq();
        dutyCycles.perA = tmpPeriodA;
        dutyCycles.perB = tmpPeriodB;
        dutyCycles.perC = tmpPeriodC;
        __enable_irq();
}

void PMSM_Controller::updateVelocity(const PhaseCurrents& phaseCurrents, const PhaseDutyCycles& dutyCycles, float thetaEncoder) {
        if (!velocityEstimator)
                return;

        float wrapped = std::fmod(thetaEncoder, TWO_PI);
        if (wrapped < 0.0f)
                wrapped += TWO_PI;

        auto wrappedAngleMilliRad = static_cast<int32_t>(std::lroundf(wrapped * 1000.0f));
        if (wrappedAngleMilliRad >= AngleVelocityEstimator::TWO_PI_MRAD)
                wrappedAngleMilliRad -= AngleVelocityEstimator::TWO_PI_MRAD;

        constexpr uint32_t deltaTime = 1000;
        velocityEstimator->update(wrappedAngleMilliRad, deltaTime);

        const float omega_rad_s = static_cast<float>(velocityEstimator->angularVelocity) / 1000.0f;
        const float VelocityError = PMSM_Config::TargetVelocity - std::fabs(omega_rad_s);

        const auto ThetaEl =
                wrapAngle(dirSign * PMSM_Config::MotorPolePairs * thetaEncoder - ZeroOffsetElectricalAngle);
        const auto dqFrameCurrents = performClarkeParkTransforms(phaseCurrents.Ia, phaseCurrents.Ib, ThetaEl);

        const float IqRef = pidVelocity.compute(VelocityError);

        constexpr float IdRef = 0.0f;

        const float IdError = IdRef - dqFrameCurrents[0];
        const float IqError = IqRef - dqFrameCurrents[1];

        const auto IdError_mA = static_cast<int32_t>(std::lroundf(IdError * 1000.0f));
        const auto IqError_mA = static_cast<int32_t>(std::lroundf(IqError * 1000.0f));

        const int32_t Vd_mV = pidId.compute(IdError_mA);
        const int32_t Vq_mV = pidIq.compute(IqError_mA);

        float Vd = static_cast<float>(Vd_mV) / 1000.0f;
        float Vq = static_cast<float>(Vq_mV) / 1000.0f;

        limitCircle(Vd, Vq, PMSM_Config::CloseLoopVoltageLimit);

        const auto AlphaBetaFrame = performInverseParkTransform(Vd, Vq, ThetaEl);
        const auto [dA, dB, dC] = pwm.compute(AlphaBetaFrame[0], AlphaBetaFrame[1]);

        const auto tmpPeriodA = pwmPeriod - static_cast<uint32_t>(static_cast<float>(pwmPeriod) * dA);
        const auto tmpPeriodB = pwmPeriod - static_cast<uint32_t>(static_cast<float>(pwmPeriod) * dB);
        const auto tmpPeriodC = pwmPeriod - static_cast<uint32_t>(static_cast<float>(pwmPeriod) * dC);

        __disable_irq();
        dutyCycles.perA = tmpPeriodA;
        dutyCycles.perB = tmpPeriodB;
        dutyCycles.perC = tmpPeriodC;
        __enable_irq();
}

void PMSM_Controller::stopMotor(const PhaseDutyCycles& dutyCycles) const {
        constexpr float Vd = 0.0f;
        constexpr float Vq = 0.0f;

        const auto [dA, dB, dC] = pwm.compute(Vd, Vq);

        const auto tmpPeriodA = pwmPeriod - static_cast<uint32_t>(static_cast<float>(pwmPeriod) * dA);
        const auto tmpPeriodB = pwmPeriod - static_cast<uint32_t>(static_cast<float>(pwmPeriod) * dB);
        const auto tmpPeriodC = pwmPeriod - static_cast<uint32_t>(static_cast<float>(pwmPeriod) * dC);

        __disable_irq();
        dutyCycles.perA = tmpPeriodA;
        dutyCycles.perB = tmpPeriodB;
        dutyCycles.perC = tmpPeriodC;
        __enable_irq();
}

} // namespace PermanentMagnetSynchronousMotor
