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
                if (!angleVel_) {
                        const float thetaEncWrapped = std::remainderf(thetaEncoder, TWO_PI);
                        angleVel_.emplace(thetaEncWrapped, 0.010f);
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
        if (!angleVel_)
                return;

        angleVel_->update(std::remainderf(thetaEncoder, TWO_PI), dT);

        const auto ThetaEl =
                wrapAngle(dirSign * PMSM_Config::MotorPolePairs * thetaEncoder - ZeroOffsetElectricalAngle);
        const auto dqFrameCurrents = performClarkeParkTransforms(phaseCurrents.Ia, phaseCurrents.Ib, ThetaEl);

        const float VelocityError = PMSM_Config::TargetVelocity - std::fabs(angleVel_->angularVelocity);
        const float IqRef = pidVelocity.compute(VelocityError);

        constexpr float IdRef = 0.0f;

        const float IdError = IdRef - dqFrameCurrents[0];
        const float IqError = IqRef - dqFrameCurrents[1];

        float Vd = pidId.compute(IdError);
        float Vq = pidIq.compute(IqError);

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
