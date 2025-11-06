#include "pmsm_controller.hpp"

namespace PermanentMagnetSynchronousMotor {

void PMSM_Controller::directionCalibration(uint32_t& perA, uint32_t& perB, uint32_t& perC) {
        thetaMechanical = wrapAngle(thetaMechanical + 0.001f * PMSM_Config::TargetCalibrationVelocity);

        const auto thetaElectrical = [&]() -> float {
                switch (calibrationDirection) {
                case Direction::CLOCKWISE:
                        return wrapAngle(thetaMechanical * PMSM_Config::MotorPolePairs);
                case Direction::COUNTERCLOCKWISE:
                        return wrapAngle(-thetaMechanical * PMSM_Config::MotorPolePairs);
                default:
                        return 0.0f;
                }
        }();

        constexpr float Vd = 0.0f;
        constexpr float Vq = PMSM_Config::InitialCalibrationVoltageLimit;

        const auto InvPark = performInverseParkTransform(Vd, Vq, thetaElectrical);
        const auto [dutyCycleA, dutyCycleB, dutyCycleC] = pwm.compute(InvPark[0], InvPark[1]);

        perA = pwmPeriod - static_cast<uint32_t>(static_cast<float>(pwmPeriod) * dutyCycleA);
        perB = pwmPeriod - static_cast<uint32_t>(static_cast<float>(pwmPeriod) * dutyCycleB);
        perC = pwmPeriod - static_cast<uint32_t>(static_cast<float>(pwmPeriod) * dutyCycleC);

        timerCounter++;

        if (timerCounter > neededTicks && directionCalibrationState == DirectionCalibrationState::CALIBRATE_CW) {
                timerCounter = 0;
                directionCalibrationState = DirectionCalibrationState::CALIBRATE_CCW;
                calibrationDirection = Direction::COUNTERCLOCKWISE;
        } else if (timerCounter > neededTicks &&
                   directionCalibrationState == DirectionCalibrationState::CALIBRATE_CCW) {
                timerCounter = 0;
                directionCalibrationState = DirectionCalibrationState::DONE;
                calibrationState = CalibrationState::DONE;
        }
}

void PMSM_Controller::encoderOffsetCalibration(uint32_t& perA, uint32_t& perB, uint32_t& perC, float thetaEncoder) {
        constexpr float Vq = 0.0f;
        constexpr float Vd = PMSM_Config::InitialCalibrationVoltageLimit;
        constexpr float ThetaElectricalLock = 0.0f;

        const auto AlphaBetaFrame = performInverseParkTransform(Vd, Vq, ThetaElectricalLock);
        const auto DutyCycles = pwm.compute(AlphaBetaFrame[0], AlphaBetaFrame[1]);
        const auto [dutyCycleA, dutyCycleB, dutyCycleC] = DutyCycles;

        perA = pwmPeriod - static_cast<uint32_t>(static_cast<float>(pwmPeriod) * dutyCycleA);
        perB = pwmPeriod - static_cast<uint32_t>(static_cast<float>(pwmPeriod) * dutyCycleB);
        perC = pwmPeriod - static_cast<uint32_t>(static_cast<float>(pwmPeriod) * dutyCycleC);

        timerCounter++;

        if (timerCounter > neededTicks) {
                __disable_irq();
                thetaMechanical = degreesToRadians(thetaEncoder);
                ZeroOffsetElectricalAngle = wrapAngle(PMSM_Config::MotorPolePairs * thetaMechanical);
                __enable_irq();
                timerCounter = 0;
                calibrationState = CalibrationState::DONE;
        }
}

void PMSM_Controller::startupCalibration(uint32_t& perA, uint32_t& perB, uint32_t& perC, float thetaEncoder) {
        if (calibrationState == CalibrationState::DIRECTION_CALIBRATION)
                directionCalibration(perA, perB, perC);
        else if (calibrationState == CalibrationState::OFFSET_CALIBRATION)
                encoderOffsetCalibration(perA, perB, perC, thetaEncoder);
        else if (controlType == ControlType::OPEN_LOOP)
                updateOpenLoop(perA, perB, perC);
        else if (controlType == ControlType::CLOSED_LOOP)
                update(perA, perB, perC);
}

void PMSM_Controller::updateOpenLoop(uint32_t& perA, uint32_t& perB, uint32_t& perC) {
        thetaMechanical = wrapAngle(thetaMechanical + 0.001f * PMSM_Config::TargetCalibrationVelocity);

        const auto thetaElectrical = wrapAngle(thetaMechanical * PMSM_Config::MotorPolePairs);

        constexpr float Vd = 0.0f;
        constexpr float Vq = PMSM_Config::OpenLoopVoltageLimit;

        const auto InvPark = performInverseParkTransform(Vd, Vq, thetaElectrical);
        const auto [dutyCycleA, dutyCycleB, dutyCycleC] = pwm.compute(InvPark[0], InvPark[1]);

        perA = pwmPeriod - static_cast<uint32_t>(static_cast<float>(pwmPeriod) * dutyCycleA);
        perB = pwmPeriod - static_cast<uint32_t>(static_cast<float>(pwmPeriod) * dutyCycleB);
        perC = pwmPeriod - static_cast<uint32_t>(static_cast<float>(pwmPeriod) * dutyCycleC);
}

void PMSM_Controller::update(uint32_t& perA, uint32_t& perB, uint32_t& perC) {

}

} // namespace PermanentMagnetSynchronousMotor
