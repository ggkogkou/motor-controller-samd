#include "StartupCalibration.hpp"

#include <cmath>
#include "math_utils.hpp"
#include "pmsm_config.hpp"
#include "pmsm_controller.hpp"

namespace PermanentMagnetSynchronousMotor {

StartupCalibration::StartupCalibration(TMR<uint32_t>& pwmPeriod, TMR<uint16_t>& ZeroOffsetElectricalAngle, TMR<int32_t>& dirSign,
                                       uint32_t velocityLoopPeriod) :
    pwmPeriod(pwmPeriod), ZeroOffsetElectricalAngle(ZeroOffsetElectricalAngle), dirSign(dirSign) {
        const float dT = static_cast<float>(velocityLoopPeriod) * 1e-6f;
        const float step_counts = PMSM_Config::TargetCalibrationVelocity * dT * (static_cast<float>(16384) / (2.0f * PI));
        const int32_t step_i32 = std::lroundf(step_counts);
        openLoopStepCounts14 = (step_i32 <= 0) ? 1u : static_cast<uint16_t>(step_i32);
}

void StartupCalibration::updateOpenLoop(const PhaseDutyCycles& dutyCycles) {
        thetaMechanical = wrapAngle(thetaMechanical + static_cast<uint32_t>(openLoopStepCounts14));
        // thetaMechanical = wrapAngle(thetaMechanical + ConvertRadToRaw(PMSM_Config::TargetCalibrationVelocity));

        const uint16_t thetaElectrical = wrapAngle(thetaMechanical * PMSM_Config::MotorPolePairs);

        const int32_t Uq = std::lroundf(PMSM_Config::OpenLoopVoltLimit_mV); /// in mV
        constexpr int32_t Ud = 0; /// in mV

        const auto InvPark = MathUtils::performInverseParkTransform(Ud, Uq, thetaElectrical);
        const auto [perA, perB, perC] = pwm.compute(InvPark[0], InvPark[1], pwmPeriod.read());

        dutyCycles.perA = pwmPeriod.read() - perA;
        dutyCycles.perB = pwmPeriod.read() - perB;
        dutyCycles.perC = pwmPeriod.read() - perC;
}

bool StartupCalibration::startupCalibration(const PhaseDutyCycles& dutyCycles, uint16_t thetaEncoder) {
        switch (calibrationState) {
        case CalibrationState::PREPARING:
                {
                        updateOpenLoop(dutyCycles);

                        const uint16_t thetaWrapped = wrapAngle(thetaEncoder);
                        if (thetaWrapped > 0 && thetaWrapped < 1024) {
                                calibrationState = CalibrationState::DIRECTION_CALIBRATION;
                                directionCalibrationState = DirectionCalibrationState::INIT;
                        } else
                                return false;
                }
                [[fallthrough]];
        case CalibrationState::DIRECTION_CALIBRATION:
                directionCalibration(dutyCycles, thetaEncoder);
                return false;
        case CalibrationState::OFFSET_CALIBRATION:
                encoderOffsetCalibration(dutyCycles, thetaEncoder);
                return false;
        case CalibrationState::DONE:
                return true;
        case CalibrationState::IDLE:
        default:
                return true;
        }
}

void StartupCalibration::directionCalibration(const PhaseDutyCycles& dutyCycles, uint16_t thetaEncoder) {
        updateOpenLoop(dutyCycles);

        switch (directionCalibrationState) {
        case DirectionCalibrationState::INIT:
                directionCalibrationThetaStart = wrapAngle(thetaEncoder);
                timerCounter = 0;
                directionCalibrationState = DirectionCalibrationState::MOVING;
                return;
        case DirectionCalibrationState::MOVING:
                timerCounter++;

                if (timerCounter > MoveDuringCalibrationTicks) {
                        const uint16_t thetaFinal = wrapAngle(thetaEncoder);

                        int32_t diff = static_cast<int32_t>(thetaFinal) - static_cast<int32_t>(directionCalibrationThetaStart);

                        if (diff > 8192)
                                diff -= 16384;
                        if (diff < -8192)
                                diff += 16384;

                        if (diff >= 0)
                                dirSign.write(+1);
                        else
                                dirSign.write(-1);

                        timerCounter = 0;
                        directionCalibrationState = DirectionCalibrationState::INIT;
                        offsetCalibrationState = OffsetCalibrationState::LOCKING;
                        calibrationState = CalibrationState::OFFSET_CALIBRATION;
                }
                return;
        default:
                directionCalibrationState = DirectionCalibrationState::INIT;
                return;
        }
}

void StartupCalibration::encoderOffsetCalibration(const PhaseDutyCycles& dutyCycles, uint16_t thetaEncoder) {
        constexpr int32_t Uq = 0; /// in mV
        const int32_t Ud = std::lroundf(PMSM_Config::EncoderOffsetCalibrationVd * 1000.0f); /// in mV
        constexpr uint16_t ThetaElectricalLock = 0;

        const auto InvPark = MathUtils::performInverseParkTransform(Ud, Uq, ThetaElectricalLock);
        const auto [perA, perB, perC] = pwm.compute(InvPark[0], InvPark[1], pwmPeriod.read());

        dutyCycles.perA = pwmPeriod.read() - perA;
        dutyCycles.perB = pwmPeriod.read() - perB;
        dutyCycles.perC = pwmPeriod.read() - perC;

        switch (offsetCalibrationState) {
        case OffsetCalibrationState::LOCKING:
                timerCounter++;

                if (timerCounter > KeepRotorLockedTicks) {
                        thetaMechanical = signedMechanicalRaw(thetaEncoder);

                        ZeroOffsetElectricalAngle.write(wrapAngle(thetaMechanical * PMSM_Config::MotorPolePairs));

                        timerCounter = 0;
                        offsetCalibrationState = OffsetCalibrationState::DONE;
                        calibrationState = CalibrationState::DONE;
                }
                return;
        case OffsetCalibrationState::DONE:
                return;
        default:
                offsetCalibrationState = OffsetCalibrationState::LOCKING;
        }
}

uint16_t StartupCalibration::wrapAngle(uint32_t angle) {
        constexpr uint16_t EncoderMask = 0x3FFF; /// for AS5047P

        return static_cast<uint16_t>(angle) & EncoderMask;
}

uint16_t StartupCalibration::signedMechanicalRaw(uint16_t rawAngle) const noexcept {
        constexpr uint16_t EncoderResolution = 16384u;
        const uint16_t wrapped = wrapAngle(rawAngle);
        const int8_t encoderDir = (dirSign.read() >= 0) ? 1 : -1;
        return encoderDir >= 0 ? wrapped : wrapAngle(EncoderResolution - wrapped);
}

} // namespace PermanentMagnetSynchronousMotor
