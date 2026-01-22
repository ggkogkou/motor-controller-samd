// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Georgios Gkogkou <ggkogkou125@gmail.com>

/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

/**
 * @file   pmsm_controller.cpp
 * @brief  Hardware-independent Field-Oriented Control algorithm implementation
 * @author Georgios Gkogkou <ggkogkou125@gmail.com>
 */

#include "pmsm_controller.hpp"

namespace PermanentMagnetSynchronousMotor {

PMSM_Controller::PMSM_Controller(uint32_t pwmPeriod) : PMSM_Controller(pwmPeriod, 1'000) {}

PMSM_Controller::PMSM_Controller(uint32_t pwmPeriod, uint32_t velocityLoopPeriod) :
    pidVelocity(0.5f, 10.0f, 0.0f, 6000.0f, static_cast<float>(velocityLoopPeriod) * 1e-6f),
    pidId(0.25f, 20.0f, 0.0f, PMSM_Config::CloseLoopVoltageLimit * 1000.0f, static_cast<float>(velocityLoopPeriod) * 1e-6f),
    pidIq(0.35f, 50.0f, 0.0f, PMSM_Config::CloseLoopVoltageLimit * 1000.0f, static_cast<float>(velocityLoopPeriod) * 1e-6f),
    velocityLoopPeriod_us(velocityLoopPeriod), pwmPeriod(pwmPeriod), dT(static_cast<float>(velocityLoopPeriod) * 1e-6f) {
        targetVelocity_mrad_s = std::lroundf(PMSM_Config::TargetVelocity * 1000.0f);

        const float step_counts = PMSM_Config::TargetCalibrationVelocity * dT * (static_cast<float>(16384) / (2.0f * PI));

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

        const auto InvPark = MathUtils::performInverseParkTransform(Ud, Uq, ThetaElectricalLock);
        const auto [perA, perB, perC] = pwm.compute(InvPark[0], InvPark[1], pwmPeriod);

        dutyCycles.perA = pwmPeriod - perA;
        dutyCycles.perB = pwmPeriod - perB;
        dutyCycles.perC = pwmPeriod - perC;

        timerCounter++;

        if (timerCounter > MoveDuringCalibrationTicks) {
                thetaMechanical = thetaEncoder;

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
        const auto [perA, perB, perC] = pwm.compute(InvPark[0], InvPark[1], pwmPeriod);

        dutyCycles.perA = pwmPeriod - perA;
        dutyCycles.perB = pwmPeriod - perB;
        dutyCycles.perC = pwmPeriod - perC;
}

void PMSM_Controller::updateVelocity(const PhaseCurrents& phaseCurrents, const PhaseDutyCycles& dutyCycles, uint16_t thetaEncoder) {
        if (not velocityEstimator)
                return;

        BENCHMARK_IO_Set();

        const int32_t wrapped_mrad = rawToMilliRad(thetaEncoder);

        const uint32_t DeltaTime = velocityLoopPeriod_us == 0 ? 1 : velocityLoopPeriod_us;

        velocityEstimator->update(wrapped_mrad, DeltaTime);

        const auto VelocityAbsoluteValue = [](int32_t x) -> int32_t {
                if (x < 0)
                        return -x;

                return x;
        }(velocityEstimator->angularVelocity);

        const int32_t VelocityError = targetVelocity_mrad_s - VelocityAbsoluteValue; /// in mrad/s

        const uint16_t ThetaEl = calculateElectricalAngle(thetaEncoder);

        const int32_t Iq_Ref = pidVelocity.compute(VelocityError); /// output is Iq,ref in mA

        const auto dqFrame = MathUtils::performClarkeParkTransforms(phaseCurrents.Ia_mA, phaseCurrents.Ib_mA, ThetaEl);

        constexpr int32_t Id_Ref = 0; /// in mA

        const int32_t Id_Error = Id_Ref - dqFrame[0]; /// in mA
        const int32_t Iq_Error = Iq_Ref - dqFrame[1]; /// in mA

        const int32_t Uq_mV = pidIq.compute(Iq_Error); /// in mV
        const int32_t Ud_mV = pidId.compute(Id_Error); /// in mV

        /// TODO: Create a limit circle function that takes also care of overmodulation etc

        const auto InvPark = MathUtils::performInverseParkTransform(Ud_mV, Uq_mV, ThetaEl);
        const auto [perA, perB, perC] = pwm.compute(InvPark[0], InvPark[1], pwmPeriod);

        dutyCycles.perA = pwmPeriod - perA;
        dutyCycles.perB = pwmPeriod - perB;
        dutyCycles.perC = pwmPeriod - perC;

        BENCHMARK_IO_Clear();
}

void PMSM_Controller::stopMotor(const PhaseDutyCycles& dutyCycles) const {
        constexpr int16_t V_Alpha = 0;
        constexpr int16_t V_Beta = 0;

        const auto [perA, perB, perC] = pwm.compute(V_Alpha, V_Beta, pwmPeriod);

        dutyCycles.perA = pwmPeriod - perA;
        dutyCycles.perB = pwmPeriod - perB;
        dutyCycles.perC = pwmPeriod - perC;
}

} // namespace PermanentMagnetSynchronousMotor
