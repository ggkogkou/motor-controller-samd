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

PMSM_Controller::PMSM_Controller(uint32_t pwmPeriod) : PMSM_Controller(pwmPeriod, 1'000, 1'000) {}

PMSM_Controller::PMSM_Controller(uint32_t pwmPeriod, uint32_t velocityLoopPeriod, uint32_t currentLoopPeriod) :
    pidPosition(3.0f, 0.0f, 0.0f, 50'000.0f, static_cast<float>(velocityLoopPeriod) * 1e-6f),
    pidVelocity(0.5f, 10.0f, 0.0f, 4000.0f, static_cast<float>(velocityLoopPeriod) * 1e-6f),
    pidId(0.25f, 20.0f, 0.0f, PMSM_Config::CloseLoopVoltageLimit * 1000.0f, static_cast<float>(currentLoopPeriod) * 1e-6f),
    pidIq(0.35f, 50.0f, 0.0f, PMSM_Config::CloseLoopVoltageLimit * 1000.0f, static_cast<float>(currentLoopPeriod) * 1e-6f),
    velocityLoopPeriod_us(velocityLoopPeriod), CurrentLoopPeriod_us(currentLoopPeriod), pwmPeriod(pwmPeriod),
    dT(static_cast<float>(velocityLoopPeriod) * 1e-6f) {
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

        if (not ongoingCalibration) {
                directionCalibrationThetaStart = wrapAngle(thetaEncoder);
                ongoingCalibration = true;
                timerCounter = 0;
                return;
        }

        timerCounter++;

        if (timerCounter > MoveDuringCalibrationTicks) {
                const uint16_t thetaFinal = wrapAngle(thetaEncoder);

                int32_t d = static_cast<int32_t>(thetaFinal) - static_cast<int32_t>(directionCalibrationThetaStart);

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

void PMSM_Controller::setTargetPosition(int32_t targetAngle_mrad) {
        constexpr int32_t TWO_PI_MRAD = 6283;
        if (targetAngle_mrad < 0) {
                targetAngle_mrad %= TWO_PI_MRAD;
                if (targetAngle_mrad < 0)
                        targetAngle_mrad += TWO_PI_MRAD;
        } else if (targetAngle_mrad >= TWO_PI_MRAD) {
                targetAngle_mrad %= TWO_PI_MRAD;
        }

        targetPosition_mrad = targetAngle_mrad;
}

void PMSM_Controller::runPositionLoop(uint16_t thetaEncoder) {
        constexpr int32_t TWO_PI_MRAD = 6283;
        constexpr int32_t PI_MRAD = TWO_PI_MRAD / 2;

        const int32_t current_mrad = rawToMilliRad(thetaEncoder);
        int32_t error = targetPosition_mrad - current_mrad;

        if (error > PI_MRAD)
                error -= TWO_PI_MRAD;
        else if (error < -PI_MRAD)
                error += TWO_PI_MRAD;

        targetVelocity_mrad_s = pidPosition.compute(error);
}

void PMSM_Controller::runVelocityLoop(uint16_t thetaEncoder) {
        // BENCHMARK_IO_Set();
        if (not velocityEstimator)
                return;

        const int32_t wrapped_mrad = rawToMilliRad(thetaEncoder);

        const uint32_t DeltaTime = velocityLoopPeriod_us == 0 ? 1 : velocityLoopPeriod_us;

        velocityEstimator->update(wrapped_mrad, DeltaTime);

        // const auto VelocityAbsoluteValue = [](int32_t x) -> int32_t {
        //         if (x < 0)
        //                 return -x;
        //
        //         return x;
        // }(velocityEstimator->angularVelocity);
        //
        // const int32_t VelocityError = targetVelocity_mrad_s - VelocityAbsoluteValue; /// in mrad/s

        const int32_t VelocityError = targetVelocity_mrad_s - velocityEstimator->angularVelocity; /// in mrad/s

        Iref.Iq_mA = pidVelocity.compute(VelocityError); /// output is Iq,ref in mA

        tlm_angle_mrad = wrapped_mrad;
        tlm_omega_mrad_s = velocityEstimator->angularVelocity;
        // BENCHMARK_IO_Clear();
}

void __attribute__((section(".ramfunc")))
PMSM_Controller::runCurrentLoop(const PhaseCurrents& phaseCurrents, PhaseDutyCycles& dutyCycles, uint16_t thetaEncoder,
                                TelemetryLogger* telemetry) {
        BENCHMARK_IO_Set();

        const uint16_t ThetaEl = calculateElectricalAngle(thetaEncoder);

        const auto dqFrame = MathUtils::performClarkeParkTransforms(phaseCurrents.Ia_mA, phaseCurrents.Ib_mA, ThetaEl);

        const int32_t Id_Error = Iref.Id_mA - dqFrame[0]; /// in mA
        const int32_t Iq_Error = Iref.Iq_mA - dqFrame[1]; /// in mA

        const int32_t Uq_mV = pidIq.compute(Iq_Error); /// in mV
        const int32_t Ud_mV = pidId.compute(Id_Error); /// in mV

        const int32_t Ud_i_mV = pidId.lastIntegralTerm();
        const int32_t Uq_i_mV = pidIq.lastIntegralTerm();

        /// TODO: Create a limit circle function that takes also care of overmodulation etc

        const auto InvPark = MathUtils::performInverseParkTransform(Ud_mV, Uq_mV, ThetaEl);
        const auto [perA, perB, perC] = pwm.compute(InvPark[0], InvPark[1], pwmPeriod);

        dutyCycles.perA = pwmPeriod - perA;
        dutyCycles.perB = pwmPeriod - perB;
        dutyCycles.perC = pwmPeriod - perC;

        if (telemetry) {
                TelemetryParameters tp;
                tp.seq = ++telemetrySeq;
                tp.t_us = SYSTICK_TimerCounterGet();

                tp.ia_mA = phaseCurrents.Ia_mA;
                tp.ib_mA = phaseCurrents.Ib_mA;
                tp.ic_mA = phaseCurrents.Ic_mA;

                tp.vd_mV = Ud_mV;
                tp.vq_mV = Uq_mV;

                tp.vd_i_mV = Ud_i_mV;
                tp.vq_i_mV = Uq_i_mV;

                tp.id_mA = dqFrame[0];
                tp.iq_mA = dqFrame[1];

                tp.id_ref_mA = Iref.Id_mA;
                tp.iq_ref_mA = Iref.Iq_mA;

                tp.id_err_mA = Id_Error;
                tp.iq_err_mA = Iq_Error;

                tp.angle_mrad = rawToMilliRad(thetaEncoder);
                tp.omega_mrad_s = tlm_omega_mrad_s;

                telemetry->updateLatest(tp);
        }

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
