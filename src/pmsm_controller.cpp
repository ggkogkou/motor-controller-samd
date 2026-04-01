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

PMSM_Controller::PMSM_Controller(uint32_t pwmPeriodArg) : PMSM_Controller(pwmPeriodArg, 1'000, 1'000) {}

PMSM_Controller::PMSM_Controller(uint32_t pwmPeriodArg, uint32_t velocityLoopPeriod, uint32_t currentLoopPeriod) :
    pidPosition(1.0f, 0.0f, 0.0f, 50'000.0f, static_cast<float>(velocityLoopPeriod) * 1e-6f),
    pidVelocity(1.0f, 10.0f, 0.0f, 4000.0f, static_cast<float>(velocityLoopPeriod) * 1e-6f),
    pidId(0.5f, 100.0f, 0.0f, PMSM_Config::CloseLoopVoltageLimit * 1000.0f, static_cast<float>(currentLoopPeriod) * 1e-6f),
    pidIq(0.5f, 100.0f, 0.0f, PMSM_Config::CloseLoopVoltageLimit * 1000.0f, static_cast<float>(currentLoopPeriod) * 1e-6f),
    velocityLoopPeriod_us(tmrCriticalVariables1.velocityLoopPeriod_us, tmrCriticalVariables2.velocityLoopPeriod_us,
                          tmrCriticalVariables3.velocityLoopPeriod_us),
    CurrentLoopPeriod_us(currentLoopPeriod),
    pwmPeriod(tmrCriticalVariables1.pwmPeriod, tmrCriticalVariables2.pwmPeriod, tmrCriticalVariables3.pwmPeriod),
    dT(static_cast<float>(velocityLoopPeriod) * 1e-6f),
    ZeroOffsetElectricalAngle(tmrCriticalVariables1.zeroOffsetElectricalAngle, tmrCriticalVariables2.zeroOffsetElectricalAngle,
                              tmrCriticalVariables3.zeroOffsetElectricalAngle),
    dirSign(tmrCriticalVariables1.dirSign, tmrCriticalVariables2.dirSign, tmrCriticalVariables3.dirSign),
    lastDirSign(tmrCriticalVariables1.lastDirSign, tmrCriticalVariables2.lastDirSign, tmrCriticalVariables3.lastDirSign),
    startupCalibrator(pwmPeriod, ZeroOffsetElectricalAngle, dirSign, velocityLoopPeriod) {
        pwmPeriod.write(pwmPeriodArg);
        velocityLoopPeriod_us.write(velocityLoopPeriod);
        ZeroOffsetElectricalAngle.write(0);
        dirSign.write(1);
        lastDirSign.write(1);

        dT = static_cast<float>(velocityLoopPeriod_us.read()) * 1e-6f;

        targetVelocity_mrad_s = std::lroundf(PMSM_Config::TargetVelocity * 1000.0f);
}

bool PMSM_Controller::startupCalibration(const PhaseDutyCycles& dutyCycles, uint16_t thetaEncoder) {
        const bool done = startupCalibrator.startupCalibration(dutyCycles, thetaEncoder);

        if (done) {
                if (not velocityEstimator) {
                        const int32_t wrapped_mrad = signedMechanical_mrad(thetaEncoder);
                        velocityEstimator.emplace(wrapped_mrad, 10'000u); // tau = 10ms
                }

                return true;
        }

        return false;
}

void PMSM_Controller::updateOpenLoop(const PhaseDutyCycles& dutyCycles) {
        startupCalibrator.updateOpenLoop(dutyCycles);
}

void PMSM_Controller::setTargetPosition(int32_t targetAngle_mrad, PositionDirection direction, int32_t revolutions) {
        constexpr int32_t TWO_PI_MRAD = 6283;

        const auto wrap_mrad = [&](int32_t angle) -> int32_t {
                angle %= TWO_PI_MRAD;
                if (angle < 0)
                        angle += TWO_PI_MRAD;
                return angle;
        };

        targetPosition_mrad = wrap_mrad(targetAngle_mrad);
        positionDirection = direction;
        targetRevolutions = revolutions < 0 ? 0 : revolutions;
        targetUnwrappedValid = false;
}

void __attribute__((section(".ramfunc"))) PMSM_Controller::runPositionLoop(uint16_t thetaEncoder) {
        constexpr int32_t TWO_PI_MRAD = 6'283;

        if (dirSign.read() != lastDirSign.read()) {
                targetUnwrappedValid = false;
                lastDirSign.write(dirSign.read());
        }

        const int32_t current_wrapped_mrad = signedMechanical_mrad(thetaEncoder);
        const int32_t current_unwrapped_mrad = velocityEstimator ? velocityEstimator->unwrappedAngle : current_wrapped_mrad;

        PositionDirection effectiveDirection = positionDirection;
        int32_t target_wrapped_mrad = targetPosition_mrad;
        if (dirSign.read() < 0) {
                target_wrapped_mrad = TWO_PI_MRAD - target_wrapped_mrad;
                if (target_wrapped_mrad >= TWO_PI_MRAD)
                        target_wrapped_mrad -= TWO_PI_MRAD;
                if (effectiveDirection == PositionDirection::CW)
                        effectiveDirection = PositionDirection::CCW;
                else if (effectiveDirection == PositionDirection::CCW)
                        effectiveDirection = PositionDirection::CW;
        }

        const int32_t forward = [&] {
                int32_t f = target_wrapped_mrad - current_wrapped_mrad;
                if (f < 0)
                        f += TWO_PI_MRAD;
                return f;
        }();

        const int32_t backward = forward - TWO_PI_MRAD;

        if (not targetUnwrappedValid) {
                const int32_t delta = [&] {
                        switch (effectiveDirection) {
                        case PositionDirection::CW:
                                return forward + targetRevolutions * TWO_PI_MRAD;
                        case PositionDirection::CCW:
                                return backward - targetRevolutions * TWO_PI_MRAD;
                        case PositionDirection::SHORTEST:
                        default:
                                return (forward <= -backward) ? forward : backward;
                        }
                }();

                targetUnwrapped_mrad = current_unwrapped_mrad + delta;
                targetUnwrappedValid = true;
        }

        tlm_target_position_mrad = target_wrapped_mrad;
        tlm_target_unwrapped_mrad = targetUnwrapped_mrad;

        const int32_t error = targetUnwrapped_mrad - current_unwrapped_mrad;
        targetVelocity_mrad_s = pidPosition.compute(error);
        if (effectiveDirection == PositionDirection::CW) {
                if (targetVelocity_mrad_s < 0)
                        targetVelocity_mrad_s = -targetVelocity_mrad_s;
        } else if (effectiveDirection == PositionDirection::CCW) {
                if (targetVelocity_mrad_s > 0)
                        targetVelocity_mrad_s = -targetVelocity_mrad_s;
        }
}

void __attribute__((section(".ramfunc"))) PMSM_Controller::runVelocityLoop(uint16_t thetaEncoder, TelemetryLogger* telemetry) {
        // BENCHMARK_IO_Set();
        if (not velocityEstimator)
                return;

        const int32_t wrapped_mrad = signedMechanical_mrad(thetaEncoder);

        const uint32_t DeltaTime = velocityLoopPeriod_us.read() == 0 ? 1u : velocityLoopPeriod_us.read();

        velocityEstimator->update(wrapped_mrad, DeltaTime);

        const int32_t VelocityError = targetVelocity_mrad_s - velocityEstimator->angularVelocity; /// in mrad/s

        Iref.Iq_mA = pidVelocity.compute(VelocityError); /// output is Iq,ref in mA

        tlm_omega_mrad_s = velocityEstimator->angularVelocity;
        tlm_target_velocity_mrad_s = targetVelocity_mrad_s;

        if (telemetry) {
                TelemetryParameters tp;
                tp.dirSign = (dirSign.read() < 0) ? -1 : 1;
                tp.ZeroOffsetElectricalAngle = static_cast<uint32_t>(ZeroOffsetElectricalAngle.read());
                tp.ThetaEl = tlm_theta_el;
                tp.ia_mA = tlm_ia_mA;
                tp.ib_mA = tlm_ib_mA;
                tp.adcOffsetU = tlm_adc_u_off;
                tp.adcOffsetV = tlm_adc_v_off;
                tp.iq_ref_mA = tlm_iq_ref_mA;
                tp.angle_raw = static_cast<uint32_t>(thetaEncoder);
                tp.runtime_mem_corruption_err = 0;
                tp.encoder_err = tlm_encoder_error_code;

                telemetry->updateLatest(tp);
        }
        // BENCHMARK_IO_Clear();
}

void __attribute__((section(".ramfunc"))) PMSM_Controller::runCurrentLoop(const PhaseCurrents& phaseCurrents,
                                                                          PhaseDutyCycles& dutyCycles, uint16_t thetaEncoder) {
        // BENCHMARK_IO_Set();

        const uint16_t ThetaEl = calculateElectricalAngle(thetaEncoder);
        tlm_theta_el = ThetaEl;

        const auto dqFrame = MathUtils::performClarkeParkTransforms(phaseCurrents.Ia_mA, phaseCurrents.Ib_mA, ThetaEl);

        const int32_t Id_Error = Iref.Id_mA - dqFrame[0]; /// in mA
        const int32_t Iq_Error = Iref.Iq_mA - dqFrame[1]; /// in mA

        const int32_t Uq_mV = pidIq.compute(Iq_Error); /// in mV
        const int32_t Ud_mV = pidId.compute(Id_Error); /// in mV

        /// TODO: Create a limit circle function that takes also care of overmodulation etc

        const auto InvPark = MathUtils::performInverseParkTransform(Ud_mV, Uq_mV, ThetaEl);
        const auto [perA, perB, perC] = pwm.compute(InvPark[0], InvPark[1], pwmPeriod.read());

        dutyCycles.perA = pwmPeriod.read() - perA;
        dutyCycles.perB = pwmPeriod.read() - perB;
        dutyCycles.perC = pwmPeriod.read() - perC;

        tlm_ia_mA = phaseCurrents.Ia_mA;
        tlm_ib_mA = phaseCurrents.Ib_mA;
        tlm_id_mA = dqFrame[0];
        tlm_iq_mA = dqFrame[1];
        tlm_id_ref_mA = Iref.Id_mA;
        tlm_iq_ref_mA = Iref.Iq_mA;

        // BENCHMARK_IO_Clear();
}

void PMSM_Controller::stopMotor(const PhaseDutyCycles& dutyCycles) const {
        constexpr int16_t V_Alpha = 0;
        constexpr int16_t V_Beta = 0;

        const auto [perA, perB, perC] = pwm.compute(V_Alpha, V_Beta, pwmPeriod.read());

        dutyCycles.perA = pwmPeriod.read() - perA;
        dutyCycles.perB = pwmPeriod.read() - perB;
        dutyCycles.perC = pwmPeriod.read() - perC;
}

} // namespace PermanentMagnetSynchronousMotor
