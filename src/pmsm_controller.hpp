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
 * @file   pmsm_controller.hpp
 * @brief  Hardware-independent Field-Oriented Control algorithm implementation
 * @author Georgios Gkogkou <ggkogkou125@gmail.com>
 */

#pragma once

#include <cstdint>
#include <optional>
#include "CriticalVariables.hpp"
#include "TMR.hpp"
#include "Telemetry.hpp"
#include "VelocityEstimator.hpp"
#include "definitions.h"
#include "math_utils.hpp"
#include "pid.hpp"
#include "pmsm_config.hpp"
#include "svpwm.hpp"

namespace PermanentMagnetSynchronousMotor {

using namespace SpaceVectorModulation;
using namespace Telemetry;

/**
 * @struct PhaseCurrents
 *
 * The phase currents of the 3-phase system in mA
 */
struct PhaseCurrents {
        int32_t Ia_mA = 0;
        int32_t Ib_mA = 0;
        int32_t Ic_mA = 0;
};

/**
 * @struct PhaseDutyCycles
 *
 * The duty cycles that will be applied to drive the 3-phase system
 */
struct PhaseDutyCycles {
        uint32_t& perA;
        uint32_t& perB;
        uint32_t& perC;

        PhaseDutyCycles(uint32_t& a, uint32_t& b, uint32_t& c) : perA(a), perB(b), perC(c) {}
};

class PMSM_Controller {
public:
        enum class PositionDirection {
                SHORTEST,
                CW,
                CCW,
        };

        enum class ControlMode {
                POSITION,
                VELOCITY,
        };
        /**
         * Class constructor
         */
        explicit PMSM_Controller(uint32_t pwmPeriod);

        explicit PMSM_Controller(uint32_t pwmPeriod, uint32_t velocityLoopPeriod, uint32_t currentLoopPeriod);

        /**
         * Function that runs the velocity loop
         *
         * @param thetaEncoder
         * @param telemetry Optional telemetry logger (nullptr disables logging)
         */
        void runVelocityLoop(uint16_t thetaEncoder, TelemetryLogger* telemetry = nullptr);

        /**
         * Function that runs the inner current control loop (Iq, Id)
         *
         * @param phaseCurrents
         * @param dutyCycles
         * @param thetaEncoder
         */
        void runCurrentLoop(const PhaseCurrents& phaseCurrents, PhaseDutyCycles& dutyCycles, uint16_t thetaEncoder);

        /**
         * Update telemetry values sourced from hardware (ADC + encoder status).
         *
         * @note Called from the fast current loop ISR before telemetry snapshot is produced.
         */
        __attribute__((always_inline)) void updateTelemetryHardware(uint32_t adcSeq, uint32_t missedPairs, uint16_t adcU_raw,
                                                                    uint16_t adcV_raw, uint16_t adcU_off, uint16_t adcV_off,
                                                                    uint32_t encoderErrorCode) {
                tlm_adc_seq = adcSeq;
                tlm_missed_pairs = missedPairs;
                tlm_adc_u_raw = adcU_raw;
                tlm_adc_v_raw = adcV_raw;
                tlm_adc_u_off = adcU_off;
                tlm_adc_v_off = adcV_off;
                tlm_encoder_error_code = encoderErrorCode;
        }

        __attribute__((always_inline)) void updateEncoderErrorCode(uint32_t encoderErrorCode) {
                tlm_encoder_error_code = encoderErrorCode;
        }

        /**
         * Function that moves the rotor in open-loop. It is used during startup calibration procedures.
         *
         * @param dutyCycles
         */
        void updateOpenLoop(const PhaseDutyCycles& dutyCycles);

        /**
         * Function that performs the initial encoder offset and direction calibration
         *
         * To perform the full startup calibration procedures, the following steps are done:
         *
         * 1. Firstly, wherever the rotor is, move it at open-loop at a random direction until the encoder readings
         * reach the interval θ∈[0, π/4) rad. Since the function takes the encoder angle θe as a 14-bits raw value,
         * these angles correspond to raw θ∈[0, 16383], so 0 rad is 0 raw, and π/4 rad is 2048 raw
         *
         * 2. Secondly, the calibration concerning the direction of the movement is performed. The direction is stored
         * and represented as a signed integer value with > 0 meaning CW and < 0 meaning CCW
         *
         * 3. Lastly, the encoder offset calibration is performed. The zero-offset electrical angle is calculated
         */
        bool startupCalibration(const PhaseDutyCycles& dutyCycles, uint16_t thetaEncoder);

        /**
         * Function that stops the motor by applying a zero-vector, calculated by the SVPWM
         *
         * @note The PWM duty cycle update does not happen inside the class and needs to be done by the user externally
         *
         * @param dutyCycles A struct reference that updates the actual duty cycles for the PWM peripheral
         */
        void stopMotor(const PhaseDutyCycles& dutyCycles) const;

        /**
         * Set a target position for the outer position control loop
         *
         * @param targetAngle_mrad Target angle in milliradians [0, 2π)
         * @param direction Path direction (CW, CCW, or SHORTEST)
         * @param revolutions Full turns to add in the given direction
         */
        void setTargetPosition(int32_t targetAngle_mrad, PositionDirection direction = PositionDirection::CCW,
                               int32_t revolutions = 0);

        /**
         * Run the position loop (outer loop) and update the target velocity
         *
         * @param thetaEncoder Current encoder angle (14-bit raw)
         */
        void runPositionLoop(uint16_t thetaEncoder);

private:
        /**
         * The Space Vector PWM block
         */
        SVPWM pwm{14'000, ZeroSequenceModulationType::MIDPOINT_CLAMP};

        /**
         * Position PI: error in mrad, output in mrad/s
         */
        PID pidPosition;

        /**
         * Velocity PI: error in mrad/s, output in mA
         */
        PID pidVelocity;

        /**
         * Current loop PI controllers (for Iq and Id)
         * Inputs in mA, outputs in mV
         *
         * TODO: Tune the P, I parameters
         */
        PID pidId;
        PID pidIq;

        /**
         * @enum CalibrationState
         *
         * Enum class that represents the possible states during the startup calibration process.
         * The values are used to flag when one process is done for the next one to begin.
         */
        enum class CalibrationState {
                IDLE, /// Calibration has not started yet
                PREPARING, /// Bring the rotor in the [0, π/4] interval
                DIRECTION_CALIBRATION, /// Ongoing direction calibration
                OFFSET_CALIBRATION, /// Ongoing encoder offset calibration
                DONE, /// Calibration procedure has finished
        };

        enum class DirectionCalibrationState {
                INIT,
                MOVING,
        };

        enum class OffsetCalibrationState {
                LOCKING,
                DONE,
        };

        /**
         * The calibration state.
         * @note This variable is used to communicate the stage of the calibration between functions.
         */
        CalibrationState calibrationState = CalibrationState::PREPARING;

        /**
         * Open-loop step in encoder counts per tick (computed once in ctor)
         */
        uint16_t openLoopStepCounts14 = 1u;

        /**
         * The target angular velocity in mrad/s
         */
        int32_t targetVelocity_mrad_s = 0;

        /**
         * The target position in mrad [0, 2π)
         */
        int32_t targetPosition_mrad = 0;

        /**
         * Desired path to reach the target position
         */
        PositionDirection positionDirection = PositionDirection::SHORTEST;

        /**
         * Extra full turns to apply in the requested direction
         */
        int32_t targetRevolutions = 0;

        /**
         * Unwrapped target position used for multi-turn positioning
         */
        int32_t targetUnwrapped_mrad = 0;
        bool targetUnwrappedValid = false;

        /**
         * Cache variables for telemetry usage
         * @note Supposed to be updated in velocity loop only
         */
        volatile int32_t tlm_angle_mrad = 0;
        volatile int32_t tlm_omega_mrad_s = 0;
        volatile int32_t tlm_target_velocity_mrad_s = 0;
        volatile int32_t tlm_target_position_mrad = 0;
        volatile int32_t tlm_target_unwrapped_mrad = 0;
        volatile int32_t tlm_ia_mA = 0;
        volatile int32_t tlm_ib_mA = 0;
        volatile int32_t tlm_id_mA = 0;
        volatile int32_t tlm_iq_mA = 0;
        volatile int32_t tlm_id_ref_mA = 0;
        volatile int32_t tlm_iq_ref_mA = 0;
        volatile uint32_t tlm_theta_el = 0;
        volatile uint32_t tlm_adc_seq = 0;
        volatile uint32_t tlm_missed_pairs = 0;
        volatile uint32_t tlm_adc_u_raw = 0;
        volatile uint32_t tlm_adc_v_raw = 0;
        volatile uint32_t tlm_adc_u_off = 0;
        volatile uint32_t tlm_adc_v_off = 0;
        volatile uint32_t tlm_encoder_error_code = 0;

        /**
         * Velocity loop period (ISR period) in microseconds
         */
        TMR<uint32_t> velocityLoopPeriod_us;

        /**
         * Current (Iq, Id) loop period (ISR period) in microseconds
         */
        const uint32_t CurrentLoopPeriod_us = 1'000;

        /**
         * The PWM period, either in counter-ticks or μs
         */
        TMR<uint32_t> pwmPeriod;

        /**
         * Helper variable to count the number of ISRs that have been executed.
         *
         * That keeps track of the position in a very inefficient and inaccurate way.
         *
         * @note Find a better alternative for this
         */
        uint32_t timerCounter = 0;

        /**
         * How many ticks to run direction calibration in open-loop
         */
        static constexpr uint32_t MoveDuringCalibrationTicks = 200;

        /**
         * How many ticks to keep rotor locked during encoder offset calibration
         */
        static constexpr uint32_t KeepRotorLockedTicks = 2000;

        /**
         * @var velocityEstimator
         *
         * Represents an optional AngleVelocityEstimator instance used to estimate the angular velocity
         *
         * This variable may contain a valid estimator or no value if the estimator is not initialized
         */
        std::optional<AngleVelocityEstimator> velocityEstimator;

        /**
         * Loop time step in seconds (derived from velocityLoopPeriod_us)
         */
        float dT;

        /**
         * Zero-offset electrical angle in raw 14-bit format
         */
        TMR<uint16_t> ZeroOffsetElectricalAngle;

        /**
         * The mechanical angle measured by the encoder as a 14-bit raw value
         */
        uint32_t thetaMechanical = 0;

        /**
         * Auto-calibrated encoder direction sign relative to control coordinates.
         * +1 means raw encoder increases with positive electrical rotation.
         */
        TMR<int32_t> dirSign;
        TMR<int32_t> lastDirSign;

        /**
         * @struct ReferenceCurrents
         *
         * A collection of the Iq,ref and Id,ref that are being used by the controller
         *
         * @note Typically, only the Iq,ref should be updated
         */
        struct ReferenceCurrents {
                int32_t Iq_mA = 0;
                int32_t Id_mA = 0;
        };

        /**
         * The ReferenceCurrents object that is used to communicate data between control loops
         */
        ReferenceCurrents Iref{};

        /**
         * Encoder angle at the moment we start the direction check
         */
        uint16_t directionCalibrationThetaStart = 0;

        /**
         * Direction calibration sub-state
         */
        DirectionCalibrationState directionCalibrationState = DirectionCalibrationState::INIT;

        /**
         * Encoder offset calibration sub-state
         */
        OffsetCalibrationState offsetCalibrationState = OffsetCalibrationState::LOCKING;

        /**
         * Function that performs the direction calibration. The logic followed to achieve this is:
         *
         * 1. The startupCalibration() function has already brought the rotor in the [0, π/4] interval, so this is taken
         * for granted
         *
         * 2. Move the rotor at open-loop for a small period of time, neededTicks
         *
         * 3. If the rotor is now in the interval [π/2, 3π/2], then the winding excitation used is CW. Else, the
         * direction of movement is CCW
         *
         * @param dutyCycles
         * @param thetaEncoder
         */
        void directionCalibration(const PhaseDutyCycles& dutyCycles, uint16_t thetaEncoder);

        /**
         * Function that performs the zero-offset calibration. Its goal is to find the zero-offset electrical angle. The
         * procedure followed is:
         *
         * 1. Inject to the motor windings only d-axis current to lock it to a position
         *
         *
         * @param dutyCycles
         * @param thetaEncoder
         */
        void encoderOffsetCalibration(const PhaseDutyCycles& dutyCycles, uint16_t thetaEncoder);

        /**
         * Helper function that wraps the angle inside the [0, 2π] interval. Since the angles are read in raw, it wraps in [0, 16383].
         *
         * Example: θm = 16.380 and θm' = θm + 10 = 16.390 ==wrapAngle(θm')==> Θm' = 6
         * @param angle
         * @return
         */
        [[nodiscard]] static uint16_t wrapAngle(uint32_t angle) {
                constexpr uint16_t EncoderMask = 0x3FFF; /// for AS5047P

                return static_cast<uint16_t>(angle) & EncoderMask;
        }

        [[nodiscard]] static inline int32_t rawToMilliRad(uint16_t rawAngle) noexcept {
                constexpr int32_t TWO_PI_mrad = 6283;
                constexpr int32_t EncoderResolution = 16384;

                const uint32_t wrapped = wrapAngle(rawAngle);
                return static_cast<int32_t>(static_cast<int64_t>(wrapped) * TWO_PI_mrad / EncoderResolution);
        }

        [[nodiscard]] inline uint16_t signedMechanicalRaw(uint16_t rawAngle) const noexcept {
                constexpr uint16_t EncoderResolution = 16384u;
                const uint16_t wrapped = wrapAngle(rawAngle);
                const int8_t encoderDir = (dirSign.read() >= 0) ? 1 : -1;
                return encoderDir >= 0 ? wrapped : wrapAngle(EncoderResolution - wrapped);
        }

        [[nodiscard]] inline int32_t signedMechanical_mrad(uint16_t rawAngle) const noexcept {
                return rawToMilliRad(signedMechanicalRaw(rawAngle));
        }

        [[nodiscard]] inline uint16_t calculateElectricalAngle(uint16_t thetaMech) const {
                const uint16_t thetaSigned = signedMechanicalRaw(thetaMech);
                const uint16_t thetaElectrical = wrapAngle(thetaSigned * PMSM_Config::MotorPolePairs);
                return wrapAngle(thetaElectrical - ZeroOffsetElectricalAngle.read());
        }

        /**
         * Helper function that converts the rad/s to raw/s
         *
         * Example: If ω_target is 2π rad/s, in the algorithm needs to become 16.384 raw/s
         *
         * @param radiansPerSec The rad/s argument to be converted to 14-bit raw/s
         * @return
         */
        static constexpr uint32_t ConvertRadToRaw(float radiansPerSec) {
                constexpr float PI_TO_RAW = 8192.0f;
                constexpr float DT_us = 670.0f; /// in μs

                const float ConversionFloats = DT_us * radiansPerSec * PI_TO_RAW / MathUtilities::PI;

                return static_cast<uint32_t>(ConversionFloats);
        }
};

} // namespace PermanentMagnetSynchronousMotor
