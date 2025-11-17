#pragma once

#include <cmath>
#include <optional>
#include "definitions.h"
#include "math_utils.hpp"
#include "pid.hpp"
#include "svpwm.hpp"
#include "pmsm_config.hpp"

namespace PermanentMagnetSynchronousMotor {

using namespace SpaceVectorModulation;
using namespace MathUtilities;

struct PhaseCurrents {
        float Ia = 0.0f;
        float Ib = 0.0f;
        float Ic = 0.0f;
};

struct PhaseDutyCycles {
        uint32_t& perA;
        uint32_t& perB;
        uint32_t& perC;

        PhaseDutyCycles(uint32_t& a, uint32_t& b, uint32_t& c) : perA(a), perB(b), perC(c) {}
};

struct AngleVelocityEstimator {
        float lastWrappedAngle;
        float unwrappedAngle;
        float angularVelocity;
        float filterTimeConstant;

        explicit AngleVelocityEstimator(float initialWrappedAngle, float tau = 0.010f) noexcept :
            lastWrappedAngle(initialWrappedAngle), unwrappedAngle(initialWrappedAngle), angularVelocity(0.0f),
            filterTimeConstant(tau) {}

        void update(float wrappedAngle, float deltaTime) noexcept {
                const float delta = std::remainderf(wrappedAngle - lastWrappedAngle, TWO_PI);

                unwrappedAngle += delta;

                const float rawDerivative = delta / deltaTime;
                const float a = filterTimeConstant / (filterTimeConstant + deltaTime);
                angularVelocity = a * angularVelocity + (1.0f - a) * rawDerivative;

                lastWrappedAngle = wrappedAngle;
        }
};

class PMSM_Controller {
public:
        PMSM_Controller() = default;

        explicit PMSM_Controller(const PMSM_Config) {}

        void update(const PhaseCurrents& phaseCurrents, const PhaseDutyCycles& dutyCycles, float thetaEncoder);

        void updateVelocity(const PhaseCurrents& phaseCurrents, const PhaseDutyCycles& dutyCycles, float thetaEncoder);

        void updateOpenLoop(const PhaseDutyCycles& dutyCycles);

        /**
         * Function that performs the initial encoder offset and direction calibration
         */
        bool startupCalibration(const PhaseDutyCycles& dutyCycles, float thetaEncoder);

        void stopMotor(const PhaseDutyCycles& dutyCycles) const;

private:
        /**
         * The Space Vector PWM block
         */
        SVPWM pwm{PMSM_Config::DCLinkVoltage, ZeroSequenceModulationType::MIDPOINT_CLAMP};

        /**
         * The outer velocity control loop PI controller
         */
        PID pidVelocity{0.5f, 10.0f, 0.0f, 6.0f, 0.00100000005};

        /**
         * The direct (d-axis) current PI controller Id
         */
        PID pidId{0.25f, 20.0f, 0.0f, PMSM_Config::CloseLoopVoltageLimit, 0.00100000005};

        /**
         * The quadrature (q-axis) current PI controller Iq
         */
        PID pidIq{0.35f, 50.0f, 0.0f, PMSM_Config::CloseLoopVoltageLimit, 0.00100000005};

        float dirSign = 1.0f;

        /**
         * Represents the possible directions of rotation
         */
        enum class Direction : int8_t {
                CLOCKWISE = 1,
                COUNTERCLOCKWISE = -1,
        };

        enum class ControlType : int8_t {
                OPEN_LOOP,
                CLOSED_LOOP,
        };

        ControlType controlType = ControlType::OPEN_LOOP;

        /**
         * Rotor's direction; must be overrriden by the startup calbration procedures
         */
        Direction direction = Direction::CLOCKWISE;

        /**
         * The zero-offset electrical angle; must be updated by the startup calibration procedures
         */
        float ZeroOffsetElectricalAngle = 0.0f;

        Direction calibrationDirection = Direction::CLOCKWISE;

        enum class CalibrationState : int8_t {
                IDLE,
                PREPARING,
                OFFSET_CALIBRATION,
                DIRECTION_CALIBRATION,
                DONE,
        };

        CalibrationState calibrationState = CalibrationState::PREPARING;

        enum class DirectionCalibrationState : int8_t {
                NOT_DONE,
                CALIBRATE_CW,
                CALIBRATE_CCW,
                DONE,
        };

        DirectionCalibrationState directionCalibrationState = DirectionCalibrationState::CALIBRATE_CW;
        void directionCalibration(const PhaseDutyCycles& dutyCycles, float thetaEncoder);

        void encoderOffsetCalibration(const PhaseDutyCycles& dutyCycles, float thetaEncoder);

        float thetaMechanical = 0.0f;

        uint32_t pwmPeriod = 1000;

        uint32_t timerCounter = 0;
        uint32_t neededTicks = 250; // a quarter

        std::optional<AngleVelocityEstimator> velocityEstimator;


        float dT = 0.001f;
};


} // namespace PermanentMagnetSynchronousMotor
