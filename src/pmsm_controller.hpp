#pragma once

#include "math_utils.hpp"
#include "pid.hpp"
#include "svpwm.hpp"

namespace PermanentMagnetSynchronousMotor {

using namespace SpaceVectorModulation;
using namespace MathUtilities;

struct PMSM_Config {
        /**
         * The DC link voltage
         */
        static constexpr float DCLinkVoltage = 20.0f;

        /**
         * The voltage limit -- DC bus utilization
         */
        static constexpr float GlobalVoltageLimit = 12.0f;

        /**
         * Encoder electrical offset and direction calibration voltage limit
         */
        static constexpr float InitialCalibrationVoltageLimit = 3.0f;

        /**
         * Target velocity for the outer velocity loop
         */
        static constexpr float TargetVelocity = 15.0f;

        /**
         * Target velocity for the encoder calibration loop (ω = 2π rad/s)
         */
        static constexpr float TargetCalibrationVelocity = TWO_PI;

        /**
         * The motor's pole pairs
         */
        static constexpr float MotorPolePairs = 11.0f;
};

class PMSM_Controller {
public:
        PMSM_Controller() = default;

        explicit PMSM_Controller(const PMSM_Config) {}

        /**
         * The main update function that implements the Field-Oriented Control
         */
        void update(float &dutyA, float &dutyB, float &dutyC);

        /**
         * Function that performs the initial encoder offset and direction calibration
         */
        void performStartupCalibration();

private:
        /**
         * The Space Vector PWM block
         */
        SVPWM pwm{PMSM_Config::DCLinkVoltage, ZeroSequenceModulationType::MIDPOINT_CLAMP};

        /**
         * The outer velocity control loop PI controller
         */
        PID pidVelocity {0.5f, 10.0f, 0.0f, 6.0f, 0.00100000005};

        /**
         * The direct (d-axis) current PI controller Id
         */
        PID pidId = PID{0.25f, 20.0f, 0.0f, PMSM_Config::GlobalVoltageLimit, 0.00100000005 };

        /**
         * The quadrature (q-axis) current PI controller Iq
         */
        PID pidIq = PID{0.35f, 50.0f, 0.0f, PMSM_Config::GlobalVoltageLimit, 0.00100000005 };

        /**
         * Represents the possible directions of rotation
         */
        enum class Direction : int8_t {
                CLOCKWISE = 1,
                COUNTERCLOCKWISE = -1,
        };

        /**
         * Rotor's direction; must be overrriden by the startup calbration procedures
         */
        Direction direction = Direction::CLOCKWISE;

        /**
         * The zero-offset electrical angle; must be updated by the startup calibration procedures
         */
        float ZeroOffsetElectricalAngle = 0.0f;

};

} // namespace PermanentMagnetSynchronousMotor
