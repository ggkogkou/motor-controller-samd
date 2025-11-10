#pragma once

#include "math_utils.hpp"

namespace PermanentMagnetSynchronousMotor {

using namespace MathUtilities;

struct PMSM_Config {
        /**
         * The DC link voltage
         */
        static constexpr float DCLinkVoltage = 20.0f;

        /**
         * The voltage limit -- DC bus utilization
         */
        static constexpr float CloseLoopVoltageLimit = 10.0f;

        /**
         * Encoder electrical offset and direction calibration voltage limit
         */
        static constexpr float InitialCalibrationVoltageLimit = 3.0f;

        /**
         * Target velocity for the outer velocity loop
         */
        static constexpr float TargetVelocity = 12.0f;

        /**
         * Target velocity for the encoder calibration loop (ω = 2π rad/s)
         */
        static constexpr float TargetCalibrationVelocity = TWO_PI;

        /**
         * The motor's pole pairs
         */
        static constexpr float MotorPolePairs = 11.0f;

        static constexpr float OpenLoopVoltageLimit = InitialCalibrationVoltageLimit;
};

}
