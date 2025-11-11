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

        /**
         *
         * Brushless DC GM4108H-120T Gimbal Motor
         * --------------------------------------
         * Pole pairs: 11
         * No-load current: 0.07±0.1A
         * No-load voltage: 20 V
         * Load torque: 1200-1800 g*cm
         * Motor internal resistance: 11.1±5% Ω
         * No-load RPM: 513-567 RPM @ 20 V => calculate its Kv rating as approximately 25.65-28.35 RPM/V
         *
         */
        static constexpr float MotorKV_Rating = 26.0f;
        static constexpr float MotorInternalResistance = 11.0f;
};

} // namespace PermanentMagnetSynchronousMotor
