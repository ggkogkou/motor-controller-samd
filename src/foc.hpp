#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include "as5047p.hpp"
#include "definitions.h"
#include "pid.hpp"
#include "svpwm.hpp"

namespace FieldOrientedControl {

using namespace SpaceVectorModulation;

class FOC {
public:
        /**
         * Default constructor for FOC class
         */
        FOC() = default;

        /**
         * Function that implements the Field-Oriented Control
         *
         * @brief The various building blocks are combined and implement the full control loop. It will be serve the
         * purpose of the ISR that will occur at the end of the period of each PWM cycle
         */
        void algorithmCallback(TC_TIMER_STATUS status, uintptr_t context);

        /**
         * Function that performs the initial encoder offset calibration for the encoder
         *
         * Currently supported: absolute encoder via SPI
         *
         */
        void encoderOffsetCalibration();

private:
        /**
         * The SVPWM building block object
         */
        SVPWM spaceVectorPWM;

        /**
         * The PI controller that is used for the closed-loop control
         */
        PID pidIq;
        PID pidId;

        /**
         * The encoder driver instance
         */
        AS5047P as5047p;

        static constexpr float EncoderAlignmentVoltageLimit = 2.0f;
};

} // namespace FieldOrientedControl
