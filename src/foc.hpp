#pragma once

#include <cmath>
#include <algorithm>
#include <cstdint>
#include "svpwm.hpp"
#include "pid.hpp"
#include "as5047p.hpp"


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
    void algorithmCallback();

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

};

}
