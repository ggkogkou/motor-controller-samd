#pragma once

#include <cstdint>
#include "TMR.hpp"
#include "svpwm.hpp"

namespace PermanentMagnetSynchronousMotor {

using namespace SpaceVectorModulation;

struct PhaseDutyCycles;

class StartupCalibration {
public:
        /**
         * Class constructor
         */
        explicit StartupCalibration(TMR<uint32_t>& pwmPeriod, TMR<uint16_t>& ZeroOffsetElectricalAngle, TMR<int32_t>& dirSign,
                                    uint32_t velocityLoopPeriod);

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

private:
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
         * The Space Vector PWM block
         */
        SVPWM pwm{14'000, ZeroSequenceModulationType::MIDPOINT_CLAMP};

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
         * The PWM period, either in counter-ticks or μs
         */
        TMR<uint32_t>& pwmPeriod;

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
         * Zero-offset electrical angle in raw 14-bit format
         */
        TMR<uint16_t>& ZeroOffsetElectricalAngle;

        /**
         * The mechanical angle measured by the encoder as a 14-bit raw value
         */
        uint32_t thetaMechanical = 0;

        /**
         * Auto-calibrated encoder direction sign relative to control coordinates.
         * +1 means raw encoder increases with positive electrical rotation.
         */
        TMR<int32_t>& dirSign;

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
        [[nodiscard]] static uint16_t wrapAngle(uint32_t angle);

        [[nodiscard]] uint16_t signedMechanicalRaw(uint16_t rawAngle) const noexcept;
};

} // namespace PermanentMagnetSynchronousMotor
