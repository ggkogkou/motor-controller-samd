#pragma once

#include <cstdint>

/**
 * Class that implements basic stepper motor
 *
 * TODO: Extend functionality to support servo and speed control (with or without encoder); maybe inheritance (?)
 *
 * The control happens with individual control of high- and low-side MOSFETs/IGBTs using appropriately PWM signals
 */
class StepperMotor {
public:
    enum class DriveType
    {
        FULL_STEPS,
        HALF_STEPS,
        MICROSTEPS_32,
    };

    enum class Direction
    {
        CLOCKWISE,
        COUNTER_CLOCKWISE,
    };

    enum class StepMode
    {
        SINGLE,
        DOUBLE,
        INTERLEAVE,
    };

    StepperMotor() = default;

    StepperMotor(DriveType driveType, float stepAngle) : driveType(driveType), stepAngle(stepAngle) {
        stepsPerRevolution = 360.0f / stepAngle;
    }

    StepperMotor(DriveType driveType, StepMode stepMode, Direction direction) : driveType(driveType),
        stepMode(stepMode), direction(direction) { }

    void setDriveType(DriveType driveType) {
        this->driveType = driveType;
    }

    void setDirection(Direction direction) {
        this->direction = direction;
    }

    /**
     * Function that controls the speed of rotor rotation
     *
     * @param targetSpeed The targeted speed (in RPS)
     */
    void setAngularSpeedRPS(float targetSpeed) {

    }

    /**
     * Function that controls the speed of rotor rotation
     *
     * @param targetSpeed The targeted speed (in RPM)
     */
    void setAngularSpeedRPM(float targetSpeed) {

    }

    /**
     * Function that commands the rotor to move for a user-specified number of steps
     *
     * @param numberOfSteps The number of steps that the rotor should move
     */
    void moveSteps(uint32_t numberOfSteps) {

    }

    /**
     * Function that reverses the direction without the need to define a CW/CCW reference
     */
    void reversDirection() {

    }

private:
    DriveType driveType = DriveType::FULL_STEPS;
    StepMode stepMode = StepMode::SINGLE;
    Direction direction = Direction::CLOCKWISE;

    /**
     * Step angle in degrees
     */
    float stepAngle = 1.8f;

    /**
     * Number of steps per revolution; derived from step angle (1 rev = 360 deg)
     */
    float stepsPerRevolution = 200.0f;

    /**
     * The current (in Amps) that brings the motor into its thermal acceptable limits
     *
     * @note The value should be specified at the motor's datasheet
     */
    float nominalCurrent = 0.0f;

    /**
     * The motor's rated voltage; the voltage that will induce the nominal current
     *
     * @note The value should be specified at the motor's datasheet; depends on RL parameters
     */
    float motorRatedVoltage = 0.0f;

    /**
     * The power supply that powers the motor; more specifically the two H-Bridge circuits
     */
    float motorSupplyVotage = 0.0f;
};
