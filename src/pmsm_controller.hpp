#pragma once

#include <cstdint>
#include <optional>
#include "definitions.h"
#include "math_utils.hpp"
#include "pid.hpp"
#include "pmsm_config.hpp"
#include "svpwm.hpp"

namespace PermanentMagnetSynchronousMotor {

using namespace SpaceVectorModulation;

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
        int32_t lastWrappedAngle; // mrad
        int32_t unwrappedAngle; // mrad
        int32_t angularVelocity; // mrad/sec
        uint32_t filterTimeConstant; // usec

        /**
         * Useful constants
         */
        static constexpr int32_t TWO_PI_MRAD = 6283;
        static constexpr int32_t PI_MRAD = TWO_PI_MRAD / 2;

        /**
         * Class constructor
         *
         * @param initialWrappedAngle The initial wrapped angle (in mrad)
         * @param tau The change of time dT (in μsec)
         */
        explicit AngleVelocityEstimator(int32_t initialWrappedAngle, uint32_t tau = 10'000) :
            lastWrappedAngle(initialWrappedAngle), unwrappedAngle(initialWrappedAngle), angularVelocity(0), filterTimeConstant(tau) {}

        /**
         * Update function that runs in the velocity control loop
         *
         * @param wrappedAngle The wrapped angle (in mrad)
         * @param deltaTime The change of time dT (in μsec)
         */
        void update(int32_t wrappedAngle, uint32_t deltaTime) {
                const auto wrapDelta = [&](int32_t a, int32_t b) -> int32_t {
                        int32_t d = a - b;
                        if (d > PI_MRAD)
                                d -= TWO_PI_MRAD;
                        if (d < -PI_MRAD)
                                d += TWO_PI_MRAD;
                        return d;
                };

                const auto derivative_mrad_per_sec = [&](int32_t delta, uint32_t dt_us) -> int32_t {
                        if (dt_us == 0u)
                                return 0;
                        const auto num = static_cast<int64_t>(delta) * 1'000'000LL;
                        return static_cast<int32_t>(num / static_cast<int64_t>(dt_us));
                };

                const auto alpha_q15 = [&](uint32_t tau_us, uint32_t dt_us) -> int32_t {
                        const uint32_t Denominator = tau_us + dt_us;
                        if (Denominator == 0u)
                                return 0;
                        const auto a = (static_cast<int64_t>(tau_us) << 15) / static_cast<int64_t>(Denominator);
                        if (a < 0)
                                return 0;
                        if (a > 32768)
                                return 32768;
                        return static_cast<int32_t>(a);
                };

                const int32_t delta = wrapDelta(wrappedAngle, lastWrappedAngle);
                unwrappedAngle += delta;

                const int32_t rawDerivative = derivative_mrad_per_sec(delta, deltaTime);

                const int32_t a_q15 = alpha_q15(filterTimeConstant, deltaTime);
                const int32_t one_minus_a_q15 = 32768 - a_q15;

                const auto filt = static_cast<int64_t>(a_q15) * static_cast<int64_t>(angularVelocity) +
                        static_cast<int64_t>(one_minus_a_q15) * static_cast<int64_t>(rawDerivative);

                angularVelocity = static_cast<int32_t>(filt >> 15);
                lastWrappedAngle = wrappedAngle;
        }
};

class PMSM_Controller {
public:
        /**
         * Class constructor
         */
        PMSM_Controller();

        explicit PMSM_Controller(const PMSM_Config) {}

        /**
         * Function that runs the velocity and current loops
         *
         * @param phaseCurrents
         * @param dutyCycles
         * @param thetaEncoder
         */
        void updateVelocity(const PhaseCurrents& phaseCurrents, const PhaseDutyCycles& dutyCycles, uint16_t thetaEncoder);

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
         * and represented as a signed integer value with >0 meaning CW and <0 meaning CCW
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

private:
        /**
         * The Space Vector PWM block
         */
        SVPWM pwm{18'000, ZeroSequenceModulationType::MIDPOINT_CLAMP};

        /**
         * Velocity PI: error in mrad/s, output in mA
         */
        PID pidVelocity{0.5f, 10.0f, 0.0f, 6000.0f, 0.001f};

        /**
         * Current loop PI controllers (for Iq and Id)
         * Inputs in mA, outputs in mV
         *
         * TODO: Tune the P, I parameters
         */
        PID pidId{0.25f, 20.0f, 0.0f, PMSM_Config::CloseLoopVoltageLimit * 1000.0f, 0.001f};
        PID pidIq{0.35f, 50.0f, 0.0f, PMSM_Config::CloseLoopVoltageLimit * 1000.0f, 0.001f};

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

        /**
         * The calibration state.
         * @note This variable is used to communicate the stage of the calibration between functions.
         */
        CalibrationState calibrationState = CalibrationState::PREPARING;

        // open-loop step in encoder counts per tick (computed once in ctor)
        uint16_t openLoopStepCounts14 = 1u;

        /**
         * The target angular velocity in mrad/s
         */
        int32_t targetVelocity_mrad_s = 0;

        /**
         * The PWM period -- ATSAMD21 specific
         *
         * @note Replace this and make the calculations outside this class
         */
        uint32_t pwmPeriod = 1000;

        /**
         * Helper variable to count the number of ISRs that have been executed.
         *
         * That keeps track of the position in a very inefficient and inaccurate way.
         *
         * @note Find a better alternative for this
         */
        uint32_t timerCounter = 0;

        /**
         * How many ticks to run during calibration (?)
         */
        static constexpr uint32_t MoveDuringCalibrationTicks = 250;

        std::optional<AngleVelocityEstimator> velocityEstimator;

        float dT = 0.0007f;

        /**
         * Zero-offset electrical angle in raw 14-bit format
         */
        uint16_t ZeroOffsetElectricalAngle = 0;

        /**
         * The mechanical angle measured by the encoder as a 14-bit raw value
         */
        uint32_t thetaMechanical = 0;

        /**
         * Direction sign
         * --------------
         * CW = +1,
         * CCW = -1
         */
        int8_t dirSign = 1;

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

        [[nodiscard]] inline uint16_t calculateElectricalAngle(uint16_t thetaMech) const {
                constexpr uint32_t EncoderResolution = 16384u;

                uint16_t thetaElectrical = wrapAngle(thetaMech * PMSM_Config::MotorPolePairs);

                if (dirSign < 0)
                        thetaElectrical = wrapAngle(EncoderResolution - thetaElectrical);

                return wrapAngle(thetaElectrical - ZeroOffsetElectricalAngle);
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
