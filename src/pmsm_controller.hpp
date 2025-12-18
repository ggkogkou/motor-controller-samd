#pragma once

#include <cmath>
#include <cstdint>
#include <optional>
#include "definitions.h"
#include "math_utils.hpp"   // contains MathUtils fixed-point transforms + LUTs
#include "pid.hpp"
#include "pid_q31.hpp"
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
        int32_t lastWrappedAngle;      // mrad
        int32_t unwrappedAngle;        // mrad
        int32_t angularVelocity;       // mrad/sec
        uint32_t filterTimeConstant;   // usec

        /**
         * Usefull constants
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
            lastWrappedAngle(initialWrappedAngle),
            unwrappedAngle(initialWrappedAngle),
            angularVelocity(0),
            filterTimeConstant(tau) {}

        /**
         * Update function that runs in the velocity control loop
         *
         * @param wrappedAngle The wrapped angle (in mrad)
         * @param deltaTime The change of time dT (in μsec)
         */
        void update(int32_t wrappedAngle, uint32_t deltaTime) {
                const auto wrapDelta = [&](int32_t a, int32_t b) -> int32_t {
                        int32_t d = a - b;
                        if (d > PI_MRAD)  d -= TWO_PI_MRAD;
                        if (d < -PI_MRAD) d += TWO_PI_MRAD;
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
                        if (a < 0)     return 0;
                        if (a > 32768) return 32768;
                        return static_cast<int32_t>(a);
                };

                const int32_t delta = wrapDelta(wrappedAngle, lastWrappedAngle);
                unwrappedAngle += delta;

                const int32_t rawDerivative = derivative_mrad_per_sec(delta, deltaTime);

                const int32_t a_q15 = alpha_q15(filterTimeConstant, deltaTime);
                const int32_t one_minus_a_q15 = 32768 - a_q15;

                const auto filt =
                        static_cast<int64_t>(a_q15) * static_cast<int64_t>(angularVelocity) +
                        static_cast<int64_t>(one_minus_a_q15) * static_cast<int64_t>(rawDerivative);

                angularVelocity = static_cast<int32_t>(filt >> 15);
                lastWrappedAngle = wrappedAngle;
        }
};

class PMSM_Controller {
public:
        PMSM_Controller() {
                q31pidVelocity.setGainsFloat(0.5f, 10.0f, 0.0f, 6.0f, 0.00100000005f);
                q31pidId.setGainsFloat(0.25f, 20.0f, 0.0f, PMSM_Config::CloseLoopVoltageLimit, 0.00100000005f);
                q31pidIq.setGainsFloat(0.35f, 50.0f, 0.0f, PMSM_Config::CloseLoopVoltageLimit, 0.00100000005f);
        }

        explicit PMSM_Controller(const PMSM_Config) {}

        // thetaEncoder is now raw 14-bit counts (0..16383)
        void update(const PhaseCurrents& phaseCurrents, const PhaseDutyCycles& dutyCycles, uint16_t thetaEncoder);

        void updateVelocity(const PhaseCurrents& phaseCurrents, const PhaseDutyCycles& dutyCycles, uint16_t thetaEncoder);

        void updateOpenLoop(const PhaseDutyCycles& dutyCycles);

        /**
         * Function that performs the initial encoder offset and direction calibration
         */
        bool startupCalibration(const PhaseDutyCycles& dutyCycles, uint16_t thetaEncoder);

        void stopMotor(const PhaseDutyCycles& dutyCycles) const;

private:
        /**
         * The Space Vector PWM block
         */
        SVPWM pwm{20'000, ZeroSequenceModulationType::MIDPOINT_CLAMP};

        /**
         * The outer velocity control loop PI controller (float in rad/s -> outputs A)
         */
        PID<float> pidVelocity{0.5f, 10.0f, 0.0f, 6.0f, 0.001f};

        /**
         * Current PI controllers: inputs in mA, outputs in mV
         */
        PID<int32_t> pidId{0.25f, 20.0f, 0.0f, static_cast<float>(PMSM_Config::CloseLoopVoltageLimit * 1000.0f), 0.001f};
        PID<int32_t> pidIq{0.35f, 50.0f, 0.0f, static_cast<float>(PMSM_Config::CloseLoopVoltageLimit * 1000.0f), 0.001f};

        PID_Q31 q31pidVelocity;
        PID_Q31 q31pidId;
        PID_Q31 q31pidIq;

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
         * Zero-offset electrical angle in raw 14-bit electrical counts
         */
        uint16_t ZeroOffsetElectricalAngle14 = 0u;

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

        void directionCalibration(const PhaseDutyCycles& dutyCycles, uint16_t thetaEncoder);
        void encoderOffsetCalibration(const PhaseDutyCycles& dutyCycles, uint16_t thetaEncoder);

        // Mechanical angle used by open-loop (raw 14-bit counts)
        uint16_t thetaMechanical14 = 0u;

        uint32_t pwmPeriod = 1000;

        uint32_t timerCounter = 0;
        uint32_t neededTicks = 250; // a quarter

        std::optional<AngleVelocityEstimator> velocityEstimator;

        float dT = 0.001f;
};

} // namespace PermanentMagnetSynchronousMotor
