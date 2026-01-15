#include "pid.hpp"

int32_t PID::compute(int32_t error) {
        const int32_t ProportionalTerm = K_Proportional * error;
        const int32_t DerivativeTerm = K_Derivative * (error - previousError);

        const auto IntegralTerm = [&]() -> int32_t {
                const int32_t sumErr = error + previousError;
                const int32_t incI = K_Integral * sumErr;
                const int32_t IntegralTermUnclamped = previousIntegralTerm + incI;
                return std::clamp(IntegralTermUnclamped, -limit, static_cast<int32_t>(limit));
        }();

        const auto PID_Output = [&]() -> int32_t {
                const int32_t PID_OutputUnclamped = ProportionalTerm + DerivativeTerm + IntegralTerm;
                return std::clamp(PID_OutputUnclamped, -limit, static_cast<int32_t>(limit));
        }();

        previousError = error;
        previousIntegralTerm = IntegralTerm;
        previousControllerOutput = PID_Output;

        return PID_Output;
}

void PID::setGainsFloat(float Kp, float Ki, float Kd, float clampLimit, float Ts) {
        limit = static_cast<int32_t>(clampLimit);
        dT = Ts;

        K_Proportional = Q16_t(Kp);
        K_Integral = Q16_t(Ki * 0.5f * Ts);
        K_Derivative = (Ts > 0.0f) ? Q16_t(Kd / Ts) : Q16_t(0.0f);
}

void PID::reset() {
        previousError = 0;
        previousIntegralTerm = 0;
        previousControllerOutput = 0;
}
