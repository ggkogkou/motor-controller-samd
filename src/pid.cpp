#include "pid.hpp"

float PID::compute(float error) {
    constexpr float timeNow = 100.0f;

    const auto dT = [&]() -> float {
        const float DT = timeNow - previousTimestamp;
        return DT <= 0.0f ? 1e-6f : DT;
    }();

    const float ProportionalTerm = K_Proportional * error;
    const float DerivativeTerm = K_Derivative * (error - previousError) / dT;
    const auto IntegralTerm = [&]() -> float {
        const float IntegralTermUnclamped = previousIntegralTerm + K_Integral * 0.5f * dT * (error + previousError);
        return std::clamp(IntegralTermUnclamped, -limit, limit);
    }();

    const auto PID_Output = [&]() -> float {
        const float PID_OutputUnclamped = ProportionalTerm + DerivativeTerm + IntegralTerm;
        return std::clamp(PID_OutputUnclamped, -limit, limit);
    }();

    previousError = error;
    previousTimestamp = timeNow;
    previousIntegralTerm = IntegralTerm;
    previousControllerOutput = PID_Output;

    return PID_Output;
}
