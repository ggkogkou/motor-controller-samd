#include "pid_q31.hpp"

q31_t PID_Q31::compute(q31_t error) {
        const q31_t ProportionalTerm = q31_mul(K_Proportional, error);
        const q31_t DerivativeTerm   = q31_mul(K_Derivative, (error - previousError));

        const q31_t IntegralTerm = [&]() -> q31_t {
                const q31_t sumErr = error + previousError;
                const q31_t incI   = q31_mul(K_Integral, sumErr);
                const q31_t unclamped = previousIntegralTerm + incI;
                return std::clamp(unclamped, -limit, limit);
        }();

        const q31_t PID_Output = [&]() -> q31_t {
                const q31_t unclamped = ProportionalTerm + DerivativeTerm + IntegralTerm;
                return std::clamp(unclamped, -limit, limit);
        }();

        previousError = error;
        previousIntegralTerm = IntegralTerm;
        previousControllerOutput = PID_Output;

        return PID_Output;
}

void PID_Q31::setGainsFloat(float Kp, float Ki, float Kd, float clampLimit, float Ts) {
        dT = Ts;

        K_Proportional = floatToQ31(Kp);
        K_Integral = floatToQ31(Ki * 0.5f * Ts);
        K_Derivative = [&]() -> q31_t {
                if (Ts <= 0.0f)
                        return 0;

                return floatToQ31(Kd / Ts);
        }();

        limit = floatToQ31(clampLimit);
}

void PID_Q31::reset() {
        previousError = 0;
        previousIntegralTerm = 0;
        previousControllerOutput = 0;
}
