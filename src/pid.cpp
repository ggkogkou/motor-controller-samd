#include "pid.hpp"

template<typename T, int GAIN_Q>
PID<T, GAIN_Q>::PID(float Kp, float Ki, float Kd, float limit, float Ts) {
        setGainsFloat(Kp, Ki, Kd, limit, Ts);
}

template<typename T, int GAIN_Q>
T PID<T, GAIN_Q>::compute(T error) {
        if constexpr (std::is_floating_point_v<T>) {
                const T ProportionalTerm = K_Proportional * error;
                const T DerivativeTerm = (dT > 0.0f) ? (K_Derivative * (error - previousError) / static_cast<T>(dT)) : T{0};

                const auto IntegralTerm = [&]() -> T {
                        const T IntegralTermUnclamped =
                                previousIntegralTerm + K_Integral * static_cast<T>(0.5f) * static_cast<T>(dT) * (error + previousError);
                        return std::clamp(IntegralTermUnclamped, -limit, limit);
                }();

                const auto PID_Output = [&]() -> T {
                        const T PID_OutputUnclamped = ProportionalTerm + DerivativeTerm + IntegralTerm;
                        return std::clamp(PID_OutputUnclamped, -limit, limit);
                }();

                previousError = error;
                previousIntegralTerm = IntegralTerm;
                previousControllerOutput = PID_Output;

                return PID_Output;
        } else {
                const T ProportionalTerm = mulQ(K_Proportional, error);
                const T DerivativeTerm   = mulQ(K_Derivative, static_cast<T>(error - previousError));

                const auto IntegralTerm = [&]() -> T {
                        const T sumErr = static_cast<T>(error + previousError);
                        const T incI   = mulQ(K_Integral, sumErr);
                        const T IntegralTermUnclamped = static_cast<T>(previousIntegralTerm + incI);
                        return std::clamp(IntegralTermUnclamped, static_cast<T>(-limit), static_cast<T>(limit));
                }();

                const auto PID_Output = [&]() -> T {
                        const T PID_OutputUnclamped =
                                static_cast<T>(ProportionalTerm + DerivativeTerm + IntegralTerm);
                        return std::clamp(PID_OutputUnclamped, static_cast<T>(-limit), static_cast<T>(limit));
                }();

                previousError = error;
                previousIntegralTerm = IntegralTerm;
                previousControllerOutput = PID_Output;

                return PID_Output;
        }
}

template<typename T, int GAIN_Q>
void PID<T, GAIN_Q>::setGainsFloat(float Kp, float Ki, float Kd, float clampLimit, float Ts) {
        limit = static_cast<T>(clampLimit);
        dT = Ts;

        if constexpr (std::is_floating_point_v<T>) {
                K_Proportional = static_cast<T>(Kp);
                K_Integral     = static_cast<T>(Ki);
                K_Derivative   = static_cast<T>(Kd);
        } else {
                // Same discrete form as your float PID:
                // Integral increment uses Ki * 0.5 * Ts * (e + e_prev)
                // Derivative uses Kd/Ts * (e - e_prev)
                K_Proportional = gainToQ(Kp);
                K_Integral     = gainToQ(Ki * 0.5f * Ts);
                K_Derivative   = (Ts > 0.0f) ? gainToQ(Kd / Ts) : 0;
        }
}

template<typename T, int GAIN_Q>
void PID<T, GAIN_Q>::reset() {
        previousError = 0;
        previousIntegralTerm = 0;
        previousControllerOutput = 0;
}

template<typename T, int GAIN_Q>
int32_t PID<T, GAIN_Q>::gainToQ(float g) {
        return static_cast<int32_t>(g * static_cast<float>(1u << GAIN_Q));
}

template<typename T, int GAIN_Q>
T PID<T, GAIN_Q>::mulQ(int32_t gainQ, T x) {
        const int64_t p = static_cast<int64_t>(gainQ) * static_cast<int64_t>(x);
        return static_cast<T>(p >> GAIN_Q);
}

// ---- Explicit instantiations (so this template can live in a .cpp) ----
template class PID<float, 16>;
template class PID<int32_t, 16>;
