#include "svpwm.hpp"

namespace SpaceVectorModulation {

    DutyCycles SVPWM::compute(float vAlpha, float vBeta) const {
        float Va = vAlpha;
        float Vb = - 0.5f * vAlpha + MathUtilities::SQRT3_2 * vBeta;
        float Vc = - 0.5f * vAlpha - MathUtilities::SQRT3_2 * vBeta;

        /// TODO: The ZSM categorization
        /// TODO: Log/flag when duty cycle saturates 0 or 1 (for anti-windup)

        const auto [V_Min, V_Max] = std::minmax({Va, Vb, Vc});
        float zeroSequenceComponent = 0.0f;

        if (zeroSequenceModulation == ZeroSequenceModulationType::MIDPOINT_CLAMP)
            zeroSequenceComponent -= 0.5f * (V_Min + V_Max);

        Va = Va + zeroSequenceComponent;
        Vb = Vb + zeroSequenceComponent;
        Vc = Vc + zeroSequenceComponent;

        const float invVDC = 1.0f / dcLinkVoltage;
        auto convertVoltageToDutyCycle = [&](float voltage) -> float {
            return std::clamp(0.5f + (voltage * invVDC), 0.0f, 1.0f);
        };

        const float DutyCycleA = convertVoltageToDutyCycle(Va);
        const float DutyCycleB = convertVoltageToDutyCycle(Vb);
        const float DutyCycleC = convertVoltageToDutyCycle(Vc);

        return DutyCycles{DutyCycleA, DutyCycleB, DutyCycleC};
    }

} /// namespace SpaceVectorModulation
