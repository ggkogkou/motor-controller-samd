#include "svpwm.hpp"

namespace SpaceVectorModulation {

DutyCycles SVPWM::compute(int32_t vAlpha, int32_t vBeta) const {
        constexpr int32_t HALF_Q15 = 16384; // 0.5 * 32768
        constexpr int32_t SQRT3_2_Q15 = 28378; // (sqrt(3)/2) * 32768

        int32_t Va = vAlpha;
        int32_t Vb = -(vAlpha >> 1) + ((SQRT3_2_Q15 * vBeta) >> 15);
        int32_t Vc = -(vAlpha >> 1) - ((SQRT3_2_Q15 * vBeta) >> 15);

        const auto [V_Min, V_Max] = findMinMax(Va, Vb, Vc);

        int32_t zeroSequenceComponent = 0;

        if (zeroSequenceModulation == ZeroSequenceModulationType::MIDPOINT_CLAMP)
                zeroSequenceComponent -= (V_Min + V_Max) / 2;

        Va += zeroSequenceComponent;
        Vb += zeroSequenceComponent;
        Vc += zeroSequenceComponent;

        // Va = std::clamp(Va, -dcLinkVoltage, dcLinkVoltage);
        // Vb = std::clamp(Vb, -dcLinkVoltage, dcLinkVoltage);
        // Vc = std::clamp(Vc, -dcLinkVoltage, dcLinkVoltage);

        constexpr int32_t MIN_Q15 = 0;
        constexpr int32_t MAX_Q15 = 32767;

        const int32_t DutyCycleA = std::clamp(HALF_Q15 + Va * inverseVdc_Q15, MIN_Q15, MAX_Q15);
        const int32_t DutyCycleB = std::clamp(HALF_Q15 + Vb * inverseVdc_Q15, MIN_Q15, MAX_Q15);
        const int32_t DutyCycleC = std::clamp(HALF_Q15 + Vc * inverseVdc_Q15, MIN_Q15, MAX_Q15);

        return DutyCycles{DutyCycleA, DutyCycleB, DutyCycleC};
}

} // namespace SpaceVectorModulation
