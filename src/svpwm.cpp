#include "svpwm.hpp"

namespace SpaceVectorModulation {

static inline int32_t clamp_i32(int32_t x, int32_t lo, int32_t hi) { return (x < lo) ? lo : (x > hi) ? hi : x; }

DutyCycles SVPWM::compute(int16_t vAlpha, int16_t vBeta) const {
        // Q15 constants
        constexpr int32_t HALF_Q15 = 16384; // 0.5 * 32768
        constexpr int32_t SQRT3_2_Q15 = 28378; // (sqrt(3)/2) * 32768

        const int32_t Va0 = static_cast<int32_t>(vAlpha);

        const int32_t Vb0 = static_cast<int32_t>(((static_cast<int64_t>(-HALF_Q15) * static_cast<int32_t>(vAlpha)) +
                                                  (static_cast<int64_t>(SQRT3_2_Q15) * static_cast<int32_t>(vBeta))) >>
                                                 15);

        const int32_t Vc0 = static_cast<int32_t>(((static_cast<int64_t>(-HALF_Q15) * static_cast<int32_t>(vAlpha)) -
                                                  (static_cast<int64_t>(SQRT3_2_Q15) * static_cast<int32_t>(vBeta))) >>
                                                 15);

        // min/max (int32)
        const auto [V_Min, V_Max] = std::minmax({Va0, Vb0, Vc0});

        int32_t zeroSequenceComponent = 0;

        if (zeroSequenceModulation == ZeroSequenceModulationType::MIDPOINT_CLAMP) {
                // zeroSequenceComponent -= 0.5*(V_Min + V_Max)
                zeroSequenceComponent = -((V_Min + V_Max) >> 1);
        }

        const int32_t Va = Va0 + zeroSequenceComponent;
        const int32_t Vb = Vb0 + zeroSequenceComponent;
        const int32_t Vc = Vc0 + zeroSequenceComponent;

        // duty_q15 = 0.5 + V/Vdc  (all integer, Q15 result)
        const int32_t vdc = static_cast<int32_t>(dcLinkVoltage);

        const auto voltageToDutyQ15 = [&](int32_t v_mV) -> int32_t {
                if (vdc <= 0)
                        return 0;

                const int32_t frac_q15 = static_cast<int32_t>((static_cast<int64_t>(v_mV) << 15) / vdc);
                const int32_t duty_q15 = HALF_Q15 + frac_q15;
                return clamp_i32(duty_q15, 0, 32767);
        };

        const int32_t DutyA_q15 = voltageToDutyQ15(Va);
        const int32_t DutyB_q15 = voltageToDutyQ15(Vb);
        const int32_t DutyC_q15 = voltageToDutyQ15(Vc);

        // Convert to float only at the end (keeps your existing code unchanged)
        constexpr float INV_Q15 = 1.0f / 32768.0f;

        return DutyCycles{static_cast<float>(DutyA_q15) * INV_Q15, static_cast<float>(DutyB_q15) * INV_Q15,
                          static_cast<float>(DutyC_q15) * INV_Q15};
}

} // namespace SpaceVectorModulation
