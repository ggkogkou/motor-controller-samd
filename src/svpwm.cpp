#include "svpwm.hpp"

namespace SpaceVectorModulation {

// Q15 clamp: 0..32767 (where 32768 would be 1.0)
static inline uint16_t clamp_u16_q15(int32_t x) {
        if (x < 0)
                return 0;
        if (x > 32767)
                return 32767;
        return static_cast<uint16_t>(x);
}

static inline int32_t min3(int32_t a, int32_t b, int32_t c) {
        int32_t m = (a < b) ? a : b;
        return (m < c) ? m : c;
}

static inline int32_t max3(int32_t a, int32_t b, int32_t c) {
        int32_t m = (a > b) ? a : b;
        return (m > c) ? m : c;
}

static inline int32_t clamp_i32(int32_t x, int32_t lo, int32_t hi) {
        if (x < lo)
                return lo;
        if (x > hi)
                return hi;
        return x;
}

DutyCyclesQ15 SVPWM::computeQ15(int16_t vAlpha, int16_t vBeta) const {
        // Q15 constants
        constexpr int32_t HALF_Q15 = 16384; // 0.5 * 32768
        constexpr int32_t SQRT3_2_Q15 = 28378; // (sqrt(3)/2) * 32768

        const int32_t vdc = static_cast<int32_t>(dcLinkVoltage);
        if (vdc <= 0) {
                return DutyCyclesQ15{0, 0, 0};
        }

        // Reciprocal in Q15: invVdc_q15 = (1<<15)/vdc
        // NOTE: still one divide per call.
        const int32_t invVdc_q15 = (1 << 15) / vdc;

        const int32_t Va0 = static_cast<int32_t>(vAlpha);

        // -0.5*Valpha is just shift (arithmetic shift is fine here)
        const int32_t minus_half_a = -(Va0 >> 1);

        // (sqrt(3)/2)*Vbeta in Q15: (SQRT3_2_Q15 * vBeta) >> 15
        // Safe in int32 because vBeta is int16.
        const int32_t kbeta = (SQRT3_2_Q15 * static_cast<int32_t>(vBeta)) >> 15;

        const int32_t Vb0 = minus_half_a + kbeta;
        const int32_t Vc0 = minus_half_a - kbeta;

        int32_t Va = Va0;
        int32_t Vb = Vb0;
        int32_t Vc = Vc0;

        if (zeroSequenceModulation == ZeroSequenceModulationType::MIDPOINT_CLAMP) {
                const int32_t vmin = min3(Va0, Vb0, Vc0);
                const int32_t vmax = max3(Va0, Vb0, Vc0);
                const int32_t v0 = -((vmin + vmax) >> 1);
                Va += v0;
                Vb += v0;
                Vc += v0;
        } else {
                // Other ZSM modes not implemented here (kept identical to MIDPOINT_CLAMP use-case).
        }

        // IMPORTANT:
        // Clamp phase voltages to +/-Vdc so Va*invVdc_q15 cannot overflow int32.
        Va = clamp_i32(Va, -vdc, vdc);
        Vb = clamp_i32(Vb, -vdc, vdc);
        Vc = clamp_i32(Vc, -vdc, vdc);

        // duty_q15 = 0.5 + V/Vdc
        // with invVdc_q15 = (1<<15)/Vdc, the term V * invVdc_q15 is Q15-scaled
        const int32_t dutyA = HALF_Q15 + (Va * invVdc_q15);
        const int32_t dutyB = HALF_Q15 + (Vb * invVdc_q15);
        const int32_t dutyC = HALF_Q15 + (Vc * invVdc_q15);

        return DutyCyclesQ15{
                clamp_u16_q15(dutyA),
                clamp_u16_q15(dutyB),
                clamp_u16_q15(dutyC),
        };
}

DutyCycles SVPWM::compute(int16_t vAlpha, int16_t vBeta) const {
        const auto dc = computeQ15(vAlpha, vBeta);
        constexpr float INV_Q15 = 1.0f / 32768.0f;

        return DutyCycles{
                static_cast<float>(dc.dutyA_q15) * INV_Q15,
                static_cast<float>(dc.dutyB_q15) * INV_Q15,
                static_cast<float>(dc.dutyC_q15) * INV_Q15,
        };
}

} // namespace SpaceVectorModulation
