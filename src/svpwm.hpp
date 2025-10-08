#pragma once

#include <cstdint>
#include <array>
#include <cmath>

namespace ZeroSequenceModulation {
/**
 * \brief Space Vector PWM generator (centered, symmetric sequence)
 *
 * Computes 3-phase PWM duty cycles from a desired space vector either in
 * alpha-beta (Vα,Vβ) or d-q (Vd,Vq + θe) form. Outputs can be timer ticks.
 *
 * Usage:
 *  - Configure with PWM period ticks and optional bus voltage normalization.
 *  - Call computeFromAB or computeFromDQ each PWM period.
 *  - Write returned duties to your PWM peripheral.
 */
class SVPWM {
public:
    /**
     * \brief Returned duties (timer counts)
     */
    struct DutyCycles {
        uint32_t dutyA;
        uint32_t dutyB;
        uint32_t dutyC;
    };

    /**
     * \brief Constructor
     * \param period_ticks PWM timer period in ticks (counts per PWM period)
     * \param vbus_dc      DC bus voltage for normalization (set 1.0f if normalized inputs)
     */
    explicit SVPWM(uint32_t period_ticks, float vbus_dc = 1.0f)
        : vbus(vbus_dc), periodT(period_ticks) {}

    /**
     * \brief Default constructor (call setters before use)
     */
    SVPWM() = default;

    ~SVPWM() = default;
    SVPWM(const SVPWM&) = delete;
    SVPWM& operator=(const SVPWM&) = delete;

    /**
     * \brief Set PWM period in timer ticks
     */
    void setPeriod(uint32_t period_ticks) { periodT = period_ticks; }

    /**
     * \brief Get PWM period in timer ticks
     */
    [[nodiscard]] uint32_t period() const { return periodT; }

    /**
     * \brief Set DC bus voltage (or 1.0f for normalized inputs)
     */
    void setVbus(float vbus_dc) { vbus = vbus_dc; }

    /**
     * \brief Get DC bus voltage
     */
    [[nodiscard]] float getVbus() const { return vbus; }

    /**
     * \brief Set minimum and maximum on-time clamp in ticks
     * \param min_on Minimum compare value (e.g., deadtime equivalent)
     * \param max_on Maximum compare value (<= period)
     */
    void setOnTimeClamp(uint32_t min_on, uint32_t max_on) { minOn = min_on; maxOn = max_on; }

    /**
     * \brief Compute duties from d-q voltages and electrical angle
     *
     * Inputs may be in volts (with vbus set) or normalized to Vdc (set vbus=1).
     *
     * \param vd      D-axis voltage
     * \param vq      Q-axis voltage
     * \param theta_e Electrical angle (rad)
     * \return DutyCycles in timer counts
     */
    [[nodiscard]] DutyCycles computeFromDQ(float vd, float vq, float theta_e) const {
        const float c = std::cos(theta_e);
        const float s = std::sin(theta_e);
        const float v_alpha = vd * c - vq * s;
        const float v_beta  = vd * s + vq * c;
        return computeFromAB(v_alpha, v_beta);
    }

    /**
     * \brief Compute duties directly from alpha-beta voltages
     *
     * Inputs may be in volts (with vbus set) or normalized to Vdc (set vbus=1).
     *
     * \param v_alpha Alpha component
     * \param v_beta  Beta component
     * \return DutyCycles in timer counts
     */
    [[nodiscard]] DutyCycles computeFromAB(float v_alpha, float v_beta) const {
        // Normalize by Vdc if provided in volts
        const float Va = v_alpha / vbus;
        const float Vb = v_beta  / vbus;

        // Determine sector from vector angle
        float theta = std::atan2(Vb, Va);            // [-pi, pi]
        if (theta < 0.0f) theta += 2.0f * PI;        // [0, 2pi)
        const int sector = static_cast<int>(theta / SECTOR_RAD) % 6; // 0..5
        const float sector_base = static_cast<float>(sector) * SECTOR_RAD;
        const float phi = theta - sector_base;       // [0, 60deg)

        // Modulation index magnitude
        const float Vref = std::sqrt(Va*Va + Vb*Vb);

        // Dwell times (centered SVM)
        float T1, T2, T0;
        computeDwellTimes(Vref, phi, T1, T2, T0);

        // Map to phase on-times Ta/Tb/Tc (centered: add T0/2 baseline)
        float Ta = 0.5f * T0;
        float Tb = 0.5f * T0;
        float Tc = 0.5f * T0;

        const auto& pm = kSectorPhaseMap[sector];
        float add[3] = {0.0f, 0.0f, 0.0f};
        add[pm.t1_plus_t2] += (T1 + T2);
        add[pm.t2_only]    += T2;

        Ta += add[0]; Tb += add[1]; Tc += add[2];

        // Convert to ticks
        DutyCycles ticks = {
            static_cast<uint32_t>(std::lround(Ta * static_cast<float>(periodT))),
            static_cast<uint32_t>(std::lround(Tb * static_cast<float>(periodT))),
            static_cast<uint32_t>(std::lround(Tc * static_cast<float>(periodT)))
        };

        // Clamp to safe range
        clampMinMax(ticks);
        return ticks;
    }

    /**
     * \brief Enumerates the six 60° sectors
     */
    enum class Sector : uint8_t {
        SECTOR_1 = 0,
        SECTOR_2,
        SECTOR_3,
        SECTOR_4,
        SECTOR_5,
        SECTOR_6,
    };

    /**
     * \brief Active (basic) inverter space vectors V1..V6
     */
    enum class BasicVector : uint8_t {
        V1=1, V2, V3, V4, V5, V6
    };

    /**
     * \brief Zero vectors (all-high or all-low)
     */
    enum class ZeroVector : uint8_t {
        V0 = 0, V7 = 7
    };

    /**
     * \brief Adjacent basic vectors per sector (if building explicit sequences)
     */
    struct AdjacentBasic {
        BasicVector v_lo;
        BasicVector v_hi;
    };

    /**
     * \brief Per-sector adjacent basic vectors (S1..S6)
     */
    static constexpr std::array<AdjacentBasic, 6> kSectorBasics = {{
        { BasicVector::V1, BasicVector::V2 }, // S1
        { BasicVector::V2, BasicVector::V3 }, // S2
        { BasicVector::V3, BasicVector::V4 }, // S3
        { BasicVector::V4, BasicVector::V5 }, // S4
        { BasicVector::V5, BasicVector::V6 }, // S5
        { BasicVector::V6, BasicVector::V1 }, // S6
    }};

    /**
     * \brief Phase mapping for centered SVM additions
     * Phase indices: 0=A, 1=B, 2=C
     */
    struct PhaseMap {
        uint8_t t1_plus_t2;
        uint8_t t2_only;
        uint8_t t0_only;
    };

    /**
     * \brief Per-sector phase map (S1..S6)
     */
    static constexpr std::array<PhaseMap, 6> kSectorPhaseMap = {{
        {0,1,2}, // S1
        {1,0,2}, // S2
        {1,2,0}, // S3
        {2,1,0}, // S4
        {2,0,1}, // S5
        {0,2,1}, // S6
    }};

    /**
     * \brief Optional preferred zero vector ordering for symmetric SVM
     */
    static constexpr std::array<std::array<ZeroVector,3>,6> kZeroSeq = {{
        { ZeroVector::V0, ZeroVector::V7, ZeroVector::V0 }, // S1
        { ZeroVector::V0, ZeroVector::V7, ZeroVector::V0 }, // S2
        { ZeroVector::V0, ZeroVector::V7, ZeroVector::V0 }, // S3
        { ZeroVector::V0, ZeroVector::V7, ZeroVector::V0 }, // S4
        { ZeroVector::V0, ZeroVector::V7, ZeroVector::V0 }, // S5
        { ZeroVector::V0, ZeroVector::V7, ZeroVector::V0 }, // S6
    }};

private:
    /**
     * \brief Clamp duties to [minOn, maxOn]
     */
    void clampMinMax(DutyCycles& d) const {
        const uint32_t lo = (minOn <= maxOn ? minOn : 0U);
        const uint32_t hi = (maxOn <= periodT ? maxOn : periodT);
        d.dutyA = (d.dutyA < lo) ? lo : (d.dutyA > hi ? hi : d.dutyA);
        d.dutyB = (d.dutyB < lo) ? lo : (d.dutyB > hi ? hi : d.dutyB);
        d.dutyC = (d.dutyC < lo) ? lo : (d.dutyC > hi ? hi : d.dutyC);
    }

    /**
     * \brief Compute dwell times T1, T2, T0 from magnitude and intra-sector angle
     * \param Vref normalized magnitude (0..~0.907 in linear SVM)
     * \param phi  angle within sector [0, pi/3)
     * \param T1   out: dwell time for lower-angle basic vector
     * \param T2   out: dwell time for higher-angle basic vector
     * \param T0   out: total zero-vector time
     */
    void computeDwellTimes(float Vref, float phi, float& T1, float& T2, float& T0) const {
        // Times are normalized to Ts = 1.0 (we scale to ticks later)
        // T1 = m * sin(60° - phi), T2 = m * sin(phi), where m = (2/√3)*Vref for unity mapping.
        // Using classic SVM proportionality: directly use Vref with sine weights.
        const float s60 = SIN60;
        T1 = Vref * std::sin(PI_DIV_3 - phi);
        T2 = Vref * std::sin(phi);

        // Normalize to ensure T1+T2 <= (√3/2)*m in some texts; here we scale by (2/√3)
        const float scale = TWO_DIV_SQRT3; // 2/√3
        T1 *= scale;
        T2 *= scale;

        // Ensure non-negative and compute T0
        if (T1 < 0.0f) T1 = 0.0f;
        if (T2 < 0.0f) T2 = 0.0f;
        float sum = T1 + T2;
        if (sum > 1.0f) { // overmodulation guard
            const float k = 1.0f / sum;
            T1 *= k; T2 *= k; sum = 1.0f;
        }
        T0 = 1.0f - sum;
        if (T0 < 0.0f) T0 = 0.0f;
    }

    float   vbus   = 1.0f;
    uint32_t periodT = 100'000;
    uint32_t minOn = 0;
    uint32_t maxOn = 0xFFFFFFFFu;

    static constexpr float PI           = 3.14159265358979323846f;
    static constexpr float PI_DIV_3     = PI / 3.0f;
    static constexpr float SECTOR_RAD   = PI_DIV_3;              // 60°
    static constexpr float SIN60        = 0.86602540378443864676f;
    static constexpr float TWO_DIV_SQRT3= 1.15470053837925152902f; // 2/√3
};

} // namespace ZeroSequenceModulation
