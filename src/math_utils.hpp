#pragma once

#include <cstdint>
#include <span>
#include <array>
#include <cmath>

namespace ZeroSequenceModulation::Math {
    /**
     * Constant representing the square root of 3 (√3) as a floating-point value
     */
    inline constexpr float SQRT3 = 1.7320508075688772f;

    /**
     * Function that implements the Inverse Park Transform
     *
     * Inverse Park Transform converts the rotating dq-frame into the fixed ab-frame
     *
     * @param dqFrame An array that contains [Ud, Uq, theta_e]
     * @return An array that contains the [Ualpha, Ubeta]
     */
    inline std::array<float, 2> inverseParkTransform(std::span<float> dqFrame) {
        // Expect at least 3 elements: Ud, Uq, theta
        if (dqFrame.size() < 3)
            return {0.0f, 0.0f};

        const float Ud = dqFrame[0];
        const float Uq = dqFrame[1];
        const float theta = dqFrame[2];

        const float c = std::cos(theta);
        const float s = std::sin(theta);

        const float Ualpha = Ud * c - Uq * s;
        const float Ubeta  = Ud * s + Uq * c;

        return { Ualpha, Ubeta };
    }

    /**
     * Function that implements the Inverse Clarke Transform
     *
     * Inverse Clarke Transform converts the fixed ab-frame into the three components Ua, Ub, Uc frame
     *
     * @param abFrame An array that contains the [Ualpha, Ubeta]
     * @return An array that contains the [Ua, Ub, Uc]
     */
    inline std::array<float, 3> inverseClarkeTransform(std::span<float> abFrame) {
        // Expect at least 2 elements: Ualpha, Ubeta
        if (abFrame.size() < 2)
            return {0.0f, 0.0f, 0.0f};

        const float Ualpha = abFrame[0];
        const float Ubeta  = abFrame[1];

        const float Ua = Ualpha;
        const float Ub = (-Ualpha + SQRT3 * Ubeta) * 0.5f;
        const float Uc = -(Ua + Ub);

        return { Ua, Ub, Uc };
    }

    /**
     * Function that implements the Clarke Transform
     *
     * Clarke Transform converts three-phase quantities (a,b,c) into the fixed αβ frame.
     * Assumes a balanced system with ia + ib + ic = 0; only ia and ib are needed.
     *
     * @param abcFrame An array that contains [Ua, Ub, Uc] (Uc can be ignored if balanced)
     * @return An array that contains the [Ualpha, Ubeta]
     */
    inline std::array<float, 2> clarkeTransform(std::span<float> abcFrame) {
        if (abcFrame.size() < 2)
            return {0.0f, 0.0f};

        const float Ua = abcFrame[0];
        const float Ub = abcFrame[1];
        // Uc is not required for balanced system (Uc = -(Ua+Ub))
        const float Ualpha = Ua;
        const float Ubeta  = (Ua + 2.0f * Ub) / SQRT3;

        return { Ualpha, Ubeta };
    }

    /**
     * Function that implements the Park Transform
     *
     * Park Transform rotates αβ components into the synchronous dq frame by angle theta.
     *
     * @param abThetaFrame An array that contains [Ualpha, Ubeta, theta_e]
     * @return An array that contains the [Ud, Uq]
     */
    inline std::array<float, 2> parkTransform(std::span<float> abThetaFrame) {
        if (abThetaFrame.size() < 3)
            return {0.0f, 0.0f};

        const float Ualpha = abThetaFrame[0];
        const float Ubeta  = abThetaFrame[1];
        const float theta  = abThetaFrame[2];

        const float c = std::cos(theta);
        const float s = std::sin(theta);

        // Ud =  Ualpha*cos + Ubeta*sin
        // Uq = -Ualpha*sin + Ubeta*cos
        const float Ud =  Ualpha * c + Ubeta * s;
        const float Uq = -Ualpha * s + Ubeta * c;

        return { Ud, Uq };
    }

} // namespace ZeroSequenceModulation::Math