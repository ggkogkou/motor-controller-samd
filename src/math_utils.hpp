#pragma once

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

    /**
     * Useful constants
     */
    inline constexpr float PI = 3.14159265358979323846f;
    inline constexpr float HALF_PI = PI / 2.0f;
    inline constexpr float TWO_PI = 2.0f * PI;

    /**
     * Struct that binds the look-up table generation for sine/cosine
     *
     * @tparam N The number of samples for the quantization of the continuous sin(x)
     */
    template<std::size_t N = 4096>
    struct SineLookUpTable {
        /**
         * Default constructor
         */
        constexpr SineLookUpTable() = default;

        /**
         * Function that calculates sin(x) from the Maclaurin power series with 6 extra terms
         * Now includes terms up to x^13 (n = 6 beyond the linear term).
         *
         * @param x The angle in radians
         * @return The value of sin(x)
         */
        static constexpr float calculateSineFromMaclaurin(float x) {
            const float xSquare = x * x;

            constexpr float Factorial_3 = 6.0f;
            constexpr float Factorial_5 = 120.0f;
            constexpr float Factorial_7 = 5040.0f;
            constexpr float Factorial_9 = 362880.0f;
            constexpr float Factorial_11 = 39916800.0f;
            constexpr float Factorial_13 = 6227020800.0f;

            constexpr float Factorial_3_Inv = 1.0f / Factorial_3;
            constexpr float Factorial_5_Inv = 1.0f / Factorial_5;
            constexpr float Factorial_7_Inv = 1.0f / Factorial_7;
            constexpr float Factorial_9_Inv = 1.0f / Factorial_9;
            constexpr float Factorial_11_Inv = 1.0f / Factorial_11;
            constexpr float Factorial_13_Inv = 1.0f / Factorial_13;

            return x * (1.0f
                + xSquare * (-Factorial_3_Inv
                + xSquare * (  Factorial_5_Inv
                + xSquare * ( -Factorial_7_Inv
                + xSquare * (  Factorial_9_Inv
                + xSquare * ( -Factorial_11_Inv
                + xSquare * Factorial_13_Inv))))));
        }

        /**
         * Alias for the look-up table type
         */
        using LookUpTable = std::array<float, N>;

        /**
         * Function that generates a Look-Up Table at compile time
         *
         * @return The look-up table
         */
        static consteval LookUpTable generateLookUpTable() {
            LookUpTable sineLUT {0.0f};

            for (size_t i = 1; i < N-1; i++)
                sineLUT[i] = calculateSineFromMaclaurin(TWO_PI * static_cast<float>(i) / static_cast<float>(N));

            return sineLUT;
        }

        /**
         * The generated sine LUT
         */
        static constexpr LookUpTable sine = generateLookUpTable();

        /**
         * Operator [] overload to easily map angle to index
         *
         * TODO: Add some checking on whether theta is in [0, 2pi]
         *
         * @param theta
         * @return
         */
        constexpr float operator[](float theta) const {
            // const auto thetaNormalized = [&] {
            //     const float t = std::fmod(theta, TWO_PI);
            //     return t < 0.0f ? t + TWO_PI : t;
            // }();

            // const std::size_t Index = static_cast<std::size_t>(thetaNormalized * static_cast<float>(N) / TWO_PI) % N;
            const std::size_t Index = static_cast<std::size_t>(theta * static_cast<float>(N) / TWO_PI) % N;

            return sine[Index];
        }

    };

    /**
     * Sine look-up table that must reside in internal flash memory
     */
    inline constexpr SineLookUpTable sinLUT;

    /**
     * Simple static assertions to quickly showcase the correctness
     */
    static_assert(sinLUT[0.0f] == 0.0f, "The sin(pi/2) does not evaluate to 1");
    static_assert(sinLUT[HALF_PI] >= 0.9999999f, "The sin(0) does not evaluate to 0");
    static_assert(sinLUT[TWO_PI] == 0.0f, "The sin(2pi) does not evaluate to 0");

    /**
     * Error limit to ensure a certain level of accuracy
     */
    inline constexpr float ErrorLimit = 0.001f;

    /**
     * Function that calculates the absolute difference of two numbers at compile time
     *
     * @param a Number a
     * @param b Number a
     * @return The absolute |a-b|
     */
    constexpr float absoluteError(float a, float b) {
        const float diff = a - b;
        return diff < 0.0f ? -diff : diff;
    }

    static_assert(absoluteError(sinLUT[0.356f], 0.34852783777f) <= ErrorLimit, "Error not acceptable");
    static_assert(absoluteError(sinLUT[1.255f], 0.95054936231f) <= ErrorLimit, "Error not acceptable");
    static_assert(absoluteError(sinLUT[1.788f], 0.97650387439f) <= ErrorLimit, "Error not acceptable");
    static_assert(absoluteError(sinLUT[2.500f], 0.59847214410f) <= ErrorLimit, "Error not acceptable");
    static_assert(absoluteError(sinLUT[3.657f], -0.49288931877f) <= ErrorLimit, "Error not acceptable");
    static_assert(absoluteError(sinLUT[4.438f], -0.96259093846f) <= ErrorLimit, "Error not acceptable");
    static_assert(absoluteError(sinLUT[5.796f], -0.46814053122f) <= ErrorLimit, "Error not acceptable");
    static_assert(absoluteError(sinLUT[6.200f], -0.08308940281f) <= ErrorLimit, "Error not acceptable");

} // namespace ZeroSequenceModulation::Math
