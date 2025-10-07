#pragma once


#include <cstdint>
#include <array>

class SVPWM {
public:

    SVPWM(uint32_t peeiod, float vbus = 1.0f) {}
    SVPWM() = default;
    ~SVPWM() = default;
    SVPWM(const SVPWM&) = delete;
    SVPWM& operator=(const SVPWM&) = delete;

    float vbus = 1.0f;



    struct DutyCycles {
        uint32_t dutyA;
        uint32_t dutyB;
        uint32_t dutyC;
    };

    DutyCycles calculateFromDQ();

    DutyCycles calculateFromAB();

    void clampMinMax();

    enum class Sector {
        SECTOR_1,
        SECTOR_2,
        SECTOR_3,
        SECTOR_4,
        SECTOR_5,
        SECTOR_6,
    };

    enum class BasicVectors {

    };

    enum class ZeroVectors {

    };


};
