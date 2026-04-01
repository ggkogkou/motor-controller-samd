#pragma once

#include "TMR.hpp"

namespace PermanentMagnetSynchronousMotor {

struct FOC_CriticalVariables {
        uint16_t zeroOffsetElectricalAngle;
        int32_t dirSign;
        int32_t lastDirSign;
        uint32_t pwmPeriod;
        uint32_t velocityLoopPeriod_us;
};

__attribute__((section(".tmr_bank_1"))) inline FOC_CriticalVariables tmrCriticalVariables1{};
__attribute__((section(".tmr_bank_2"))) inline FOC_CriticalVariables tmrCriticalVariables2{};
__attribute__((section(".tmr_bank_3"))) inline FOC_CriticalVariables tmrCriticalVariables3{};

} // namespace PermanentMagnetSynchronousMotor
