#pragma once

#include <cstdint>
#include "device.h"

class GenericClockController {
public:
        GenericClockController() = default;

        static void initializePeripheral();

private:
        static void initialize_SYSCTRL();
        static void initialize_DFLL();
        static void initialize_GCLK0();
        static void initialize_GCLK1();
        static void initialize_GCLK2();
};
