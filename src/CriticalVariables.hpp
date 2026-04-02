// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Georgios Gkogkou <ggkogkou125@gmail.com>

/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

/**
 * @file   CriticalVariables.hpp
 * @brief  A struct definition of the most important FOC variables that are subject to TMR
 * @author Georgios Gkogkou <ggkogkou125@gmail.com>
 */

#pragma once

#include "cstdint"

namespace PermanentMagnetSynchronousMotor {

/**
 * @struct FOC_CriticalVariables
 * A struct that contains the most important variables of the FOC algorithm
 *
 * The idea behind this is to store the most important variables in three separate banks in memory. Then, when reading the variables
 * from RAM, a simple majority voting procedure happens to select the value that appears to be in >2 banks the same. By doing this the
 * SEU/SEFI should have less effect to drive the algorithm to instability.
 *
 * The variables are enforced by design to live in different memory locations. That is done by writing three new sections in the linker
 * script and aligning them in a way that they are always pre-allocated at 1024 bytes and their size is not affected/shrinked.
 *
 * Testing the result:
 * Run (e.g.in bash): arm-none-eabi-nm -n build/g18a/motor-controller-samd21.elf | grep tmrCriticalVariables
 * Output should be:
 *      20002000 B _ZN31PermanentMagnetSynchronousMotor21tmrCriticalVariables1E
 *      20002400 B _ZN31PermanentMagnetSynchronousMotor21tmrCriticalVariables2E
 *      20002800 B _ZN31PermanentMagnetSynchronousMotor21tmrCriticalVariables3E
 *
 * @note That test also showcases the name mangling of C++, and this is the reason that startup code must be extern "C"
 */
struct FOC_CriticalVariables {
        uint16_t zeroOffsetElectricalAngle;
        int32_t dirSign;
        int32_t lastDirSign;
        uint32_t pwmPeriod;
        uint32_t velocityLoopPeriod_us;

        int32_t targetVelocity_mrad_s;
        int32_t iq_ref_mA;
        int32_t id_ref_mA;
        int32_t targetPosition_mrad;
        int32_t targetRevolutions;
        int32_t targetUnwrapped_mrad;
        bool targetUnwrappedValid;
        int32_t positionDirection;
};

extern constinit FOC_CriticalVariables tmrCriticalVariables1;
extern constinit FOC_CriticalVariables tmrCriticalVariables2;
extern constinit FOC_CriticalVariables tmrCriticalVariables3;

} // namespace PermanentMagnetSynchronousMotor
