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
 * @file   CriticalVariables.cpp
 * @brief  A struct definition of the most important FOC variables that are subject to TMR
 * @author Georgios Gkogkou <ggkogkou125@gmail.com>
 */

#include "CriticalVariables.hpp"

namespace PermanentMagnetSynchronousMotor {

constinit FOC_CriticalVariables tmrCriticalVariables1 __attribute__((section(".tmr_bank_1"), used)){};
constinit FOC_CriticalVariables tmrCriticalVariables2 __attribute__((section(".tmr_bank_2"), used)){};
constinit FOC_CriticalVariables tmrCriticalVariables3 __attribute__((section(".tmr_bank_3"), used)){};

constinit FOC_CriticalVariables tmrPID_CriticalVariables1 __attribute__((section(".tmr_bank_1"), used)){};
constinit FOC_CriticalVariables tmrPID_CriticalVariables2 __attribute__((section(".tmr_bank_2"), used)){};
constinit FOC_CriticalVariables tmrPID_CriticalVariables3 __attribute__((section(".tmr_bank_3"), used)){};


} // namespace PermanentMagnetSynchronousMotor
