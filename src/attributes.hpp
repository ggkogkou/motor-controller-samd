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
 * @file   attributes.hpp
 * @brief  Define a set of attributes to use for putting code in RAM and also inlining critical parts of the code
 * @author Georgios Gkogkou <ggkogkou125@gmail.com>
 */

#pragma once

#if defined(__GNUC__)
#define RAMFUNC __attribute__((section(".ramfunc")))
#define ALWAYS_INLINE __attribute__((always_inline)) inline
#else
#define RAMFUNC
#define ALWAYS_INLINE inline
#endif
