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
 * @file   main.cpp
 * @brief  Location of the main() function and the superloop
 * @author Georgios Gkogkou <ggkogkou125@gmail.com>
 */

#include <cstdlib>
#include "definitions.h"
#include "USART_TxStream.hpp"
#include "spi_buffer.hpp"
#include "SAMD21_FOC.hpp"

using namespace PermanentMagnetSynchronousMotor;

[[noreturn]] int main() {
        SYS_Initialize(nullptr);
        SYSTICK_TimerStart();

        SPI_Buffer::init();

        USART_TxStream logging;
        logging.init();

        SAMD21_FOC foc;

        while (true) {
                static uint8_t c = 'A';
                logging.write(std::span(&c, 1));
                if (++c > 'Z')
                        c = 'A';

                uint8_t eol[2] = {'\r', '\n'};
                logging.write(std::span(eol, 2));

                SYSTICK_DelayMs(200);
        }
}
