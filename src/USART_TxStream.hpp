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
 * @file   USART_TxStream.hpp
 * @brief  A SAMD21 USART transaction abstraction using Harmony 3 PLIBs and implementing a ring-buffer
 * @author Georgios Gkogkou <ggkogkou125@gmail.com>
 */

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include "definitions.h"

class USART_TxStream {
public:
        static constexpr size_t BufferSize = 1024;
        static constexpr size_t MaxChunkSize = 64;

        void init() {
                SERCOM3_USART_WriteCallbackRegister(&USART_TxStream::txDoneThunk, reinterpret_cast<uintptr_t>(this));
        }

        size_t write(std::span<uint8_t> data);

        void beginTransaction();

private:
        volatile size_t txHead = 0;
        volatile size_t txTail = 0;
        std::array<uint8_t, BufferSize> buffer{};

        volatile bool inFlight = false;
        volatile size_t inFlightLen = 0;

        static void txDoneThunk(uintptr_t ctx) {
                auto* self = reinterpret_cast<USART_TxStream*>(ctx);
                self->onTxCompletion();
        }

        void onTxCompletion();

        uint32_t primask_ = 0;
        void enterCritical_() {
                primask_ = __get_PRIMASK();
                __disable_irq();
        }
        void exitCritical_() {
                if (primask_ == 0)
                        __enable_irq();
        }
};
