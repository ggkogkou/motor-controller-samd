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
#include "RingBuffer.hpp"
#include "definitions.h"

class USART_TxStream {
public:
        static constexpr size_t BufferSize = 1024;
        static constexpr size_t MaxChunkSize = 256;
        static constexpr DMAC_CHANNEL TxDmaChannel = DMAC_CHANNEL_0;

        USART_TxStream() : ring(std::span<uint8_t>(storage.data(), storage.size())) {}

        void init();

        size_t write(std::span<const uint8_t> data);
        size_t write(std::span<uint8_t> data) {
                return write(std::span<const uint8_t>(data.data(), data.size()));
        }

        void poll();

private:
        std::array<uint8_t, BufferSize> storage{};

        RingBuffer ring;

        volatile bool inFlight = false;
        volatile size_t inFlightLen = 0;

        volatile bool kickPending = false;

        void beginTransaction();

        static void dmacDoneThunk(DMAC_TRANSFER_EVENT event, uintptr_t ctx);
        void onDmaCompletion(DMAC_TRANSFER_EVENT event);
};
