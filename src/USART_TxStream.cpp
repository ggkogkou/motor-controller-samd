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
 * @file   USART_TxStream.cpp
 * @brief  A SAMD21 USART transaction abstraction using Harmony 3 PLIBs and implementing a ring-buffer
 * @author Georgios Gkogkou <ggkogkou125@gmail.com>
 */

#include "USART_TxStream.hpp"

size_t USART_TxStream::write(std::span<uint8_t> data) {
        size_t written = 0;

        enterCritical_();

        while (written < data.size()) {
                const size_t nextHead = (txHead + 1) % BufferSize;

                if (nextHead == txTail)
                        break;

                buffer[txHead] = data[written++];
                txHead = nextHead;
        }

        exitCritical_();

        beginTransaction();

        return written;
}

void USART_TxStream::beginTransaction() {
        if (inFlight || SERCOM3_USART_WriteIsBusy())
                return;

        size_t head, tail;
        enterCritical_();
        head = txHead;
        tail = txTail;
        exitCritical_();

        const size_t available = (head >= tail) ? (head - tail) : (BufferSize - (tail - head));

        if (available == 0)
                return;

        size_t chunk = (head >= tail) ? (head - tail) : (BufferSize - tail);

        if (chunk > MaxChunkSize)
                chunk = MaxChunkSize;

        inFlight = true;
        inFlightLen = chunk;

        if (not SERCOM3_USART_Write(&buffer[tail], chunk)) {
                inFlight = false;
                inFlightLen = 0;
        }
}

void USART_TxStream::onTxCompletion() {
        if (inFlight) {
                txTail = (txTail + inFlightLen) % BufferSize;
                inFlight = false;
                inFlightLen = 0;
        }

        beginTransaction();
}
