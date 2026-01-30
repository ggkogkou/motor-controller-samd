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

void USART_TxStream::init() {
        DMAC_ChannelCallbackRegister(TxDmaChannel, &USART_TxStream::dmacDoneThunk, reinterpret_cast<uintptr_t>(this));
}

size_t USART_TxStream::write(std::span<const uint8_t> data) {
        const size_t written = ring.write(data);

        if (written) {
                NVIC_DisableIRQ(DMAC_IRQn);
                beginTransaction();
                NVIC_EnableIRQ(DMAC_IRQn);
        }

        return written;
}

void USART_TxStream::beginTransaction() {
        if (inFlight) {
                return;
        }

        const auto span = ring.peekContiguous();
        if (span.empty()) {
                return;
        }

        size_t chunk = span.size();
        if (chunk > MaxChunkSize) {
                chunk = MaxChunkSize;
        }

        inFlight = true;
        inFlightLen = chunk;

        const void* src = static_cast<const void*>(span.data());
        const void* dst = const_cast<const void*>(
            static_cast<const volatile void*>(&SERCOM3_REGS->USART_INT.SERCOM_DATA));

        if (!DMAC_ChannelTransfer(TxDmaChannel, src, dst, chunk)) {
                inFlight = false;
                inFlightLen = 0;
        }
}

void USART_TxStream::dmacDoneThunk(DMAC_TRANSFER_EVENT event, uintptr_t ctx) {
        if (auto* self = reinterpret_cast<USART_TxStream*>(ctx))
                self->onDmaCompletion(event);
}

void USART_TxStream::onDmaCompletion(DMAC_TRANSFER_EVENT event) {
        if (event == DMAC_TRANSFER_EVENT_COMPLETE && inFlight) {
                ring.consume(inFlightLen);
        }

        inFlight = false;
        inFlightLen = 0;

        beginTransaction();
}
