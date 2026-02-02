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
 * @file   spi_buffer.cpp
 * @brief  Hardware-dependent implementaton of an SPI transaction abstraction layer buffer using Harmony 3 PLIBs
 * @author Georgios Gkogkou <ggkogkou125@gmail.com>
 */

#include "spi_buffer.hpp"

namespace ATSAMD21_GGKOGKOU {

void SPI_Buffer::init() {
        DMAC_ChannelCallbackRegister(DMA_RxChannel, &onDMA_RxCompletion, reinterpret_cast<uintptr_t>(nullptr));
        DMAC_ChannelCallbackRegister(DMA_TxChannel, &onDMA_TxCompletion, reinterpret_cast<uintptr_t>(nullptr));
}
SPI_Buffer::TransactionState SPI_Buffer::submit(const SPI_Request& job) {
        if (SERCOM5_SPI_IsBusy() || DMAC_ChannelIsBusy(DMA_RxChannel) || DMAC_ChannelIsBusy(DMA_TxChannel) || not finished)
                return TransactionState::FAILED;

        finished = false;
        rxComplete = false;
        txComplete = false;
        cachedRequest = const_cast<SPI_Request*>(&job);

        PORT_PinClear(job.chipSelectPin);

        const auto SPI_DataRegister = const_cast<const void*>(static_cast<const volatile void*>(&SERCOM5_REGS->SPIM.SERCOM_DATA));

        const auto RX_Buffer = static_cast<const void*>(job.rxBuffer.data());
        const auto TX_Buffer = static_cast<const void*>(job.txBuffer.data());

        const auto RX_Started = DMAC_ChannelTransfer(DMA_RxChannel, SPI_DataRegister, RX_Buffer, job.rxBuffer.size());
        const auto TX_Started = DMAC_ChannelTransfer(DMA_TxChannel, TX_Buffer, SPI_DataRegister, job.txBuffer.size());

        if (not RX_Started || not TX_Started) {
                PORT_PinSet(job.chipSelectPin);
                DMAC_ChannelDisable(DMA_RxChannel);
                DMAC_ChannelDisable(DMA_TxChannel);
                cachedRequest = nullptr;

                return TransactionState::FAILED;
        }

        return TransactionState::PLACED;
}

void SPI_Buffer::onDMA_RxCompletion(DMAC_TRANSFER_EVENT event, uintptr_t context) {
        (void)context;

        if (event == DMAC_TRANSFER_EVENT_ERROR) {
                DMAC_ChannelDisable(DMA_TxChannel);
                DMAC_ChannelDisable(DMA_RxChannel);
                finished = true;
                cachedRequest = nullptr;
                return;
        }

        rxComplete = true;
        finalizeTransfer();
}

void SPI_Buffer::onDMA_TxCompletion(DMAC_TRANSFER_EVENT event, uintptr_t context) {
        (void)context;

        if (event == DMAC_TRANSFER_EVENT_ERROR) {
                DMAC_ChannelDisable(DMA_TxChannel);
                DMAC_ChannelDisable(DMA_RxChannel);
                finished = true;
                cachedRequest = nullptr;
                return;
        }

        txComplete = true;
        finalizeTransfer();
}

void SPI_Buffer::finalizeTransfer() {
        if (not rxComplete || !txComplete || cachedRequest == nullptr)
                return;

        while (SERCOM5_SPI_IsTransmitterBusy()) {
                // wait for the SPI shifter to finish before releasing CS
        }

        PORT_PinSet(cachedRequest->chipSelectPin);

        auto* callback = cachedRequest->callback;
        void* userContext = cachedRequest->context;

        cachedRequest = nullptr;
        finished = true;

        if (callback) {
                callback(userContext);
        }
}

} // namespace ATSAMD21_GGKOGKOU
