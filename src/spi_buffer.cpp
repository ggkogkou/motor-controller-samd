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
        SERCOM5_SPI_CallbackRegister(&onTransferCompletion, reinterpret_cast<uintptr_t>(nullptr));
}
SPI_Buffer::TransactionState SPI_Buffer::submit(const SPI_Request& job) {
        if (SERCOM5_SPI_IsBusy() || not finished)
                return TransactionState::FAILED;

        finished = false;
        cachedRequest = const_cast<SPI_Request*>(&job);

        PORT_PinClear(job.chipSelectPin);

        if (!SERCOM5_SPI_WriteRead(const_cast<uint8_t*>(job.txBuffer.data()), job.txBuffer.size(),
                                   job.rxBuffer.data(), job.rxBuffer.size())) {
                PORT_PinSet(job.chipSelectPin);
                cachedRequest = nullptr;
                finished = true;
                return TransactionState::FAILED;
        }

        return TransactionState::PLACED;
}

void SPI_Buffer::onTransferCompletion(uintptr_t context) {
        (void)context;
        finalizeTransfer();
}

void SPI_Buffer::finalizeTransfer() {
        if (cachedRequest == nullptr)
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
