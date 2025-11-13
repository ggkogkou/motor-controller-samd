#include "spi_buffer.hpp"

namespace ATSAMD21_GGKOGKOU {

void SPI_Buffer::init() { SERCOM4_SPI_CallbackRegister(&onTransferCompletion, reinterpret_cast<uintptr_t>(nullptr)); }

SPI_Buffer::TransactionState SPI_Buffer::submit(const SPI_Request& job) {
        if (SERCOM4_SPI_IsBusy() || not finished)
                return TransactionState::FAILED;

        finished = false;
        cachedRequest = job;

        PORT_PinClear(job.chipSelectPin);

        const auto Success = SERCOM4_SPI_WriteRead(&cachedRequest.txBuffer[0], cachedRequest.txBuffer.size(),
                                                   &job.rxBuffer[0], job.rxBuffer.size());

        if (not Success) {
                PORT_PinSet(cachedRequest.chipSelectPin);
                return TransactionState::FAILED;
        }

        return TransactionState::PLACED;
}

void SPI_Buffer::onTransferCompletion(uintptr_t context) {
        (void)context;

        PORT_PinSet(cachedRequest.chipSelectPin);

        auto* callback = cachedRequest.callback;
        void* userContext = cachedRequest.context;

        if (callback) {
                callback(userContext);
        }

        finished = true;
}

} // namespace ATSAMD21_GGKOGKOU
