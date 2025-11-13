#pragma once

#include <array>
#include <cstdint>
#include "definitions.h"

namespace ATSAMD21_GGKOGKOU {

struct SPI_Request {
        std::array<uint8_t, 2> txBuffer;
        mutable std::array<uint8_t, 2> rxBuffer;
        PORT_PIN chipSelectPin = PORT_PIN_NONE;

        void (*callback)(void* context) = nullptr;
        void* context = nullptr;
};

/**
 * @class SPI_Buffer
 *
 * A buffer SPI handler that receives requests from the different SPI devices and initiates transactions
 */
class SPI_Buffer {
public:
        /**
         * Initialization function that registers the callback function to the HAL provided ISR
         *
         * @warning MUST BE CALLED BEFORE THE 'SPI_Buffer' IS USED
         */
        static void init();

        /**
         * @enum TransactionState
         *
         * Possible SPI transaction states; might be used for error handling
         */
        enum class TransactionState {
                PLACED,
                FAILED,
        };

        /**
         * Function that fires an SPI transaction
         *
         * @param job The SPI specifics defined by the caller
         * @return false if a transfer is already happening
         */
        static TransactionState submit(const SPI_Request& job);

        SPI_Buffer() = delete;
        SPI_Buffer(const SPI_Buffer&) = delete;
        SPI_Buffer& operator=(const SPI_Buffer&) = delete;
        SPI_Buffer(SPI_Buffer&&) = delete;
        SPI_Buffer& operator=(SPI_Buffer&&) = delete;

private:
        /**
         * Callback on SPI transaction completion
         *
         * @brief Calls the user's provided callback function
         * @param context
         */
        static void onTransferCompletion(uintptr_t context);

        /**
         * Keeping a copy of the SPI in order to be callable from the onTransferCompletion callback
         */
        static inline SPI_Request cachedRequest{};

        /**
         * If SPI transaction is done and caller's callback is over, mark buffer as ready
         */
        static inline bool finished = true;
};

} // namespace ATSAMD21_GGKOGKOU
