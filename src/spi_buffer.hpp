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
         * Constructor that registers the callback function to the HAL provided ISR
         */
        SPI_Buffer();

        /**
         * Function that fires an SPI transaction
         *
         * @param job The SPI specifics defined by the caller
         * @return false if a transfer is already happening
         */
        static bool submit(const SPI_Request& job);

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
};

} // namespace ATSAMD21_GGKOGKOU
