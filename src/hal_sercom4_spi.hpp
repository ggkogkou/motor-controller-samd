#pragma once

#include <array>
#include <cstdint>
#include "definitions.h"

namespace ATSAMD21_GGKOGKOU {

struct SPI_Job {
        std::array<uint8_t, 2> txBuffer;
        mutable std::array<uint8_t, 2> rxBuffer;
        PORT_PIN chipSelectPin = PORT_PIN_NONE;

        void (*callback)(void* context) = nullptr;
        void* context = nullptr;
};

class SPI {
public:
        SPI() {
                SERCOM4_SPI_CallbackRegister(&onTransferCompletion, reinterpret_cast<uintptr_t>(this));
        }

        /**
         * Non-blocking submit
         *
         * @param job The SPI job
         * @return false if a transfer is already in flight
         */
        static bool submit(const SPI_Job &job) {
                if (SERCOM4_SPI_IsBusy())
                        return false;

                cachedJob = job;

                PORT_PinClear(cachedJob.chipSelectPin);

                const auto Success = SERCOM4_SPI_WriteRead(&cachedJob.txBuffer[0], cachedJob.txBuffer.size(),
                                                            &cachedJob.rxBuffer[0], cachedJob.rxBuffer.size());

                if (not Success) {
                        PORT_PinSet(cachedJob.chipSelectPin);
                        return false;
                }

                return true;
        }

private:
        /**
         * Callback on SPI transaction completion
         *
         * @brief Calls the user's provided callback function
         * @param context
         */
        static void onTransferCompletion(uintptr_t context) {
                (void) context;

                PORT_PinSet(cachedJob.chipSelectPin);



                auto* callback = cachedJob.callback;
                void* userContext = cachedJob.context;

                callback(userContext);
        }

        static SPI_Job cachedJob;
};

} // namespace ATSAMD21_GGKOGKOU
