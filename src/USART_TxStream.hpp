#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "definitions.h" // brings in SERCOM3_USART_* prototypes

class USART_TxStream {
public:
        static constexpr size_t BufferSize = 1024;
        static constexpr size_t MaxChunkSize = 64;

        void init() {
                SERCOM3_USART_WriteCallbackRegister(&USART_TxStream::txDoneThunk, reinterpret_cast<uintptr_t>(this));
        }

        // Accepts bytes to enqueue; returns number of bytes actually enqueued.
        size_t write(std::span<uint8_t> data);

        // Try to start a PLIB transaction if idle.
        void beginTransaction();

private:
        // Ring buffer
        volatile size_t txHead = 0;
        volatile size_t txTail = 0;
        std::array<uint8_t, BufferSize> buffer{};

        // Track the currently in-flight PLIB write
        volatile bool inFlight = false;
        volatile size_t inFlightLen = 0;

        // C callback trampoline (must be static)
        static void txDoneThunk(uintptr_t ctx) {
                auto* self = reinterpret_cast<USART_TxStream*>(ctx);
                self->onTxCompletion();
        }

        void onTxCompletion();

        // IRQ save/restore (better than always enable_irq())
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
