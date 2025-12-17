#include "USART_TxStream.hpp"
#include <cstring>

size_t USART_TxStream::write(std::span<uint8_t> data) {
        size_t written = 0;

        enterCritical_();
        while (written < data.size()) {
                const size_t nextHead = (txHead + 1) % BufferSize;

                // Full when head would catch tail
                if (nextHead == txTail) break;

                buffer[txHead] = data[written++];
                txHead = nextHead;
        }
        exitCritical_();

        beginTransaction();
        return written;
}

void USART_TxStream::beginTransaction() {
        // Only one PLIB write at a time
        if (inFlight || SERCOM3_USART_WriteIsBusy())
                return;

        // Compute available bytes in ring
        size_t head, tail;
        enterCritical_();
        head = txHead;
        tail = txTail;
        exitCritical_();

        const size_t available = (head >= tail) ? (head - tail) : (BufferSize - (tail - head));
        if (available == 0) return;

        // Contiguous bytes we can pass without wrapping
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
        // Called in SERCOM ISR context (PLIB callback)
        if (inFlight) {
                txTail = (txTail + inFlightLen) % BufferSize;
                inFlight = false;
                inFlightLen = 0;
        }
        beginTransaction();
}
