#include <cstdlib>
#include "definitions.h"
#include "USART_TxStream.hpp"
#include "logger.hpp"
#include "spi_buffer.hpp"
#include "SAMD21_FOC.hpp"

using namespace PermanentMagnetSynchronousMotor;

[[noreturn]] int main() {
        SYS_Initialize(nullptr);
        SYSTICK_TimerStart();

        SPI_Buffer::init();

        USART_TxStream logging;
        logging.init();

        SAMD21_FOC foc;

        while (true) {
                static uint8_t c = 'A';
                logging.write(std::span(&c, 1));
                if (++c > 'Z')
                        c = 'A';

                uint8_t eol[2] = {'\r', '\n'};
                logging.write(std::span(eol, 2));

                SYSTICK_DelayMs(200);
        }
}
