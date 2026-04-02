#pragma once

#include <cstdint>
#include "device.h"
#include "plib_adc_common.h"

#define ADC_LINEARITY0_POS (27U)
#define ADC_LINEARITY0_Msk ((0x1FUL << ADC_LINEARITY0_POS))

#define ADC_LINEARITY1_POS (0U)
#define ADC_LINEARITY1_Msk ((0x7U << ADC_LINEARITY1_POS))

#define ADC_BIASCAL_POS (3U)
#define ADC_BIASCAL_Msk ((0x7U << ADC_BIASCAL_POS))

namespace SAMD21_Drivers {
class ADC {
public:
        enum class Channel {
                AIN0 = 0x00,
                AIN1 = 0x01,
                AIN2 = 0x02,
                AIN3 = 0x03,
                AIN4 = 0x04,
                AIN5 = 0x05,
                AIN6 = 0x06,
                AIN7 = 0x07,
                AIN8 = 0x08,
                AIN9 = 0x09,
                AIN10 = 0x0A,
                AIN11 = 0x0B,
                AIN12 = 0x0C,
                AIN13 = 0x0D,
                AIN14 = 0x0E,
                AIN15 = 0x0F,
                AIN16 = 0x10,
                AIN17 = 0x11,
                AIN18 = 0x12,
                AIN19 = 0x13,

                TEMP = 0x18,
                BANDGAP = 0x19,
                SCALEDCOREVCC = 0x1A,
                SCALEDIOVCC = 0x1B,
                DAC = 0x1C,
        };

        enum class GainFactor {
                GAIN_1X   = 0x0,
                GAIN_2X   = 0x1,
                GAIN_4X   = 0x2,
                GAIN_8X   = 0x3,
                GAIN_16X  = 0x4,

                DIV2      = 0xF,  // 1/2x
            };

        ADC();

        uint16_t readResult();

        void enable();

        void disable();

        void startConversion();

        void selectChannel(Channel channel);

        void enableInterrupts();

        void disableInterrupts();

private:
        void initializePeripheral();
};
} // namespace SAMD21_Drivers
