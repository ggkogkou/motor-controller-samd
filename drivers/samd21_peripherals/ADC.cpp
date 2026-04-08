#include "ADC.hpp"

namespace SAMD21_Drivers {

ADC::ADC() {
        initializePeripheral();
}

uint16_t ADC::readResult() {
        return ADC_REGS->ADC_RESULT;
}

void ADC::enable() {
        ADC_REGS->ADC_CTRLA |= ADC_CTRLA_ENABLE_Msk;
        while ((ADC_REGS->ADC_STATUS & ADC_STATUS_SYNCBUSY_Msk) != 0U) {
        }
}

void ADC::disable() {
        ADC_REGS->ADC_CTRLA = ((ADC_REGS->ADC_CTRLA) & (uint8_t)(~ADC_CTRLA_ENABLE_Msk));
        while ((ADC_REGS->ADC_STATUS & ADC_STATUS_SYNCBUSY_Msk) != 0U) {
        }
}

void ADC::startConversion() {
        ADC_REGS->ADC_SWTRIG |= ADC_SWTRIG_START_Msk;
        while ((ADC_REGS->ADC_STATUS & ADC_STATUS_SYNCBUSY_Msk) != 0U) {
        }
}

void ADC::selectChannel(Channel channel) {
        // TODO: Select the ADC input channel.
        const auto SelectedChannel = static_cast<uint32_t>(channel);
}

void ADC::enableInterrupts() {

}

void ADC::initializePeripheral() {
        /* Reset ADC */
        ADC_REGS->ADC_CTRLA = ADC_CTRLA_SWRST_Msk;

        while ((ADC_REGS->ADC_STATUS & ADC_STATUS_SYNCBUSY_Msk) != 0U) {
                /* Wait for Synchronization */
        }

        uint32_t adc_linearity0 = (((*(uint32_t*)OTP4_ADDR) & ADC_LINEARITY0_Msk) >> ADC_LINEARITY0_POS);
        uint32_t adc_linearity1 = (((*(uint32_t*)(OTP4_ADDR + 4U)) & ADC_LINEARITY1_Msk) >> ADC_LINEARITY1_POS);

        /* Write linearity calibration and bias calibration */
        ADC_REGS->ADC_CALIB = (uint16_t)((ADC_CALIB_LINEARITY_CAL(adc_linearity0 | (adc_linearity1 << 5U))) |
                                         ADC_CALIB_BIAS_CAL((((*(uint32_t*)(OTP4_ADDR + 4U)) & ADC_BIASCAL_Msk) >> ADC_BIASCAL_POS)));

        /* Sampling length: minimum for fastest conversions */
        ADC_REGS->ADC_SAMPCTRL = ADC_SAMPCTRL_SAMPLEN(0U);

        /* reference */
        ADC_REGS->ADC_REFCTRL = ADC_REFCTRL_REFSEL_INTVCC0;

        /*
         * Scan only AIN10 and AIN11:
         * - Start at PIN10
         * - INPUTSCAN = 1 means total channels = 1+1 = 2 => PIN10, PIN11
         */
        ADC_REGS->ADC_INPUTCTRL = (uint32_t)ADC_POSINPUT_PIN10 | (uint32_t)ADC_NEGINPUT_GND | ADC_INPUTCTRL_INPUTSCAN(1U) |
                ADC_INPUTCTRL_INPUTOFFSET(0U) | ADC_INPUTCTRL_GAIN_1X;

        /* No hardware averaging */
        ADC_REGS->ADC_AVGCTRL = ADC_AVGCTRL_SAMPLENUM(0U) | ADC_AVGCTRL_ADJRES(0U);

        while ((ADC_REGS->ADC_STATUS & ADC_STATUS_SYNCBUSY_Msk) != 0U) {
                /* Wait for Synchronization */
        }

        /* Prescaler, Resolution & Operation Mode */
        ADC_REGS->ADC_CTRLB = ADC_CTRLB_PRESCALER_DIV4 | ADC_CTRLB_RESSEL_12BIT;

        while ((ADC_REGS->ADC_STATUS & ADC_STATUS_SYNCBUSY_Msk) != 0U) {
                /* Wait for Synchronization */
        }

        /* Clear all interrupt flags */
        ADC_REGS->ADC_INTFLAG = ADC_INTFLAG_Msk;

        /* Enable interrupts */
        ADC_REGS->ADC_INTENSET = ADC_INTENSET_RESRDY_Msk | ADC_INTENSET_OVERRUN_Msk;

        /* Events configuration  */
        ADC_REGS->ADC_EVCTRL = ADC_EVCTRL_STARTEI_Msk;

        while ((ADC_REGS->ADC_STATUS & ADC_STATUS_SYNCBUSY_Msk) != 0U) {
                /* Wait for Synchronization */
        }
}

} // namespace SAMD21_Drivers
