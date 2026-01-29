#include "plib_adc.h"
#include "interrupts.h"

static volatile ADC_CALLBACK_OBJ ADC_CallbackObject;

#define ADC_LINEARITY0_POS (27U)
#define ADC_LINEARITY0_Msk ((0x1FUL << ADC_LINEARITY0_POS))

#define ADC_LINEARITY1_POS (0U)
#define ADC_LINEARITY1_Msk ((0x7U << ADC_LINEARITY1_POS))

#define ADC_BIASCAL_POS (3U)
#define ADC_BIASCAL_Msk ((0x7U << ADC_BIASCAL_POS))

void ADC_Initialize(void) {
        /* Reset ADC */
        ADC_REGS->ADC_CTRLA = ADC_CTRLA_SWRST_Msk;
        while ((ADC_REGS->ADC_STATUS & ADC_STATUS_SYNCBUSY_Msk) != 0U) {
        }

        uint32_t adc_linearity0 = (((*(uint32_t*)OTP4_ADDR) & ADC_LINEARITY0_Msk) >> ADC_LINEARITY0_POS);
        uint32_t adc_linearity1 = (((*(uint32_t*)(OTP4_ADDR + 4U)) & ADC_LINEARITY1_Msk) >> ADC_LINEARITY1_POS);

        /* Write linearity calibration and bias calibration */
        ADC_REGS->ADC_CALIB = (uint16_t)((ADC_CALIB_LINEARITY_CAL(adc_linearity0 | (adc_linearity1 << 5U))) |
                                         ADC_CALIB_BIAS_CAL(((*(uint32_t*)(OTP4_ADDR + 4U)) & ADC_BIASCAL_Msk) >> ADC_BIASCAL_POS));

        /* Sampling length */
        // ADC_REGS->ADC_SAMPCTRL = ADC_SAMPCTRL_SAMPLEN(3U);
        ADC_REGS->ADC_SAMPCTRL = ADC_SAMPCTRL_SAMPLEN(8U);

        /* Reference: INTVCC0 (VDD/1.48 ~= 2.23V @ 3.3V VDD) */
        ADC_REGS->ADC_REFCTRL = ADC_REFCTRL_REFSEL_INTVCC0;

        /*
         * Scan only AIN10 and AIN11:
         * - Start at PIN10
         * - INPUTSCAN = 1 means total channels = 1+1 = 2 => PIN10, PIN11
         */
        ADC_REGS->ADC_INPUTCTRL = (uint32_t)ADC_POSINPUT_PIN10 | (uint32_t)ADC_NEGINPUT_GND | ADC_INPUTCTRL_INPUTSCAN(1U) |
                ADC_INPUTCTRL_INPUTOFFSET(0U) | ADC_INPUTCTRL_GAIN_1X;

        ADC_REGS->ADC_AVGCTRL  = ADC_AVGCTRL_SAMPLENUM(2U) | ADC_AVGCTRL_ADJRES(0U);

        while ((ADC_REGS->ADC_STATUS & ADC_STATUS_SYNCBUSY_Msk) != 0U) {
        }

        /* Prescaler, Resolution & Operation Mode */
        ADC_REGS->ADC_CTRLB = ADC_CTRLB_PRESCALER_DIV32 | ADC_CTRLB_RESSEL_16BIT;

        while ((ADC_REGS->ADC_STATUS & ADC_STATUS_SYNCBUSY_Msk) != 0U) {
        }

        /* Clear all interrupt flags */
        ADC_REGS->ADC_INTFLAG = ADC_INTFLAG_Msk;

        /* Enable interrupts */
        ADC_REGS->ADC_INTENSET = ADC_INTENSET_RESRDY_Msk | ADC_INTENSET_OVERRUN_Msk;

        /* Events configuration: start conversion on event */
        ADC_REGS->ADC_EVCTRL = ADC_EVCTRL_STARTEI_Msk;
        while ((ADC_REGS->ADC_STATUS & ADC_STATUS_SYNCBUSY_Msk) != 0U) {
        }
}

void ADC_Enable(void) {
        ADC_REGS->ADC_CTRLA |= ADC_CTRLA_ENABLE_Msk;
        while ((ADC_REGS->ADC_STATUS & ADC_STATUS_SYNCBUSY_Msk) != 0U) {
        }
}

void ADC_Disable(void) {
        ADC_REGS->ADC_CTRLA = ((ADC_REGS->ADC_CTRLA) & (uint8_t)(~ADC_CTRLA_ENABLE_Msk));
        while ((ADC_REGS->ADC_STATUS & ADC_STATUS_SYNCBUSY_Msk) != 0U) {
        }
}

uint16_t ADC_ConversionResultGet(void) {
        return (uint16_t)ADC_REGS->ADC_RESULT;
}

void ADC_InterruptsClear(ADC_STATUS interruptMask) {
        ADC_REGS->ADC_INTFLAG = interruptMask;
}

void ADC_CallbackRegister(ADC_CALLBACK callback, uintptr_t context) {
        ADC_CallbackObject.callback = callback;
        ADC_CallbackObject.context = context;
}

void __attribute__((used)) ADC_InterruptHandler(void) {
        ADC_STATUS status = (ADC_STATUS)ADC_REGS->ADC_INTFLAG;

        /* Clear only the flags that are set */
        ADC_REGS->ADC_INTFLAG = status & (ADC_INTFLAG_RESRDY_Msk | ADC_INTFLAG_OVERRUN_Msk);

        if (ADC_CallbackObject.callback != NULL)
                ADC_CallbackObject.callback(status, ADC_CallbackObject.context);
}
