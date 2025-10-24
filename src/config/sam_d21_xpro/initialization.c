#include "definitions.h"
#include "device.h"


/*******************************************************************************
  Function:
    void SYS_Initialize (void *data)

  Summary:
    Initializes the board, services, drivers, application and other modules.

  Remarks:
 */

void SYS_Initialize ( void* data ) {
  NVMCTRL_REGS->NVMCTRL_CTRLB = NVMCTRL_CTRLB_RWS(3UL);

  PORT_Initialize();
  CLOCK_Initialize();
  NVIC_Initialize();
  SYSTICK_TimerInitialize();
  NVMCTRL_Initialize( );
  EIC_Initialize();
  SERCOM1_SPI_Initialize();
  SERCOM3_USART_Initialize();
  SERCOM4_SPI_Initialize();
  SERCOM5_USART_Initialize();
  ADC_Initialize();
  TC4_CaptureInitialize();
  TCC1_PWMInitialize();
  TCC2_PWMInitialize();
  TCC0_PWMInitialize();
}
