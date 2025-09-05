#include <stddef.h>                     // Defines NULL
#include <stdbool.h>                    // Defines true
#include <stdlib.h>                     // Defines EXIT_FAILURE
#include "definitions.h"                // SYS function prototypes
#include "logger.h"                     // Logger utility

/***************************************
 * Check PWM outputs on pins
 * Channel 0 PWMH - PA08
 * Channel 0 PWML - PB10
 * Channel 1 PWMH - PA09
 * Channel 1 PWML - PB11
 * Channel 2 PWMH - PA10
 * Channel 2 PWML - PB12
***************************************/

/* Duty cycle increment value */
#define DUTY_INCREMENT (10U)

/* Save PWM period */
static uint32_t period;

char myData[] = {"\r\nSoftware is running\n\r"};
char messageError[] = "**** USART error occurred ****\r\n";

static bool errorStatus = false;
static bool writeStatus = true;
static bool readStatus = false;

void APP_WriteCallback(uintptr_t context)
{
    writeStatus = false;
}

/* This function is called after TCC period event */
void TCC_PeriodEventHandler(uint32_t status, uintptr_t context)
{
    /* duty cycle values */
    static uint32_t duty0 = 0U;
    static uint32_t duty1 = 800U;
    static uint32_t duty2 = 1600U;

    TCC0_PWM24bitDutySet(TCC0_CHANNEL0, duty0);
    TCC0_PWM24bitDutySet(TCC0_CHANNEL1, duty1);
    TCC0_PWM24bitDutySet(TCC0_CHANNEL2, duty2);

    /* Increment duty cycle values */
    duty0 += DUTY_INCREMENT;
    duty1 += DUTY_INCREMENT;
    duty2 += DUTY_INCREMENT;

    if (duty0 > period)
        duty0 = 0U;
    if (duty1 > period)
        duty1 = 0U;
    if (duty2 > period)
        duty2 = 0U;
}

int main ( void )
{
    /* Initialize all modules */
    SYS_Initialize ( NULL );

    /* Register callback functions and send start message */
    SERCOM3_USART_WriteCallbackRegister(APP_WriteCallback, 0);
    SERCOM3_USART_Write(&myData[0], sizeof(myData));

    /* Initialize logger - this will override the callback */
    Logger_Initialize();

    /* Register callback function for period event */
    TCC0_PWMCallbackRegister(TCC_PeriodEventHandler, (uintptr_t)NULL);

    /* Read the period */
    period = TCC0_PWM24bitPeriodGet();
    Logger_Info("PWM period configured\r\n");

    /* Start PWM*/
    TCC0_PWMStart();
    Logger_Info("PWM started\r\n");

    uint32_t loopCounter = 0;

    while ( true )     {
        /* Maintain state machines of all polled MPLAB Harmony modules. */
        // SYS_Tasks ( );

        if(errorStatus == true)
        {
            /* Send error message to console */
            errorStatus = false;
            SERCOM3_USART_Write(&messageError[0], sizeof(messageError));
            Logger_Error("Error occurred\r\n");
        }

        // else if(writeStatus == true)
        // {
        //     /* Submit buffer to read user data */
        //     SERCOM3_USART_Write(&myData[0], sizeof(myData));
        // }

        /* Simple periodic message */
        if (++loopCounter >= 10000000) {
            Logger_Info("Heartbeat\r\n");
            loopCounter = 0;
        }
    }

    /* Execution should not come here during normal operation */
    Logger_Send("Main loop exited!\r\n");

    return ( EXIT_FAILURE );
}
