/*******************************************************************************
Simple Logger Implementation

  File Name:
    logger.c

  Summary:
    Simple logger utility for USART communication

  Description:
    Simple logger for sending char arrays via USART
*******************************************************************************/

#include "logger.h"
#include <string.h>

static bool loggerReady = true;

static void Logger_WriteCallback(uintptr_t context)
{
    loggerReady = true;
}

void Logger_Initialize(void)
{
    SERCOM3_USART_WriteCallbackRegister(Logger_WriteCallback, 0);
    loggerReady = true;

    Logger_Send("Logger initialized\r\n");
}

bool Logger_IsReady(void)
{
    return loggerReady && !SERCOM3_USART_WriteIsBusy();
}

void Logger_Send(const char* message)
{
    if (!Logger_IsReady()) {
        return; // Skip if not ready
    }

    loggerReady = false;
    SERCOM3_USART_Write((void*)message, strlen(message));
}