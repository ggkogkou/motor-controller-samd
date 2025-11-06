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

#define LOGGER_BUFFER_SIZE 256

// ANSI color codes
#define COLOR_RED "\033[31m"
#define COLOR_RESET "\033[0m"

static bool loggerReady = true;
static char logBuffer[LOGGER_BUFFER_SIZE];

static void Logger_WriteCallback(uintptr_t context) { loggerReady = true; }

// Simple string copy
static void simple_strcpy(char* dest, const char* src) {
        while ((*dest++ = *src++))
                ;
}

// Simple string concatenation
static void simple_strcat(char* dest, const char* src) {
        while (*dest)
                dest++;
        while ((*dest++ = *src++))
                ;
}

void Logger_Initialize(void) {
        SERCOM3_USART_WriteCallbackRegister(Logger_WriteCallback, 0);
        loggerReady = true;

        Logger_Send("Logger initialized\r\n");
}

bool Logger_IsReady(void) { return loggerReady && !SERCOM3_USART_WriteIsBusy(); }

void Logger_Send(const char* message) {
        if (!Logger_IsReady()) {
                return; // Skip if not ready
        }

        loggerReady = false;
        SERCOM3_USART_Write((void*)message, strlen(message));
}

void Logger_Info(const char* message) {
        if (!Logger_IsReady()) {
                return;
        }

        // Clear buffer
        logBuffer[0] = '\0';

        // Format: [INFO] message
        simple_strcpy(logBuffer, "[INFO] ");
        simple_strcat(logBuffer, message);

        loggerReady = false;
        SERCOM3_USART_Write(logBuffer, strlen(logBuffer));
}

void Logger_Error(const char* message) {
        if (!Logger_IsReady()) {
                return;
        }

        // Clear buffer
        logBuffer[0] = '\0';

        // Format: RED[ERROR] message RESET
        simple_strcpy(logBuffer, COLOR_RED);
        simple_strcat(logBuffer, "[ERROR] ");
        simple_strcat(logBuffer, message);
        simple_strcat(logBuffer, COLOR_RESET);

        loggerReady = false;
        SERCOM3_USART_Write(logBuffer, strlen(logBuffer));
}
