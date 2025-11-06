
/*******************************************************************************
  Simple Logger Header File

  File Name:
    logger.h

  Summary:
    Simple logger utility for USART communication

  Description:
    Simple logger for sending char arrays via USART
*******************************************************************************/

#ifndef LOGGER_H
#define LOGGER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "definitions.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Initialize the logger */
void Logger_Initialize(void);

/* Check if logger is ready for new message */
bool Logger_IsReady(void);

/* Send a simple message */
void Logger_Send(const char* message);

/* Send INFO message (no color) */
void Logger_Info(const char* message);

/* Send ERROR message (red color) */
void Logger_Error(const char* message);

#ifdef __cplusplus
}
#endif

#endif /* LOGGER_H */
