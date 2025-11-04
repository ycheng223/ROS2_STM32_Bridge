#ifndef UART_H
#define UART_H

#include "stm32f4xx.h"
#include <stdint.h>

// Initialize USARTs
void UART_Init(void);  // Initialize all three UARTs

// Send a single byte over the specified UART
void UART_SendByte(USART_TypeDef *usart, uint8_t byte);

// Read a character from RX buffer of the specified UART (uart_id)
uint8_t UART_ReadByte(uint8_t uart_id);

// Send a string from UART2 to command computer
void UART_SendString(uint8_t uart_id, const char *str);

// Check if data is available in UART2 RX buffer
uint8_t UART_DataAvailable(uint8_t uart_id);


#endif /* UART_H */
