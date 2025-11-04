#include <ctype.h>
#include <stdio.h>
#include <stm32f4xx.h>
#include <uart.h>
#include <vesc.h>
#include <init.h>
#include <control.h>
#include <ultrasonic.h>

// Global millisecond tick counter
volatile uint32_t msTicks = 0;

// SysTick interrupt handler - increments every 1ms
void SysTick_Handler(void) {
    msTicks++;
}

// Function to get current system tick count (1ms resolution)
uint32_t GetTick(void) {
    return msTicks;
}

int main(void) {
    SystemClock_Config();
    UART_Init();
    Control_Init();
    Ultrasonic_Init();

    // Send startup messages
    UART_SendString(2, "==================================\r\n");
    UART_SendString(2, "STM32F446 VESC Control System\r\n");
    UART_SendString(2, "==================================\r\n");
    UART_SendString(2, "Left Motor:       USART1 (VESC)\r\n");
    UART_SendString(2, "Right Motor:      USART3 (VESC)\r\n");
    UART_SendString(2, "ROS2 Bridge:      USART2\r\n");
    UART_SendString(2, "VESC Protocol:    5-byte binary\r\n");
    UART_SendString(2, "==================================\r\n");

    uint32_t lastCommandTime = GetTick();  // Initialize with current time
    const uint32_t TIMEOUT_MS = 1000;     // 1 second timeout
    uint32_t lastDistancePrint = GetTick(); // For periodic distance print
    const uint32_t DISTANCE_PRINT_INTERVAL = 500; // 0.500 seconds

    while(1) {      // Process all ROS2 5-bit binary commands from UART2
        while (UART_DataAvailable(2)) {  // Check UART2
            uint8_t byte = UART_ReadByte(2);  // Read byte from UART2
            lastCommandTime = GetTick();
            Control_ProcessByte(byte); // Process byte through state machine in control.c
        }        
        if ((GetTick() - lastCommandTime) > TIMEOUT_MS) {       // Check for command timeout (1 second without input)
            Control_Stop();
            lastCommandTime = GetTick();  // Reset to prevent continuous stopping
        }
        
        Control_SafetyCheck();         // Perform ultrasonic safety check

        if ((GetTick() - lastDistancePrint) > DISTANCE_PRINT_INTERVAL) {         // Print distance periodically (every 0.25 seconds)

            uint32_t distance = Ultrasonic_MeasureDistance();
            char buffer[64];
            if (distance != 0xFFFFFFFF) {
                sprintf(buffer, "Ultrasonic distance: %lu cm\r\n", distance);
            } else {
                sprintf(buffer, "Ultrasonic distance: Out of range\r\n");
            }
            UART_SendString(2, buffer);
            lastDistancePrint = GetTick();
        }
        for (volatile int i = 0; i < 100; i++){};        // Small delay to prevent excessive polling


    }
}
