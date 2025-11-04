#ifndef CONTROL_H
#define CONTROL_H

#include "stm32f4xx.h"
#include <stdint.h>

// ROS2 binary protocol state machine (5-byte protocol)
// Protocol: [Motor ID: 1 byte] + [duty_scaled: 4-byte int32, little-endian]
typedef enum {
    ROS2_WAITING_MOTOR,      // Expecting 'L' or 'R'
    ROS2_WAITING_INT32_BYTE0, // LSB of scaled duty (little-endian)
    ROS2_WAITING_INT32_BYTE1,
    ROS2_WAITING_INT32_BYTE2,
    ROS2_WAITING_INT32_BYTE3  // MSB of scaled duty
} ros2_protocol_state_t;

// Initialize the controller
void Control_Init(void);

// Process a single incoming byte from UART2 (ROS2)
void Control_ProcessByte(uint8_t byte);

// Process single character commands ('X', ' ', etc.)
void Control_ProcessCommand(uint8_t command);

// Run periodic safety checks
void Control_SafetyCheck(void);

// Stop both motors
void Control_Stop(void);

// Brake both motors
void Control_Brake(void);


#endif /* CONTROL_H */