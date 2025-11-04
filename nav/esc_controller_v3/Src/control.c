#include <control.h>
#include <uart.h>
#include <vesc.h>
#include "ultrasonic.h"
#include <string.h>  // For memcpy


// Safety override flag
static uint8_t safetyOverride = 0;

// ROS2 binary protocol state machine variables
static ros2_protocol_state_t protocol_state = ROS2_WAITING_MOTOR;
static char current_motor = 0;
static uint8_t int32_bytes[4];  // Buffer to accumulate 4 bytes of int32


void Control_Init(void) {
    // Initialize motors to stopped state
    Control_Stop();

    // Initialize Ultrasonic sensor
    Ultrasonic_Init();

    UART_SendString(2, "Control system initialized (VESC mode)\r\n");
}


// Process incoming byte from UART2 (ROS2 binary protocol)
// Motor: 1 byte ('L' or 'R')] + Scaled Duty Cycle: 4-byte int32, little-endian

void Control_ProcessByte(uint8_t byte) {
    switch(protocol_state) {
        case ROS2_WAITING_MOTOR:
            if (byte == 'L' || byte == 'R') {
                // Valid motor ID - start receiving duty cycle
                current_motor = byte;
                protocol_state = ROS2_WAITING_INT32_BYTE0;
            }
            break;

        case ROS2_WAITING_INT32_BYTE0:
            int32_bytes[0] = byte;  // LSB (little-endian)
            protocol_state = ROS2_WAITING_INT32_BYTE1;
            break;

        case ROS2_WAITING_INT32_BYTE1:
            int32_bytes[1] = byte;
            protocol_state = ROS2_WAITING_INT32_BYTE2;
            break;

        case ROS2_WAITING_INT32_BYTE2:
            int32_bytes[2] = byte;
            protocol_state = ROS2_WAITING_INT32_BYTE3;
            break;

        case ROS2_WAITING_INT32_BYTE3:
            int32_bytes[3] = byte;  // MSB (little-endian)

            // Reconstruct int32 from 4 bytes (little-endian)
            int32_t duty_scaled;
            memcpy(&duty_scaled, int32_bytes, 4);

            // Send to motor using VESC_SetDutyScaled (zero conversion!)
            switch(current_motor) {
                case 'L':
                    VESC_SetDuty(MOTOR_LEFT, duty_scaled);
                    break;
                case 'R':
                    VESC_SetDuty(MOTOR_RIGHT, duty_scaled);
                    break;
            }

            // Reset state machine for next command
            protocol_state = ROS2_WAITING_MOTOR;
            break;
    }
}

// Run safety check w/ultrasonic sensors
void Control_SafetyCheck(void) {
    if (!safetyOverride) {
        Ultrasonic_SafetyCheck();
    }
}

// Apply no current
void Control_Stop(void) {
    VESC_SetDuty(MOTOR_LEFT, 0);
    VESC_SetDuty(MOTOR_RIGHT, 0);
}

// Apply brake current
void Control_Brake(void) {
    int32_t brake_current_mA = 10000; // 10A brake current, conservative and won't cause heat dissipation if the battery can't absorb more current
    
    VESC_SetCurrentBrake(MOTOR_LEFT, brake_current_mA);
    VESC_SetCurrentBrake(MOTOR_RIGHT, brake_current_mA);
}
