#ifndef VESC_H
#define VESC_H

#include <stdint.h>

// VESC Command IDs (from official VESC firmware)
#define COMM_SET_DUTY           5
#define COMM_SET_CURRENT        6
#define COMM_SET_CURRENT_BRAKE  7
#define COMM_SET_RPM            8
#define COMM_SET_POS            9
#define COMM_GET_VALUES         4

// Packet framing bytes
#define PACKET_START_SHORT      2
#define PACKET_START_LONG       3
#define PACKET_STOP             3
#define VESC_MAX_PAYLOAD        512

// Motor selection
typedef enum {
    MOTOR_LEFT = 0,   // Uses USART1 (PA9/PA10)
    MOTOR_RIGHT = 1   // Uses USART3 (PC10/PC11)
} MotorID_t;

// Public VESC motor control API, input parameters are scaled integers to be compatible with VESC without conversion
void VESC_SetDuty(MotorID_t motor, int32_t duty_scaled);
void VESC_SetCurrent(MotorID_t motor, int32_t current_mA);
void VESC_SetCurrentBrake(MotorID_t motor, int32_t brake_current_mA);
void VESC_SetRPM(MotorID_t motor, int32_t rpm);
void VESC_SetPosition(MotorID_t motor, int32_t position_scaled);

#endif /* VESC_H */