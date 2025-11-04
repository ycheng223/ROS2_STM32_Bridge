#include <stdint.h>
#include "stm32f4xx.h"

#include <uart.h>
#include <vesc.h>


// Calculate CRC16 checksum for VESC packets
// Remember, CRC is POLYNOMIAL BINARY DIVISION
// Take data packet inputs before send and after receive, perform polynomial division (NOT THE SAME AS LONG DIVISION) on both and COMPARE
// Polynomial division: Shift binary left, if we see a "one", perform XOR (^) with generator polynomial
// Generator polynomial = 0x1021 = 1*x^3 + 2*x^1 + 1*x^0

static uint16_t VESC_CRC16(const uint8_t *data, uint32_t len) {
    uint16_t crc = 0;

    for (uint32_t i = 0; i < len; i++) { // 32 bit to match with "len" parameter
        crc ^= data[i] << 8; // XOR to load data into upper 8 bits of CRC register
        for (uint8_t j = 0; j < 8; j++) { // iterate through each bit in the packet
            if (crc & 0x8000) { // If leftmost CRC bit is 1
                crc = (crc << 1) ^ 0x1021; // Shift left and XOR with generator polynomial (0x1021 i.e. x^3 + 2x^2 + 1)
            } else {
                crc = crc << 1; // Shift left

            }
        } // packet done
    } // message done
    return crc;
} // We can now reject the packet if the CRC doesn't match between sent and received


// Convert packet in buffer to big-endian (MSB first), required by VESC
// STM32 is little endian internally
static void VESC_AppendInt32(uint8_t *buffer, int32_t number, uint32_t *index) {
    buffer[(*index)++] = (number >> 24) & 0xFF;
    buffer[(*index)++] = (number >> 16) & 0xFF;
    buffer[(*index)++] = (number >> 8) & 0xFF;
    buffer[(*index)++] = number & 0xFF;
}


// Format packet for VESC specifications (UART + CRC)
static uint32_t VESC_FormatPacket(uint8_t *buffer, 
                                   uint8_t cmd, 
                                   const uint8_t *payload, 
                                   uint32_t payload_len)
{
    uint32_t packet_index = 0;
    uint32_t payload_with_cmd_len = payload_len + 1; // +1 for command byte
    
    // Add start byte and specify packet format + length
    if (payload_with_cmd_len <= 256) {  // Use only MSB as start byte if msg length <256
        buffer[packet_index++] = PACKET_START_SHORT;
        buffer[packet_index++] = (uint8_t)payload_with_cmd_len;
    } else { // Otherwise use long packet (MSB + LSB)
        buffer[packet_index++] = PACKET_START_LONG;
        buffer[packet_index++] = (uint8_t)(payload_with_cmd_len >> 8);
        buffer[packet_index++] = (uint8_t)(payload_with_cmd_len & 0xFF);
    }
    // Add command ID
    buffer[packet_index++] = (uint8_t)cmd;
    // Add payload
    for (uint32_t i = 0; i < payload_len; i++) {
        buffer[packet_index++] = payload[i];
    }
    // Add CRC (calculated over payload + command)
    uint16_t crc = VESC_CRC16(buffer + (payload_with_cmd_len <= 256 ? 2 : 3), 
                               payload_with_cmd_len);
    buffer[packet_index++] = (uint8_t)(crc >> 8);
    buffer[packet_index++] = (uint8_t)(crc & 0xFF);
    // Add stop byte
    buffer[packet_index++] = PACKET_STOP;
    
    return packet_index; // return the total packet length
}


// Send formatted packet over to motor ESC via UART
static void VESC_SendBuffer(MotorID_t motor, const uint8_t *buffer, uint32_t length)
{
    switch (motor) {
        case MOTOR_LEFT:
            for (uint32_t i = 0; i < length; i++) {
                UART_SendByte(USART1, buffer[i]);
            }
            break;

        case MOTOR_RIGHT:
            for (uint32_t i = 0; i < length; i++) {
                UART_SendByte(USART3, buffer[i]);
            }
            break;

        default:
            // Invalid motor ID - do nothing
            break;
    }
}


// ============================================================================
// Public VESC Motor Control API
// ============================================================================


// Set the duty cycle for the specific motor
// duty_scaled = duty_cycle * 1000
void VESC_SetDuty(MotorID_t motor, int32_t duty_scaled) {
    // Clamp to valid range (-100000 to 100000 for VESC Duty Cycles)
    if (duty_scaled > 100000) duty_scaled = 100000;
    if (duty_scaled < -100000) duty_scaled = -100000;
    
    // Convert to big-endian bytes for VESC
    uint8_t payload[4];
    uint32_t index = 0;
    VESC_AppendInt32(payload, duty_scaled, &index);
    
    // Format and send packet
    uint8_t packet[20];
    uint32_t len = VESC_FormatPacket(packet, COMM_SET_DUTY, payload, 4);
    VESC_SendBuffer(motor, packet, len);
}


// Set the motor current for the specific motor
// scaled_current (current_mA) = current (A) * 1000
void VESC_SetCurrent(MotorID_t motor, int32_t current_mA) {
    // Clamp to motor maximum (85A = 85000 mA)
    if (current_mA > 85000) current_mA = 85000;
    if (current_mA < -85000) current_mA = -85000;
    
    uint8_t payload[4];
    uint32_t index = 0;
    VESC_AppendInt32(payload, current_mA, &index);
    
    uint8_t packet[20];
    uint32_t len = VESC_FormatPacket(packet, COMM_SET_CURRENT, payload, 4);
    VESC_SendBuffer(motor, packet, len);
}


// Set the brake current to apply against the motor rotation for regenerative braking
// Also scaled by 1000
void VESC_SetCurrentBrake(MotorID_t motor, int32_t brake_current_mA) {
    // Clamp to valid range (0 to 85A = 85000 mA)
    if (brake_current_mA < 0) brake_current_mA = 0;
    if (brake_current_mA > 85000) brake_current_mA = 85000;
    
    uint8_t payload[4];
    uint32_t index = 0;
    VESC_AppendInt32(payload, brake_current_mA, &index);
    
    uint8_t packet[20];
    uint32_t len = VESC_FormatPacket(packet, COMM_SET_CURRENT_BRAKE, payload, 4);
    VESC_SendBuffer(motor, packet, len);
}


// Set the RPM for the specific motor
// No scaling per VESC
void VESC_SetRPM(MotorID_t motor, int32_t rpm) {
    uint8_t payload[4];
    uint32_t index = 0;
    VESC_AppendInt32(payload, rpm, &index);

    uint8_t packet[20];
    uint32_t len = VESC_FormatPacket(packet, COMM_SET_RPM, payload, 4);
    VESC_SendBuffer(motor, packet, len);
}


// Set the motor position for the specific motor
// position_scaled = position * 1*10^6
void VESC_SetPosition(MotorID_t motor, int32_t position_scaled) {
    // Clamp to valid range (0 to 360 degrees = 0 to 360,000,000)
    if (position_scaled < 0) position_scaled = 0;
    if (position_scaled > 360000000) position_scaled = 360000000;

    uint8_t payload[4];
    uint32_t index = 0;
    VESC_AppendInt32(payload, position_scaled, &index);
    
    uint8_t packet[20];
    uint32_t len = VESC_FormatPacket(packet, COMM_SET_POS, payload, 4);
    VESC_SendBuffer(motor, packet, len);
}