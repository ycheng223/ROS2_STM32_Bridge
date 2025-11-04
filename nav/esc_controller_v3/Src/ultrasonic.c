#include "ultrasonic.h"
#include "control.h"
#include "timer.h"
#include "uart.h"
#include <stdio.h>

static volatile uint32_t echo_start = 0;
static volatile uint32_t echo_end = 0;
static volatile uint8_t measurement_complete = 0;

/**
 * @brief Initialize the ultrasonic sensor
 */
void Ultrasonic_Init(void) {
    // Enable GPIO clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    
    // Configure Trig pin as output (PA0)
    ULTRASONIC_TRIG_PORT->MODER &= ~(3UL << (2 * 0)); // Clear bits
    ULTRASONIC_TRIG_PORT->MODER |= (1UL << (2 * 0));  // Set as output
    
    // Configure Echo pin (PA1) as TIM5_CH2 input
    RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
    ULTRASONIC_ECHO_PORT->MODER &= ~(3UL << (2 * 1)); // Clear bits
    ULTRASONIC_ECHO_PORT->MODER |= (2UL << (2 * 1));  // Set as alternate function
    ULTRASONIC_ECHO_PORT->AFR[0] &= ~(0xFUL << (4 * 1)); // Clear AF bits
    ULTRASONIC_ECHO_PORT->AFR[0] |= (2UL << (4 * 1));    // Set AF2 (TIM5_CH2)
    
    // Configure TIM5 for input capture
    TIM5->PSC = 84 - 1;           // 84MHz/84 = 1MHz (1us tick)
    TIM5->ARR = 0xFFFFFFFF;       // Max counter value
    
    // Configure channel 2 for input capture
    TIM5->CCMR1 &= ~TIM_CCMR1_CC2S;       // Clear capture/compare selection
    TIM5->CCMR1 |= TIM_CCMR1_CC2S_0;      // CC2 channel is input, IC2 is mapped on TI2
    
    // Configure for both edges
    TIM5->CCER &= ~TIM_CCER_CC2P;         // Rising edge initially
    TIM5->CCER |= TIM_CCER_CC2E;          // Enable capture
    
    // Enable interrupt on capture
    TIM5->DIER |= TIM_DIER_CC2IE;
    
    // Enable TIM5 interrupt in NVIC
    NVIC_SetPriority(TIM5_IRQn, 3);
    NVIC_EnableIRQ(TIM5_IRQn);
    
    // Start timer
    TIM5->CR1 |= TIM_CR1_CEN;
    
    UART_SendString(2, "Ultrasonic sensor initialized with timer capture\r\n");
}

/**
 * @brief TIM5 interrupt handler
 */
void TIM5_IRQHandler(void) {
    // Check if capture occurred on channel 2
    if (TIM5->SR & TIM_SR_CC2IF) {
        if (!(TIM5->CCER & TIM_CCER_CC2P)) {
            // Rising edge detected
            echo_start = TIM5->CCR2;
            
            // Switch to falling edge detection
            TIM5->CCER |= TIM_CCER_CC2P;
        } else {
            // Falling edge detected
            echo_end = TIM5->CCR2;
            
            // Switch back to rising edge detection
            TIM5->CCER &= ~TIM_CCER_CC2P;
            
            // Mark measurement as complete
            measurement_complete = 1;
        }
        
        // Clear interrupt flag
        TIM5->SR &= ~TIM_SR_CC2IF;
    }
}

/**
 * @brief Measure distance using ultrasonic sensor
 * @return Distance in centimeters or 0xFFFFFFFF if measurement failed
 */
uint32_t Ultrasonic_MeasureDistance(void) {
    uint32_t distance;
    uint32_t timeout = 0;
    
    measurement_complete = 0;
    
    // Generate 10us pulse on Trig pin (PA0)
    ULTRASONIC_TRIG_PORT->BSRR = (1 << 0); // Set PA0
    for (int i = 0; i < 800; i++) __NOP(); // ~10us delay at 84MHz
    ULTRASONIC_TRIG_PORT->BSRR = (1 << (0 + 16)); // Reset PA0
    
    // Wait for measurement to complete or timeout
    while (!measurement_complete && timeout < 25000) {
        timeout++;
        __NOP();
    }
    
    if (timeout >= 25000) {
        return 0xFFFFFFFF; // Measurement failed
    }
    
    // Calculate distance (speed of sound = 343m/s)
    // distance = (time in us) * 343 / 2 / 10000 (cm)
    // Simplified: distance = time * 0.01715 (cm)
    uint32_t echo_time;
    
    // Handle timer overflow
    if (echo_end > echo_start) {
        echo_time = echo_end - echo_start;
    } else {
        echo_time = (0xFFFFFFFF - echo_start) + echo_end + 1;
    }
    
    // time in us * 343 / 2 / 10000 simplified with fixed-point math
    distance = (echo_time * 1715) / 100000;
    
    return distance;
}

/**
 * @brief Check distance and trigger safety stop if object is too close
 */
void Ultrasonic_SafetyCheck(void) {
    uint32_t distance = Ultrasonic_MeasureDistance();
    
    // Valid measurement and within safety threshold
    if (distance != 0xFFFFFFFF && distance < SAFETY_DISTANCE_CM) {
        // Stop all motors
        Control_Stop();
        
        // Notify via UART
        char buffer[50];
        sprintf(buffer, "SAFETY STOP: Object detected at %lu cm\r\n", distance);
        UART_SendString(2, buffer);
    }
}
