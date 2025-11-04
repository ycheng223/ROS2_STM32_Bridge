#include <timer.h>

// Enable PPM signal on pin PC6 (on top right side of nucleo, the right column of morpho pins) using channel 1 of timer 3

/* CURRENTLY DISABLED AS WE DON'T USE PWM FOR MOTOR CONTROL IN THIS VERSION
void Timer_ConfigurePPM(void) {
    // Enable GPIO and Timer Clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    // Configure PC6 (TIM3_CH1) as Alternate Function (AF2)
    GPIOC->MODER |= GPIO_MODER_MODER6_1;          // Alternate function mode
    GPIOC->AFR[0] &= ~(0xF << (6 * 4));          // Clear AF for PC6
    GPIOC->AFR[0] |= (2 << (6 * 4));             // AF2 (TIM3_CH1)

    // Configure TIM3 for PPM
    TIM3->PSC = 15;                              // 1 MHz timer (16 MHz / 16)
    TIM3->ARR = PPM_FRAME_LENGTH - 1;            // 22 ms frame (22000 µs)
    TIM3->CCR1 = PPM_PULSE_WIDTH;                // 1.5 ms pulse
    TIM3->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE; // PWM1 mode + preload
    TIM3->CCER |= TIM_CCER_CC1E;                 // Enable CH1 output
    TIM3->CR1 |= TIM_CR1_ARPE | TIM_CR1_CEN;     // Enable auto-reload & start timer
}
*/

// Enable PWM signal on pin PB6 (on middle right side of nucleo, the left column of morpho pins) using channel 1 of timer 4.
// Need to keep some distance between PWM signal generating pins to minimize emf interference.

/* CURRENTLY DISABLED AS WE DON'T USE PWM FOR MOTOR CONTROL IN THIS VERSION
void Timer_ConfigurePWM(void) {
    // Enable GPIO and Timer Clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

    // Configure PB6 (TIM4_CH1) as Alternate Function (AF2)
    GPIOB->MODER |= GPIO_MODER_MODER6_1;         // Alternate function mode
    GPIOB->AFR[0] &= ~(0xF << (6 * 4));          // Clear AF for PC6
    GPIOB->AFR[0] |= (2 << (6 * 4));             // AF2 (TIM3_CH1)

    // Configure Timer 4 for PWM
    TIM4->PSC = 15;                              // 1 MHz timer (16 MHz / 16)
    TIM4->ARR = PWM_FRAME_LENGTH - 1;            // 22 ms frame (22000 µs)
    TIM4->CCR1 = PWM_PULSE_WIDTH;                // Initialize Duty Cycle to 0
    TIM4->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE; // PWM1 mode + preload
    TIM4->CCER |= TIM_CCER_CC1E;                 // Enable CH1 output
    TIM4->CR1 |= TIM_CR1_ARPE | TIM_CR1_CEN;     // Enable auto-reload & start timer
}
*/