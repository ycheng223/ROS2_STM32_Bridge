#ifndef TIMER_H
#define TIMER_H

#include "stm32f4xx.h"
#include <stdint.h>

// Timer configuration constants
#define PPM_FRAME_LENGTH  22000  // 22 ms frame (45.45 Hz)
#define PPM_PULSE_WIDTH   9000   // PPM width for standby
#define PWM_FRAME_LENGTH  22000  // same as PPM
#define PWM_PULSE_WIDTH   9000   // PWM width for standby

// Configure Timer 3 for PPM output on PC6
void Timer_ConfigurePPM(void);

// Configure Timer 4 for PWM output on PB6
void Timer_ConfigurePWM(void);

#endif /* TIMER_H */
