#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include "stm32f4xx.h"
#include <stdint.h>

// Function prototypes
void Ultrasonic_Init(void);
uint32_t Ultrasonic_MeasureDistance(void);
void Ultrasonic_SafetyCheck(void);

// Configuration parameters
#define ULTRASONIC_TRIG_PORT GPIOA
#define ULTRASONIC_ECHO_PORT GPIOA
#define SAFETY_DISTANCE_CM   20

#endif /* ULTRASONIC_H */
