[![Demo](https://img.youtube.com/vi/PpxxjT3AoEo/0.jpg)](https://www.youtube.com/watch?v=PpxxjT3AoEo)

Currently configured to run with ROS2 Humble using a realsense D456 camera, HCSR04 ultrasonic sensors, STM32F446RE, dual 6374 BLDC motors run using VESC firmware, and a command computer sufficiently fast enough to run YOLOV11 at 60+FPS. 

STM32 code can be found in ESC controller V3 folder in Nav and needs to be flashed to the STM32F446RE. UART1,2,3, and TIMER5 on the STM32 are used for this implementation.

Left Motor -> USART1 (PA9 and PA10)
Right Motor -> USART3 (PC10 and PC11)
ROS2 Bridge + Ultrasonic Readings -> USART2 (PA2 and PA3)
