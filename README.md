[![Demo](https://img.youtube.com/vi/PpxxjT3AoEo/0.jpg)](https://www.youtube.com/watch?v=PpxxjT3AoEo)

Currently configured to run with ROS2 Humble using a realsense D456 camera, HCSR04 ultrasonic sensors, STM32F446RE, dual 6374 BLDC motors run using VESC firmware, and a command computer sufficiently fast enough to run YOLOV11 at 60+FPS. 

STM32 code can be found in ESC controller V3 folder in Nav and needs to be flashed to the STM32F446RE. UART1/2/3 on the STM32 is used for this implementation.
