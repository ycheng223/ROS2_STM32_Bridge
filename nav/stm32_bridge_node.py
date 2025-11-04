#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist
import serial
import time
import threading
import struct

class STM32Bridge(Node):

    def __init__(self):
        super().__init__('stm32_bridge')

        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)

        # Motor parameters
        self.declare_parameter('max_velocity', 1.2)
        self.declare_parameter('angular_scale', 1.0)

        # Get parameters
        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value

        # Initialize serial connection
        self.get_logger().info(f'Connecting to {serial_port} at {baud_rate} baud')
        try:
            self.serial_port = serial.Serial(serial_port, baud_rate, timeout=1.0)
            time.sleep(2)  # Wait for Arduino/STM32 to reset
            self.get_logger().info('Successfully connected to STM32')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to serial port: {e}')
            rclpy.shutdown()
            return

        # Publishers
        self.distance_publisher = self.create_publisher(Float32, 'ultrasonic_distance', 10)
        self.raw_data_publisher = self.create_publisher(String, 'stm32_raw_data', 10)

        # Subscribers
        self.cmd_vel_subscriber = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.command_subscriber = self.create_subscription(
            String, 'stm32_command', self.command_callback, 10)

        # Start serial reading thread
        self.serial_thread = threading.Thread(target=self.serial_reader_thread, daemon=True)
        self.serial_thread.start()

        self.get_logger().info('STM32 Bridge Node initialized')


    def serial_reader_thread(self):
        """Continuously read from serial port in separate thread"""
        while rclpy.ok():
            try:
                if self.serial_port.in_waiting > 0:
                    line = self.serial_port.readline().decode('utf-8').strip()
                    if line:
                        self.process_serial_data(line)
            except Exception as e:
                self.get_logger().error(f'Serial read error: {e}')
                time.sleep(0.1)


    def process_serial_data(self, data):
        """Process incoming serial data from STM32"""
        # Publish raw data for debugging
        raw_msg = String()
        raw_msg.data = data
        self.raw_data_publisher.publish(raw_msg)

        # Parse structured data
        if data.startswith('Ultrasonic distance:'):
            # Parse: "Ultrasonic distance: 25 cm"
            try:
                distance_str = data.split(':')[1].strip().split()[0]
                if distance_str != "Out":  # Handle "Out of range"
                    distance = float(distance_str)

                    distance_msg = Float32()
                    distance_msg.data = distance
                    self.distance_publisher.publish(distance_msg)

                    self.get_logger().debug(f'Distance: {distance} cm')
            except (IndexError, ValueError) as e:
                self.get_logger().warn(f'Failed to parse distance: {data}')

        # Handle other STM32 messages
        elif "Moving" in data:
            self.get_logger().info(f'STM32: {data}')
        elif "SAFETY STOP" in data:
            self.get_logger().warn(f'STM32 Safety: {data}')


    # Send motor command as 5 byte binary packet (i.e. motor (1 byte character) + scaled duty cycle (4 bytes int32 little-endian for STM32 compatability))
    def send_binary_motor_command(self, motor, duty_scaled_int32):
        motor_byte = ord(motor)  # 'L' → 0x4C
        
        # Pack int32 as 4 bytes (little-endian signed)
        duty_bytes = struct.pack('<i', duty_scaled_int32)  # '<i' = little-endian signed int32

        # Combine: 1 motor byte + 4 int32 bytes = 5 bytes total
        binary_data = bytes([motor_byte]) + duty_bytes

        self.serial_port.write(binary_data)


    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        ANGULAR_SCALE = self.get_parameter('angular_scale').get_parameter_value().double_value
        MAX_VELOCITY = self.get_parameter('max_velocity').get_parameter_value().double_value
        
        # Differential drive math
        left_speed = linear_x - (angular_z * ANGULAR_SCALE)
        right_speed = linear_x + (angular_z * ANGULAR_SCALE)
        
        # Convert velocity to VESC duty cycle (-1.0 to 1.0)
        def velocity_to_duty_scaled(velocity):
            # Clamp to ±MAX_VELOCITY
            velocity = max(-MAX_VELOCITY, min(MAX_VELOCITY, velocity))
            # Normalize to ±1.0
            duty = velocity / MAX_VELOCITY
            # VESC requires uint32_t input over UART so scale duty cycle by 100000 to get value compatible with VESC integer range
            duty_scaled = int(duty * 100000)
            return duty_scaled
        
        left_duty_scaled = velocity_to_duty_scaled(left_speed)
        right_duty_scaled = velocity_to_duty_scaled(right_speed)
        
        self.send_binary_motor_command('L', left_duty_scaled)
        self.send_binary_motor_command('R', right_duty_scaled)


    def command_callback(self, msg):
        """Handle direct STM32 commands"""
        command = msg.data
        self.send_command(command) 


    def send_command(self, command):
        """Send single character command to STM32"""
        try:
            self.serial_port.write(command.encode('utf-8'))
            self.get_logger().debug(f'Sent command: {command}')
        except Exception as e:
            self.get_logger().error(f'Failed to send command: {e}')


    def destroy_node(self):
        """Clean shutdown"""
        if hasattr(self, 'serial_port') and self.serial_port.is_open:
            self.send_command(' ')  # Stop motors
            self.serial_port.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = STM32Bridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()