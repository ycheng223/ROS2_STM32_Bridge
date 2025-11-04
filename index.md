# Nav Package - Codebase Index

## Package Overview
The **nav** ROS2 package provides a bridge between ROS2 and an STM32 microcontroller for robot control with Bluetooth controller support.

## Directory Structure
```
/home/jyuc/ros2_ws/src/nav/
├── CLAUDE.md                    # Comprehensive documentation on controller integration
├── config/                      # Empty (configuration files would go here)
├── launch/
│   ├── controller_launch.py     # Joystick + teleop + STM32 bridge launcher
│   ├── full_launch.py          # Complete system: RealSense + YOLO + controller
│   └── nav_launch.py           # Basic STM32 bridge launcher
├── nav/
│   ├── __init__.py             # Package initialization (empty)
│   └── stm32_bridge_node.py    # Main STM32 serial communication node
├── resource/nav                # ROS2 resource marker (empty)
├── package.xml                 # Package dependencies & metadata (v0.5.0)
├── setup.cfg                   # Installation configuration
└── setup.py                    # Python package setup (v0.0.1)
```

## Core Components

### 1. STM32Bridge Node
**File:** [nav/stm32_bridge_node.py](nav/stm32_bridge_node.py)

**Subscribes to:**
- `/nav/cmd_vel` (geometry_msgs/Twist): Velocity commands from controller
- `/nav/stm32_command` (std_msgs/String): Direct command interface

**Publishes:**
- `/nav/ultrasonic_distance` (std_msgs/Float32): Distance sensor readings
- `/nav/stm32_raw_data` (std_msgs/String): Raw serial debug data

**Parameters:**
- `serial_port`: `/dev/ttyACM0` (default)
- `baud_rate`: 115200
- `motor_neutral_left`: 9200 (PWM neutral position)
- `motor_neutral_right`: 9360 (PWM neutral position)
- `pwm_range`: 1800 (PWM range from neutral)
- `max_velocity`: 1.5 (m/s)
- `angular_scale`: 1.0 (angular velocity scaling)
- `min_pwm_offset`: 220 (minimum PWM offset to prevent motor jitter)

**Key Functions:**
- `cmd_vel_callback()` (line 111): Converts Twist messages to differential drive PWM
- `send_binary_motor_command()` (line 103): Sends 3-byte binary motor commands
- `serial_reader_thread()` (line 60): Non-blocking serial data reading
- `process_serial_data()` (line 73): Parses incoming sensor data
- `velocity_to_pwm()` (line 128): Converts velocity to PWM with deadband

**Protocol:**
- Binary format: [motor_id_byte, pwm_low_byte, pwm_high_byte]
- Motor IDs: 'R' (right), 'L' (left)

### 2. Controller Launch
**File:** [launch/controller_launch.py](launch/controller_launch.py)

**Nodes:**
1. **joy_node** (joy package)
   - Device: `/dev/input/js0`
   - Deadzone: 0.05
   - Autorepeat rate: 10.0 Hz

2. **teleop_twist_joy**
   - Linear axis: 1 (left stick vertical)
   - Angular axis: 0 (left stick horizontal)
   - Normal speed: 1.2 m/s linear, 1.2 rad/s angular
   - Turbo speed: 1.5 m/s linear, 1.5 rad/s angular
   - Enable button: 4 (L1)
   - Turbo button: 5 (R1)

3. **STM32Bridge** (via nav_launch.py)

### 3. Navigation Launch
**File:** [launch/nav_launch.py](launch/nav_launch.py)

Launches STM32Bridge node with default serial port and baud rate parameters.

### 4. Full System Launch
**File:** [launch/full_launch.py](launch/full_launch.py)

Integrates multiple subsystems:
1. RealSense camera (from yolo_realsense package)
2. YOLO object detection processor
3. Controller system (joystick + STM32 bridge)

## Dependencies

### Package Dependencies (package.xml)
- **Core:** rclpy, std_msgs, geometry_msgs
- **Controller:** joy, teleop_twist_joy
- **Testing:** ament_copyright, ament_flake8, ament_pep257, python3-pytest

### Python Setup (setup.py)
- **Entry point:** `stm32_bridge_node = nav.stm32_bridge_node:main`
- Installs launch files to share directory

## Key Features

### 1. Differential Drive Kinematics
```python
# From stm32_bridge_node.py:118-119
left_speed = linear_x - (angular_z * ANGULAR_SCALE)
right_speed = linear_x + (angular_z * ANGULAR_SCALE)
```

### 2. Velocity to PWM Conversion (lines 128-139)
- Clamps velocity to max_velocity range
- Applies minimum PWM offset for forward/reverse
- Returns neutral PWM for zero velocity
- Prevents motor jitter at low speeds

### 3. Binary Motor Protocol
- Efficient 3-byte commands
- Motor ID as ASCII character byte
- 16-bit PWM value (little-endian)

### 4. Threaded Serial Communication
- Non-blocking serial reads in separate thread
- Publishes raw data for debugging
- Parses structured sensor data (ultrasonic)

### 5. Safety Features
- Motor stop on node shutdown (line 167)
- Velocity clamping to prevent over-speed
- Minimum PWM threshold to prevent jitter
- Serial timeout handling

## Data Flow

### Controller to Motors
```
Bluetooth Controller
  → /dev/input/js0
  → joy_node
  → /nav/joy
  → teleop_twist_joy
  → /nav/cmd_vel
  → STM32Bridge
  → Serial (binary PWM)
  → STM32 ESC Controller
```

### Sensor Data Flow
```
STM32 Ultrasonic Sensor
  → Serial (text)
  → STM32Bridge
  → /nav/ultrasonic_distance
```

## Configuration Notes

1. **Version Mismatch:** package.xml (v0.5.0) vs setup.py (v0.0.1)
2. **No YAML Config:** All parameters are inline in launch files
3. **Empty Config Directory:** /home/jyuc/ros2_ws/src/nav/config exists but unused
4. **Joy vs Joy_Linux:** Implementation uses `joy` package, but CLAUDE.md documents `joy_linux`

## Usage Commands

### Launch Options
```bash
# Basic STM32 bridge only
ros2 launch nav nav_launch.py

# Controller + STM32 bridge
ros2 launch nav controller_launch.py

# Full system (camera + YOLO + controller)
ros2 launch nav full_launch.py
```

### Manual Node Execution
```bash
# Run STM32 bridge node
ros2 run nav stm32_bridge_node

# Test joystick
ros2 run joy joy_node --ros-args -p dev:="/dev/input/js0"

# Monitor topics
ros2 topic echo /nav/cmd_vel
ros2 topic echo /nav/joy
ros2 topic echo /nav/ultrasonic_distance
```

### Hardware Testing
```bash
# List joystick devices
ls /dev/input/js*

# Test joystick input
jstest /dev/input/js0

# Check serial port
ls -l /dev/ttyACM*
```

## Architecture Diagram
```
┌─────────────────────────────────────────────────────┐
│                    nav Package                       │
├─────────────────────────────────────────────────────┤
│                                                      │
│  ┌──────────────┐    ┌──────────────────────────┐  │
│  │   joy_node   │───▶│  teleop_twist_joy_node   │  │
│  │ (joy pkg)    │    │  (teleop_twist_joy pkg)  │  │
│  └──────────────┘    └──────────────────────────┘  │
│         │                        │                   │
│   /nav/joy                 /nav/cmd_vel             │
│                                  │                   │
│                                  ▼                   │
│                       ┌─────────────────────┐       │
│                       │  STM32Bridge Node   │       │
│                       │  (nav package)      │       │
│                       └─────────────────────┘       │
│                                  │                   │
│                          Serial (binary)             │
│                                  │                   │
└──────────────────────────────────┼───────────────────┘
                                   ▼
                          ┌─────────────────┐
                          │  STM32 ESC      │
                          │  Controller     │
                          └─────────────────┘
```

## Related Packages
- **yolo_realsense**: Provides RealSense camera and YOLO detection nodes
- **joy**: Linux joystick driver (standard ROS2 package)
- **teleop_twist_joy**: Joystick to Twist converter (standard ROS2 package)

## Last Updated
Generated: 2025-10-06