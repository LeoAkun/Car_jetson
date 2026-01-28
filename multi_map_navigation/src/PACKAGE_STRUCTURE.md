# Package Structure

This directory contains two separate ROS2 packages for the multi-map navigation system:

## 1. multi_map_navigation_msgs (CMakeLists.txt package)

**Location:** `src/multi_map_navigation_msgs/`

**Build Type:** `ament_cmake`

**Purpose:** Contains custom message definitions for the multi-map navigation system.

**Contents:**
- `msg/Waypoint.msg` - Single waypoint definition with GPS and local coordinates
- `msg/WaypointList.msg` - List of waypoints for navigation tasks
- `msg/MapSwitchTrigger.msg` - Trigger message for map switching operations

**Dependencies:**
- std_msgs
- geometry_msgs

**Build:**
```bash
cd /home/akun/workspace/Car_jetson/multi_map_navigation
colcon build --packages-select multi_map_navigation_msgs
```

## 2. multi_map_navigation (setup.py package)

**Location:** `src/multi_map_navigation_py/`

**Build Type:** `ament_python`

**Purpose:** Contains Python nodes for MQTT communication and navigation management.

**Contents:**
- `multi_map_navigation/communication/` - MQTT communication nodes
  - `mqtt_task_receiver.py` - Receives tasks from MQTT broker
  - `status_reporter.py` - Reports robot status to MQTT broker
- `multi_map_navigation/navigation/` - Navigation management nodes
  - `navigation_manager.py` - Manages waypoint navigation sequence
  - `map_switch_controller.py` - Handles map switching operations
  - `process_manager.py` - Manages ROS2 process lifecycle
- `launch/` - Launch files
- `config/` - Configuration files

**Dependencies:**
- rclpy
- std_msgs
- geometry_msgs
- sensor_msgs
- nav_msgs
- nav2_msgs
- multi_map_navigation_msgs (custom messages)

**Build:**
```bash
cd /home/akun/workspace/Car_jetson/multi_map_navigation
colcon build --packages-select multi_map_navigation
```

## Build Order

Due to dependencies, packages must be built in this order:

1. First build `multi_map_navigation_msgs` (message definitions)
2. Then build `multi_map_navigation` (Python nodes that use the messages)

**Build all packages:**
```bash
cd /home/akun/workspace/Car_jetson/multi_map_navigation
colcon build
```

## Package Separation Rationale

The packages are separated to follow ROS2 best practices:

1. **Message definitions (CMakeLists.txt):** Message, service, and action definitions should use `ament_cmake` build type for proper code generation across multiple languages (C++, Python, etc.)

2. **Python nodes (setup.py):** Pure Python nodes can use `ament_python` build type with setup.py for simpler dependency management and faster builds.

This separation also allows:
- Independent versioning of messages and implementation
- Reusing message definitions in other packages
- Faster iteration on Python code without rebuilding messages
- Better organization and maintainability

## Usage

After building, source the workspace and run nodes:

```bash
source install/setup.bash

# Run individual nodes
ros2 run multi_map_navigation mqtt_task_receiver
ros2 run multi_map_navigation navigation_manager
ros2 run multi_map_navigation map_switch_controller

# Or use launch files
ros2 launch multi_map_navigation system.launch.py
```
