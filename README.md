# TE3001B - Kinematics Challenge - Haptic Teleoperation
Dr. Luis Alberto Muñoz Ubando

## Overview

This project implements a haptic teleoperation system for the **xArm6** robot using ROS 2. A force sensor connected via an Arduino (micro-ROS) monitors contact forces in real time and automatically stops the robot when a force threshold is exceeded.

### Components

- **force_stop_node.py** — ROS 2 node that subscribes to `/force_sensor`, monitors force readings, and stops the xArm6 by canceling its trajectory when contact is detected.
- **Force_Sensor.ino** — Arduino sketch that reads an analog force sensor (GPIO 34), averages 10 samples, and publishes `Int32` messages to `/force_sensor` at ~10 Hz via micro-ROS.
- **xarm_ros2** — Full xArm ROS 2 driver stack (controllers, MoveIt config, Gazebo simulation).
- **pymoveit2** — Python interface for MoveIt 2 motion planning.

## Prerequisites

- **ROS 2** (Humble or later)
- **MoveIt 2**
- **micro-ROS Agent** (for Arduino communication)
- **Arduino IDE** with micro-ROS library (for flashing the sensor firmware)
- **xArm6** robot or Gazebo simulation

## Setup

### 1. Clone the repository

```bash
git clone --recurse-submodules https://github.com/<your-org>/Challenge-Kinematics.git
cd Challenge-Kinematics
```

If you already cloned without submodules:

```bash
git submodule update --init --recursive
```

### 2. Install dependencies

```bash
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Build the workspace

```bash
colcon build --symlink-install
source install/setup.bash
```

### 4. Flash the Arduino

1. Open `Force_Sensor.ino` in the Arduino IDE.
2. Install the **micro-ROS Arduino** library.
3. Connect the force sensor to **GPIO 34** and the built-in LED is on **GPIO 2**.
4. Flash the sketch to your ESP32 board.

## How to Run

### Step 1 — Start the micro-ROS Agent

Connect the Arduino via USB and launch the agent:

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
```

> Adjust `/dev/ttyUSB0` to your actual serial port.

### Step 2 — Launch the xArm6 controller

**Real robot:**

```bash
ros2 launch xarm_controller xarm6_control_rviz_display.launch.py
```

**Simulation (Gazebo):**

```bash
ros2 launch xarm_gazebo xarm6_beside_table.launch.py
```

### Step 3 — Run the force stop node

```bash
ros2 run force_stop_node force_stop_node
```

Or run the script directly:

```bash
python3 src/force_stop_node.py
```

### Step 4 — (Optional) Launch MoveIt Servo for teleoperation

```bash
ros2 launch xarm_moveit_servo xarm_moveit_servo_realmove.launch.py
```

For simulation:

```bash
ros2 launch xarm_moveit_servo xarm_moveit_servo_fake.launch.py
```

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/force_sensor` | `std_msgs/msg/Int32` | Force sensor reading from Arduino |
| `/xarm6_traj_controller/follow_joint_trajectory` | Action | xArm6 trajectory controller |

## Parameters

The `force_stop_node` accepts the following ROS 2 parameters:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `force_threshold` | `3900` | Force reading below this value triggers a stop |