# TE3001B - Kinematics Challenge - Haptic Teleoperation
Dr. Luis Alberto Muñoz Ubando

## Overview

This project implements a **bilateral haptic teleoperation** system for two **xArm Lite 6** robots using ROS 2. A human operator moves the master robot by hand, and the slave robot replicates the movements in real time. When the slave detects a collision (via joint effort or an FSR force sensor), the system reflects force back to the master so the operator feels resistance.

### Hardware

| Robot | IP | Role |
|---|---|---|
| xArm Lite 6 | 192.168.1.175 | **Slave** — replicates master movements |
| xArm Lite 6 | 192.168.1.167 | **Master** — moved by human operator |
| ESP32 + micro-ROS | USB `/dev/ttyUSB0` | FSR force sensor |

### Components

- **bilateral_teleop.py** — Main ROS 2 node (FSM + effort-based haptic feedback)
- **master_teleop.py** — Master robot node
- **Force_Sensor.ino** — ESP32 sketch that reads an analog force sensor and publishes `Int32` messages to `/force_sensor` via micro-ROS
- **xarm_ros2** — Full xArm ROS 2 driver stack (controllers, MoveIt config)
- **pymoveit2** — Python interface for MoveIt 2 motion planning

## Prerequisites

- **ROS 2 Humble**
- **MoveIt 2**
- **micro-ROS Agent** (`ros-humble-micro-ros-agent`)
- **topic_tools** (`ros-humble-topic-tools`)
- **xarm-python-sdk** (`pip install xarm-python-sdk`)
- **Arduino IDE** with micro-ROS library (for flashing sensor firmware)
- Two **xArm Lite 6** robots on the same network

## Setup

### 1. Clone the repository

```bash
git clone --recurse-submodules https://github.com/<your-org>/Challenge-Kinematics.git
cd ~/Desktop/Challenge-Kinematics
```

If you already cloned without submodules:

```bash
git submodule update --init --recursive
```

### 2. Install dependencies

```bash
sudo apt install ros-humble-topic-tools ros-humble-micro-ros-agent
pip install xarm-python-sdk
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Build the workspace

```bash
cd ~/Desktop/Challenge-Kinematics
colcon build --symlink-install
source install/setup.bash
```

### 4. Flash the ESP32

1. Open `Force_Sensor.ino` in the Arduino IDE.
2. Install the **micro-ROS Arduino** library.
3. Connect the force sensor to **GPIO 34**.
4. Flash the sketch to your ESP32 board.

## How to Run — Bilateral Teleoperation (6 terminals)

Open each terminal in order and wait for the confirmation message before proceeding.

### Terminal 1 — micro-ROS Agent (ESP32 force sensor)

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
```

Confirm: ESP32 connects and session logs appear.

### Terminal 2 — Slave MoveIt

```bash
cd ~/Desktop/Challenge-Kinematics && source install/setup.bash
ros2 launch xarm_moveit_config lite6_moveit_realmove.launch.py \
  robot_ip:=192.168.1.175 \
  add_realsense_d435i:=false
```

Confirm: `You can start planning now!`

### Terminal 3 — Master driver (namespace /master)

```bash
cd ~/Desktop/Challenge-Kinematics && source install/setup.bash
ros2 run xarm_api xarm_driver_node \
  --ros-args \
  -r __ns:=/master \
  -p robot_ip:=192.168.1.167 \
  -p dof:=6 \
  -p report_type:=dev
```

Confirm: `[TCP STATUS] CONTROL: 1`

> **`report_type:=dev` is required.** With `normal` you only get ~5 Hz and calibration fails.

### Terminal 4 — Master topic relay

```bash
cd ~/Desktop/Challenge-Kinematics && source install/setup.bash
ros2 run topic_tools relay \
  /master/xarm/joint_states \
  /master/joint_states
```

Confirm: no errors, terminal stays silent.

### Terminal 5 — MoveIt Servo

```bash
cd ~/Desktop/Challenge-Kinematics && source install/setup.bash
ros2 launch xarm_moveit_servo lite6_moveit_servo_realmove.launch.py \
  robot_ip:=192.168.1.175
```

Confirm: `Loaded node '/servo_server'`

> `Failed loading controller lite6_traj_controller` is **normal** — Terminal 2 already loaded it.

### Terminal 6 — Bilateral teleoperation node

```bash
cd ~/Desktop/Challenge-Kinematics && source install/setup.bash
ros2 run haptic_teleop bilateral_teleop --ros-args \
  --params-file src/haptic_teleop/config/teleop_params.yaml
```

Confirm: `Calibration OK → RUNNING`

The node moves the master to HOME automatically, waits 2 seconds, then calibrates. **Do not touch either robot during calibration.**

## Usage

Once the node is in **RUNNING** state, move the **master robot** by hand and the **slave** will replicate the movements. If the slave hits an obstacle, you will feel resistance in the master (haptic feedback).

## Topics

| Topic | Type | Description |
|---|---|---|
| `/master/xarm/joint_states` | `sensor_msgs/JointState` | Master joint state (~90 Hz) |
| `/master/joint_states` | `sensor_msgs/JointState` | Master relay |
| `/joint_states` | `sensor_msgs/JointState` | Slave joint state + efforts (~150 Hz) |
| `/force_sensor` | `std_msgs/Int32` | ESP32 ADC (rest ~4095, contact <3900) |
| `/lite6_traj_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | Trajectory commands to slave |
| `/master/reflected_torques` | `std_msgs/Float64MultiArray` | Haptic feedback torques to master |

## Parameters

See [`src/haptic_teleop/config/teleop_params.yaml`](src/haptic_teleop/config/teleop_params.yaml) for all tunable parameters.

| Parameter | Default | Description |
|---|---|---|
| `traj_time_ms` | 60 | Trajectory duration sent to slave [ms] |
| `still_thr_deg` | 0.1 | Movement threshold to consider master still [deg] |
| `still_cycles` | 3 | Still cycles before sending hold to slave |
| `force_threshold` | 3900 | ADC threshold to detect FSR contact |
| `force_hysteresis` | 3950 | ADC threshold to release FSR contact |
| `calib_samples` | 80 | Samples for calibration |
| `watchdog_timeout` | 0.5 | Time without force data before EMERGENCY [s] |
| `torque_gain` | 0.15 | Haptic feedback gain for position error |

## Error Recovery (C31)

If the slave throws error C31 (`Collision Caused Abnormal Joint Current`):

```bash
python3 ~/recover_slave.py
```

Then restart Terminal 5 to reload `lite6_traj_controller`.

## Troubleshooting

| Problem | Cause | Solution |
|---|---|---|
| Calibration stuck at 0/80 | Master topic at 5 Hz | Use `report_type:=dev` in Terminal 3 |
| Calibration fails (high std) | Robots moving during startup | Don't touch robots for first 3 seconds |
| C31 error during motion | Aggressive trajectory or real collision | Run `recover_slave.py`, restart Terminal 5 |
| `Package 'haptic_teleop' not found` | `install/setup.bash` not sourced | Run `source install/setup.bash` after build |
| `Package 'topic_tools' not found` | Missing system package | `sudo apt install ros-humble-topic-tools` |
