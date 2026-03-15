# TE3001B - Kinematics Challenge - Haptic Teleoperation
Dr. Luis Alberto Muñoz Ubando

**Team:** Dream Team  
| Name | Matricula | Github User |
|------|-----------|-------------|
| Hector Tovar | A00840308 | @htovarm7 |
| José Luis Domínguez Morales  | A01285873 | @JLDominguezM |
| Paola Llamas Hernandez | A01178479 | @PaolaLlh18|
| Jocelyn Anahi Velarde Barrón | A01285780 | @JocelynVelarde |


## Overview

This project implements a **bilateral haptic teleoperation** system for two **xArm Lite 6** robots using ROS 2. A human operator moves the master robot by hand, and the slave robot replicates the movements in real time. When the slave detects a collision (via joint effort or an FSR force sensor), the master locks up so the operator feels the collision.

### Hardware

| Robot | IP | Role |
|---|---|---|
| xArm Lite 6 | 192.168.1.175 | **Slave** — replicates master movements via ros2_control |
| xArm Lite 6 | 192.168.1.167 | **Master** — moved by human operator (SDK direct) |
| ESP32 + micro-ROS | USB `/dev/ttyUSB0` | FSR force sensor |

### Components

- **bilateral_teleop.py** — Main ROS 2 node (FSM + effort-based collision detection + master SDK connection)
- **recover_slave.py** — Script to clear xArm errors and reset the slave for ros2_control
- **Force_Sensor.ino** — ESP32 sketch that reads an analog force sensor and publishes `Int32` messages to `/force_sensor` via micro-ROS
- **xarm_ros2** — Full xArm ROS 2 driver stack (controllers, MoveIt config)

## Prerequisites

- **ROS 2 Humble**
- **MoveIt 2**
- **micro-ROS Agent** (`ros-humble-micro-ros-agent`)
- **xarm-python-sdk** (`pip install xarm-python-sdk`)
- **Arduino IDE** with micro-ROS library (for flashing sensor firmware)
- Two **xArm Lite 6** robots on the same network (192.168.1.x)

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
sudo apt install ros-humble-micro-ros-agent
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

## How to Run — Bilateral Teleoperation (4 terminals)

Open each terminal in order and wait for the confirmation message before proceeding.

### Terminal 1 — micro-ROS Agent (ESP32 force sensor)

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
```

Confirm: ESP32 connects and session logs appear.

### Terminal 2 — Recover slave (one-time, before each session)

```bash
cd ~/Desktop/Challenge-Kinematics/src
python3 recover_slave.py
```

Confirm: `Listo para ros2_control: mode=1, state=2`

> This clears any pending errors on the slave and puts it in servo mode for ros2_control.

### Terminal 3 — Slave MoveIt + ros2_control

```bash
cd ~/Desktop/Challenge-Kinematics && source install/setup.bash
ros2 launch xarm_moveit_config lite6_moveit_realmove.launch.py \
  robot_ip:=192.168.1.175 \
  add_realsense_d435i:=false
```

Confirm: `You can start planning now!` and `lite6_traj_controller` loaded.

### Terminal 4 — Bilateral teleoperation node

```bash
cd ~/Desktop/Challenge-Kinematics && source install/setup.bash
ros2 run haptic_teleop bilateral_teleop --ros-args \
  --params-file src/haptic_teleop/config/teleop_params.yaml
```

Confirm: `Estado -> RUNNING` followed by `Grace period terminado - maestro desbloqueado`

## Startup Sequence

The node follows this state machine:

```
HOMING → CALIBRATING → RUNNING (grace 3s) → RUNNING (active)
```

1. **HOMING** — Both robots move to HOME position. Master is locked (mode 0). Slave moves via trajectory controller.
2. **CALIBRATING** — Collects 80 samples of joint positions and efforts from both robots to compute offsets and effort baselines. **Do not touch either robot.**
3. **RUNNING (grace)** — 3-second stabilization period. Slave holds position, master remains locked.
4. **RUNNING (active)** — Master unlocks (teach mode). Operator can move it. Slave follows.

## Collision Behavior

When the slave detects an external force above the effort threshold (6.0 N.m):

1. **Slave holds** — stops following the master, maintains current position
2. **Master locks** — switches to position mode, operator cannot move it
3. **Minimum hold** — collision state is maintained for at least 2 seconds
4. **Release** — when external force drops and hold time has passed, master unlocks and teleoperation resumes

## Topics

| Topic | Type | Description |
|---|---|---|
| `/master/joint_states` | `sensor_msgs/JointState` | Master joint state (published by bilateral_teleop via SDK, ~90 Hz) |
| `/joint_states` | `sensor_msgs/JointState` | Slave joint state + efforts (~150 Hz) |
| `/force_sensor` | `std_msgs/Int32` | ESP32 ADC (rest ~4095, contact <3900) |
| `/lite6_traj_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | Trajectory commands to slave |
| `/master/reflected_torques` | `std_msgs/Float64MultiArray` | Haptic feedback torques (logged, not applied) |

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
| `perturbation_thr_deg` | 2.0 | Threshold for detecting external perturbation [deg] |

## Error Recovery

### C31 — Collision Caused Abnormal Joint Current

```bash
cd ~/Desktop/Challenge-Kinematics/src
python3 recover_slave.py
```

Then restart Terminal 3 (MoveIt + ros2_control).

### C60 — Abnormal Error Code

Same procedure as C31: run `recover_slave.py` and restart Terminal 3.

### Master stuck / won't move

The node automatically clears errors on the master when switching modes. If the master is completely unresponsive, restart Terminal 4 (bilateral_teleop).

## Troubleshooting

| Problem | Cause | Solution |
|---|---|---|
| Calibration stuck at 0/80 | No slave data received | Check Terminal 3 is running |
| Calibration fails (high std) | Robots moving during calibration | Don't touch robots, they will stabilize |
| C31/C60 error on slave | Collision or servo command rejected | Run `recover_slave.py`, restart Terminal 3 |
| `Package 'haptic_teleop' not found` | `install/setup.bash` not sourced | Run `source install/setup.bash` after build |
| Master won't unlock after collision | xArm error state | Restart Terminal 4 |
| Slave drifts without input | Joint limits clipping | Fixed — limits handled by xArm firmware |
