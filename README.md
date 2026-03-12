# LAB2: Quadrotor Control using Model Predictive Control (MPC)

**Course:** FRA532 - Mobile Robot
**Team:** Pao-Pond Hero

A ROS2 quadrotor simulation with MPC, LQR, and PID controllers. The MPC controller uses a constrained QP solver (OSQP) to track hover positions and 2D/3D trajectories in Gazebo Harmonic.

---

## Table of Contents

- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Build](#build)
- [Launch](#launch)
- [Usage](#usage)
  - [Flight Mode Activation](#flight-mode-activation)
  - [Point-to-Point Navigation (GOTO)](#point-to-point-navigation-goto)
  - [2D Trajectories](#2d-trajectories-x-z-plane)
  - [3D Trajectories](#3d-trajectories-x-y-z)
  - [Stopping](#stopping)
- [Plotting Flight Data](#plotting-flight-data)
- [Project Structure](#project-structure)
- [Documentation](#documentation)

---

## Prerequisites

- **Ubuntu 22.04** (or compatible)
- **ROS2 Humble**
- **Gazebo Harmonic** (with `ros_gz` bridge packages)

---

## Installation

### 1. Install ROS2 Packages

```bash
sudo apt update
sudo apt install -y \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-bridge \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-rviz2 \
  ros-humble-actuator-msgs \
  ros-humble-plotjuggler-ros
```

### 2. Install Python Dependencies

```bash
pip3 install osqp numpy scipy matplotlib
```

### 3. Clone the Repository

```bash
cd ~
git clone <your-repo-url> Mobile_Robot-Lab2
```

---

## Build

```bash
cd ~/Mobile_Robot-Lab2
colcon build --symlink-install
source install/setup.bash
```

> Add `source ~/Mobile_Robot-Lab2/install/setup.bash` to your `~/.bashrc` to auto-source on every terminal.

---

## Launch

### MPC Controller (default: no-wind environment)

```bash
ros2 launch quad_description mpc.launch.py
```

### With Wind Disturbance (4 m/s in -y)

```bash
ros2 launch quad_description mpc.launch.py world:=$(ros2 pkg prefix quad_description)/share/quad_description/worlds/wind.sdf
```

This launches:
- Gazebo Harmonic simulation with quadrotor
- RViz2 visualization
- ROS2-Gazebo bridge
- MPC controller node (`mpc_node.py`)

### (Optional) Real-time 3D Trajectory Plot

In a separate terminal:

```bash
ros2 run lab2 plot_path.py --ros-args -p use_sim_time:=true
```

---

## Usage

All commands are sent via `ros2 topic pub`. The typical workflow is:

### 1. Flight Mode Activation

First, activate the motors and take off to 2 m hover:

```bash
# For 2D trajectories (X-Z plane only):
ros2 topic pub --once /set_flight_mode std_msgs/String "data: '2D'"

# For 3D trajectories (X-Y-Z):
ros2 topic pub --once /set_flight_mode std_msgs/String "data: '3D'"
```

Wait for the drone to stabilize at 2 m altitude before sending trajectory commands.

### 2. Point-to-Point Navigation (GOTO)

Send a target position (requires `2D` or `3D` mode active):

```bash
ros2 topic pub --once /set_target_xyz geometry_msgs/Vector3 "{x: 2.0, y: 1.0, z: 3.0}"
```

### 3. 2D Trajectories (X-Z plane)

Activate `2D` mode first, then:

```bash
# Straight line forward (+3.0 m/s in +x)
ros2 topic pub --once /set_flight_mode std_msgs/String "data: '2D_STRAIGHT_F'"

# Straight line backward (-3.0 m/s in -x)
ros2 topic pub --once /set_flight_mode std_msgs/String "data: '2D_STRAIGHT_B'"

# Sine wave (0.3 m/s forward, +/-1.0 m altitude oscillation)
ros2 topic pub --once /set_flight_mode std_msgs/String "data: '2D_SINE'"

# Ramp/triangle wave (0.8 m/s forward, +/-1.5 m amplitude, 4s period)
ros2 topic pub --once /set_flight_mode std_msgs/String "data: '2D_RAMP_WAVE'"
```

### 4. 3D Trajectories (X-Y-Z)

Activate `3D` mode first, then:

```bash
# Helix spiral (radius 2m, 0.5 rad/s, +0.1 m/s climb)
ros2 topic pub --once /set_flight_mode std_msgs/String "data: '3D_HELIX'"

# 3D straight line (X +1.0, Y +0.5, Z +0.2 m/s)
ros2 topic pub --once /set_flight_mode std_msgs/String "data: '3D_STRAIGHT'"

# Figure-8 pattern (+/-2m XY, +/-0.5m Z, 0.3 rad/s)
ros2 topic pub --once /set_flight_mode std_msgs/String "data: '3D_FIGURE8'"
```

### 5. Stopping

Return to hover or shut down motors:

```bash
# Return to hover (keeps motors on)
ros2 topic pub --once /set_flight_mode std_msgs/String "data: '3D'"

# Kill motors (drone falls)
ros2 topic pub --once /set_flight_mode std_msgs/String "data: 'IDLE'"
```

---

## Example Command Sequence

```bash
# Terminal 1: Launch simulation
ros2 launch quad_description mpc.launch.py

# Terminal 2: Send commands
ros2 topic pub --once /set_flight_mode std_msgs/String "data: '3D'"
# ... wait ~5s for takeoff ...

ros2 topic pub --once /set_target_xyz geometry_msgs/Vector3 "{x: 2.0, y: 1.0, z: 3.0}"
# ... wait for drone to reach target ...

ros2 topic pub --once /set_flight_mode std_msgs/String "data: '3D_HELIX'"
# ... watch helix trajectory ...

ros2 topic pub --once /set_flight_mode std_msgs/String "data: '3D'"
# ... returns to hover ...

ros2 topic pub --once /set_flight_mode std_msgs/String "data: 'IDLE'"
# ... motors off ...
```

---

## Plotting Flight Data

`plot_path.py` automatically records CSV files to `~/Mobile_Robot-Lab2/docs/` during trajectory flights. To generate a 6-panel dashboard from the CSV:

```bash
# Plot the most recent flight data
python3 ~/Mobile_Robot-Lab2/docs/plot_csv.py

# Plot a specific CSV file
python3 ~/Mobile_Robot-Lab2/docs/plot_csv.py docs/flight_20260312_153651_3D_HELIX.csv
```

Each plot includes: Position vs Setpoint, Position Error, Velocity, Attitude (RPY), 3D Path, and Summary Statistics. A PNG is saved alongside the CSV.

---

## Project Structure

```
Mobile_Robot-Lab2/
├── src/
│   ├── lab2/                              # Control package
│   │   ├── scripts/
│   │   │   ├── mpc_node.py               # MPC controller (OSQP)
│   │   │   ├── lqr_node.py               # LQR controller
│   │   │   ├── pid_node.py               # PID controller
│   │   │   └── plot_path.py              # Trajectory plotter + CSV logger
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   └── fra532-lab2-control-pao-pond_hero/
│       └── quad_description/              # Robot description
│           ├── launch/
│           │   ├── mpc.launch.py          # MPC simulation launch
│           │   ├── sim.launch.py          # PID/LQR simulation launch
│           │   └── rsp.launch.py          # Robot state publisher
│           ├── urdf/                      # Quadrotor URDF/Xacro
│           ├── worlds/                    # Gazebo worlds (empty, wind)
│           ├── config/                    # Bridge & plot configs
│           ├── rviz/                      # RViz config
│           └── meshes/                    # 3D robot meshes
├── docs/
│   ├── MPC_Documentation.md               # Full MPC technical documentation
│   ├── plot_csv.py                        # Flight data visualization script
│   └── flight_*.csv / flight_*.png        # Recorded flight data & plots
└── README.md
```

---

## Documentation

For the full technical write-up (dynamics, linearization, MPC formulation, QP constraints, tuning, results), see:

[docs/MPC_Documentation.md](docs/MPC_Documentation.md)
