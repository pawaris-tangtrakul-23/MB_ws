# Lab1 - ROS2 SLAM and Localization Package

A comprehensive ROS2 package for robot localization and SLAM using wheel odometry, IMU sensor fusion (EKF), ICP-based scan matching, and SLAM Toolbox integration.

## 🎯 Features

- **Wheel Odometry Conversion**: Converts joint states to odometry with configurable robot parameters
- **Extended Kalman Filter (EKF)**: Fuses wheel odometry and IMU data for improved pose estimation
- **ICP-based SLAM**: Iterative Closest Point scan matching with EKF initialization
- **SLAM Toolbox Integration**: Graph-based SLAM with loop closure detection
- **Path Visualization**: Real-time trajectory visualization for multiple odometry sources
- **Fully Portable**: Works on any computer with ROS2 Humble (no hardcoded paths!)

## 📋 Prerequisites

### Required
- **ROS2 Humble** (or compatible)
- **Python 3.8+**
- **C++14** compiler
- **CMake 3.5+**

### Python Dependencies
```bash
pip3 install scipy numpy
```

### ROS2 Packages
```bash
sudo apt install ros-humble-slam-toolbox ros-humble-tf-transformations
```

## 🚀 Installation

### 1. Create Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 2. Clone/Copy Package

```bash
# Option A: Clone from repository
git clone <your-repo-url> lab1

# Option B: Copy existing package
cp -r /path/to/lab1 .
```

### 3. Install Dataset

Choose one of these methods:

#### Method A: Environment Variable (Recommended)
```bash
# Copy dataset to a permanent location
mkdir -p ~/datasets
cp -r /path/to/FRA532_LAB1_DATASET ~/datasets/

# Set environment variable (add to ~/.bashrc for permanence)
export LAB1_DATASET_PATH="$HOME/datasets/FRA532_LAB1_DATASET"
echo 'export LAB1_DATASET_PATH="$HOME/datasets/FRA532_LAB1_DATASET"' >> ~/.bashrc
```

#### Method B: In Package Source
```bash
cp -r /path/to/FRA532_LAB1_DATASET ~/ros2_ws/src/lab1/
```

### 4. Build Package

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select lab1
```

### 5. Source Workspace

```bash
source install/setup.bash
```

## 📦 Package Contents

### Nodes

| Node | Description | Input Topics | Output Topics |
|------|-------------|--------------|---------------|
| `converter.py` | Wheel odometry converter | `/joint_states` | `/odom_raw`, `/path_raw` |
| `ekf_node.py` | EKF sensor fusion | `/odom_raw`, `/imu` | `/odom_ekf` |
| `slam_ekf_node.py` | SLAM-optimized EKF | `/odom_raw`, `/imu` | `/odom_ekf` |
| `icp_ekf_node.py` | ICP+EKF SLAM | `/scan`, `/odom_ekf` | `/odom_icp`, `/map` |
| `path_publisher_node.py` | Path visualization | `/odom_*` | `/path_*` |

### Launch Files

| Launch File | Description | Required Args |
|-------------|-------------|---------------|
| `ekf.launch.py` | Basic EKF localization | None (auto-finds dataset) |
| `slam_mapping.launch.py` | SLAM with SLAM Toolbox | None (auto-finds dataset) |
| `slam_mapping_pkg.launch.py` | SLAM (package structure) | None (auto-finds dataset) |
| `icp_ekf_launch.py` | ICP+EKF fusion SLAM | `bag_file` |

### Configuration

| File | Purpose |
|------|---------|
| `config/slam_config.yaml` | SLAM Toolbox parameters |
| `config/icp.rviz` | RViz config for ICP SLAM |
| `config/slam_rviz_config.rviz` | RViz config for SLAM Toolbox |

## 🎮 Usage

### Basic EKF Localization

```bash
ros2 launch lab1 ekf.launch.py
```

Runs:
- Bag playback (with `/clock`)
- Wheel odometry converter
- EKF node (fusing wheel odometry + IMU)
- RViz visualization

### SLAM Mapping

```bash
ros2 launch lab1 slam_mapping.launch.py
```

Runs complete SLAM pipeline:
- Bag playback
- Converter (no TF publishing)
- EKF node
- SLAM Toolbox
- RViz

### SLAM with Package Structure

```bash
ros2 launch lab1 slam_mapping_pkg.launch.py
```

Uses installed executables and proper package structure.

### ICP + EKF Fusion SLAM

```bash
ros2 launch lab1 icp_ekf_launch.py bag_file:=/path/to/bag_folder
```

Advanced SLAM using:
- EKF for initial pose guess
- ICP for scan matching refinement
- Occupancy grid mapping

### Custom Dataset Path

```bash
# Override default dataset location
ros2 launch lab1 ekf.launch.py bag_path:=/custom/path/to/bag
```

## 🔧 Configuration

### Robot Parameters

Edit in `scripts/converter.py`:
```python
self.wheel_radius = 0.033      # meters
self.wheel_separation = 0.160  # meters
```

### EKF Parameters

Edit in `scripts/ekf_node.py`:
```python
self.Q = np.diag([0.00001, 0.00001, 0.00001])  # Process noise
self.R = np.array([[0.01]])                     # Measurement noise
```

### SLAM Parameters

Edit `config/slam_config.yaml`:
```yaml
max_laser_range: 4.0           # Maximum LiDAR range
resolution: 0.04               # Map resolution
minimum_travel_distance: 0.3   # Minimum distance between scans
```

### ICP Parameters

Edit in `scripts/icp_ekf_node.py`:
```python
ICP_MAX_ITERATIONS = 50
ICP_TOLERANCE = 0.0001
ICP_OUTLIER_THRESHOLD = 0.5    # meters
```

## 🛠️ Troubleshooting

### Dataset Not Found

**Problem**: Launch fails with "dataset_not_found"

**Solutions**:
```bash
# Check environment variable
echo $LAB1_DATASET_PATH

# Set it if not set
export LAB1_DATASET_PATH="/path/to/FRA532_LAB1_DATASET"

# Or pass explicitly
ros2 launch lab1 ekf.launch.py bag_path:=/explicit/path/to/bag
```

### Package Not Found

**Problem**: `ros2 launch lab1 ...` fails

**Solutions**:
```bash
# Source workspace
source ~/ros2_ws/install/setup.bash

# Verify package exists
ros2 pkg list | grep lab1

# Rebuild if needed
cd ~/ros2_ws
colcon build --packages-select lab1
source install/setup.bash
```

### Transform Errors

**Problem**: "Could not transform from base_link to map"

**Solutions**:
- Check that all nodes are running: `ros2 node list`
- Verify TF tree: `ros2 run tf2_tools view_frames`
- Check base_frame in `slam_config.yaml` matches your converter output

### IMU Not Working

**Problem**: EKF not using IMU data

**Solutions**:
```bash
# Check IMU topic exists
ros2 topic list | grep imu

# Check IMU messages
ros2 topic echo /imu

# Enable IMU in EKF
ros2 launch lab1 ekf.launch.py use_imu:=true
```

### Build Errors

**Problem**: CMake or compilation errors

**Solutions**:
```bash
# Clean build
cd ~/ros2_ws
rm -rf build install log
colcon build --packages-select lab1

# Check dependencies
rosdep install --from-paths src --ignore-src -r -y
```

## 📁 File Structure

```
lab1/
├── CMakeLists.txt              # Build configuration
├── package.xml                 # Package metadata
├── README.md                   # This file
├── PORTABLE_SETUP.md           # Portability guide
│
├── config/                     # Configuration files
│   ├── slam_config.yaml        # SLAM Toolbox config
│   ├── icp.rviz                # RViz config for ICP
│   ├── slam_rviz_config.rviz   # RViz config for SLAM
│   └── def.rviz                # Default RViz config
│
├── launch/                     # Launch files
│   ├── ekf.launch.py           # EKF localization
│   ├── slam_mapping.launch.py  # SLAM mapping
│   ├── slam_mapping_pkg.launch.py
│   ├── icp_ekf_launch.py       # ICP+EKF SLAM
│   └── *.sh                    # Shell scripts
│
├── scripts/                    # Python nodes
│   ├── converter.py            # Wheel odometry
│   ├── converter_no_tf.py      # Converter without TF
│   ├── ekf_node.py             # EKF fusion
│   ├── slam_ekf_node.py        # SLAM-optimized EKF
│   ├── icp_ekf_node.py         # ICP+EKF SLAM
│   ├── path_publisher_node.py  # Path visualization
│   └── find_dataset.py         # Dataset path helper
│
├── src/                        # C++ sources
│   └── cpp_node.cpp
│
├── include/lab1/               # C++ headers
│   └── cpp_header.hpp
│
├── lab1/                       # Python package
│   ├── __init__.py
│   └── dummy_module.py
│
└── FRA532_LAB1_DATASET/        # Dataset (optional)
    ├── fibo_floor3_seq00/
    ├── fibo_floor3_seq01/
    └── fibo_floor3_seq02/
```

## 🔬 Technical Details

### Odometry Pipeline

```
Joint States → Differential Drive Kinematics → Wheel Odometry
  (/joint_states)                               (/odom_raw)
                                                     ↓
                                                EKF Fusion ← IMU Data
                                                              (/imu)
                                                     ↓
                                              Fused Odometry
                                                (/odom_ekf)
```

### SLAM Pipeline

```
LiDAR Scans ──────────┐
  (/scan)             │
                      ├──→ SLAM Toolbox → Map + Pose
EKF Odometry ─────────┘      (Graph SLAM)   (/map, TF: map→odom)
  (/odom_ekf)
```

### ICP+EKF Pipeline

```
Scan(t) + Scan(t-1) + EKF Motion Estimate → ICP Matching → Refined Pose
                                              ↓
                                         Occupancy Grid
                                            (/map)
```

### Frame Hierarchy

```
map (SLAM Toolbox publishes map→odom)
 └── odom (EKF publishes odom→base_link_ekf)
      └── base_link_ekf (static TF)
           └── base_scan (LiDAR frame)
```

## 🌐 Portability

This package is designed to work on **any computer** with ROS2:

✅ No hardcoded paths
✅ Auto-detects dataset location
✅ Workspace-independent
✅ Cross-platform compatible

See [PORTABLE_SETUP.md](PORTABLE_SETUP.md) for detailed portability information.

## 📝 Example Workflows

### Record Your Own Bag

```bash
ros2 bag record /joint_states /imu /scan -o my_dataset
```

### Run SLAM on Custom Bag

```bash
export LAB1_DATASET_PATH="$HOME/my_custom_dataset"
ros2 launch lab1 slam_mapping.launch.py
```

### Compare Odometry Sources

```bash
# Launch path publisher to visualize all sources
ros2 run lab1 path_publisher_node.py

# View in RViz (add Path displays for /path_raw, /path_ekf, /path_icp)
rviz2
```

### Save Map

After SLAM:
```bash
# Map is auto-saved to /tmp/slam_toolbox_map.*
# Copy to permanent location
cp /tmp/slam_toolbox_map.* ~/maps/
```

## 🤝 Contributing

Contributions are welcome! Please:
1. Follow ROS2 coding standards
2. Test on fresh workspace before submitting
3. Update documentation for new features
4. Ensure portability (no hardcoded paths!)

## 📄 License

[Specify your license here]

## 👥 Authors

[Add author information]

## 🙏 Acknowledgments

- SLAM Toolbox by Steve Macenski
- ROS2 community
- FRA532 Lab dataset contributors

## 📞 Support

For issues and questions:
- Check [Troubleshooting](#-troubleshooting) section
- See [PORTABLE_SETUP.md](PORTABLE_SETUP.md)
- Open an issue on GitHub (if applicable)

---

**Built with ROS2 Humble** | **Fully Portable** | **Production Ready**
