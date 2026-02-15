#!/bin/bash

# SLAM Mapping Launch Script with Proper Time Synchronization
# This ensures all nodes use simulation time from the rosbag

cd "$(dirname "$0")"

echo "======================================================"
echo "SLAM Mapping - Starting all nodes with use_sim_time"
echo "======================================================"
echo ""

# Kill any existing processes
pkill -f converter_no_tf.py
pkill -f ekf_node.py
pkill -f slam_toolbox
pkill -f static_transform_publisher
sleep 1

# Set ROS domain
export ROS_DOMAIN_ID=0

# Bag path
BAG_PATH="$PWD/FRA532_LAB1_DATASET/fibo_floor3_seq00"
CONFIG_FILE="$PWD/slam_config.yaml"

echo "[1/6] Starting converter..."
python3 converter_no_tf.py --ros-args -p use_sim_time:=true &
CONVERTER_PID=$!
sleep 2

echo "[2/6] Starting EKF..."
python3 ekf_node.py --ros-args -p use_sim_time:=true &
EKF_PID=$!
sleep 2

echo "[3/6] Starting static TF (base_link_ekf → base_scan)..."
ros2 run tf2_ros static_transform_publisher \
  0 0 0.1 0 0 0 base_link_ekf base_scan \
  --ros-args -p use_sim_time:=true &
TF_PID=$!
sleep 1

echo "[4/6] Starting SLAM Toolbox..."
ros2 run slam_toolbox async_slam_toolbox_node \
  --ros-args \
  --params-file "$CONFIG_FILE" \
  -p use_sim_time:=true &
SLAM_PID=$!
sleep 3

echo "[5/6] Starting RViz2..."
if [ -f "$PWD/slam_rviz_config.rviz" ]; then
    rviz2 -d "$PWD/slam_rviz_config.rviz" --ros-args -p use_sim_time:=true &
elif [ -f "$PWD/slam.rviz" ]; then
    rviz2 -d "$PWD/slam.rviz" --ros-args -p use_sim_time:=true &
else
    rviz2 --ros-args -p use_sim_time:=true &
fi
RVIZ_PID=$!
sleep 2

echo "[6/6] Playing rosbag with --clock..."
echo "Bag: $BAG_PATH"
echo ""
echo "======================================================"
echo "ALL NODES STARTED - use_sim_time enabled"
echo "======================================================"
echo ""
echo "Watch RViz for map building!"
echo "Press Ctrl+C to stop all processes"
echo ""

ros2 bag play "$BAG_PATH" --clock --rate 1.0

echo ""
echo "Bag finished. Keeping nodes running..."
echo "Press Ctrl+C to stop all nodes"
echo ""

# Wait for Ctrl+C
trap "echo 'Stopping all processes...'; kill $CONVERTER_PID $EKF_PID $TF_PID $SLAM_PID $RVIZ_PID 2>/dev/null; exit" INT TERM

wait
