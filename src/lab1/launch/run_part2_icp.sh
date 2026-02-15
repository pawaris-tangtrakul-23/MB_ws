#!/bin/bash
# Quick launcher for Part 2: ICP Odometry Refinement

cd "$(dirname "$0")/../.."

echo "======================================================"
echo "FRA532 LAB1 - Part 2: ICP Odometry Refinement"
echo "======================================================"
echo ""

# Kill any existing processes
pkill -f converter
pkill -f ekf_node
pkill -f icp_ekf
pkill -f rviz2
sleep 1

export ROS_DOMAIN_ID=0
BAG_PATH="$PWD/FRA532_LAB1_DATASET/fibo_floor3_seq00"

echo "Starting nodes..."
ros2 launch src/launch/icp_ekf_launch.py bag_path:="$BAG_PATH"
