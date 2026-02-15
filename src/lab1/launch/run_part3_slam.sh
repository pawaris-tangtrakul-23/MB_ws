#!/bin/bash
# Quick launcher for Part 3: Full SLAM (Fixed version)
# This is the RECOMMENDED script with all bug fixes applied!

cd "$(dirname "$0")/../.."

echo "======================================================"
echo "FRA532 LAB1 - Part 3: Full SLAM (FIXED VERSION)"
echo "======================================================"
echo ""
echo "✅ All critical bugs fixed:"
echo "  ✓ dt calculation bug"
echo "  ✓ Timestamp synchronization"
echo "  ✓ Backdated TF for early scans"
echo ""

# Kill any existing processes
pkill -f converter
pkill -f ekf_node
pkill -f slam_toolbox
pkill -f static_transform_publisher
pkill -f rviz2
sleep 1

export ROS_DOMAIN_ID=0

# Allow sequence selection
SEQUENCE="${1:-seq00}"
BAG_PATH="$PWD/FRA532_LAB1_DATASET/fibo_floor3_$SEQUENCE"
CONFIG_FILE="$PWD/config/slam_config.yaml"

if [ ! -d "$BAG_PATH" ]; then
    echo "❌ Error: Bag directory not found: $BAG_PATH"
    echo ""
    echo "Usage: $0 [seq00|seq01|seq02]"
    echo "  seq00 - Empty hallway (default)"
    echo "  seq01 - Sharp turns"
    echo "  seq02 - Smooth motion"
    exit 1
fi

echo "Using sequence: $SEQUENCE"
echo "Bag path: $BAG_PATH"
echo ""

echo "[1/5] Starting converter (with SLAM fixes)..."
python3 src/nodes/converter_no_tf.py --ros-args -p use_sim_time:=true &
CONVERTER_PID=$!
sleep 2

echo "[2/5] Starting static TF (base_link_ekf → base_scan)..."
ros2 run tf2_ros static_transform_publisher \
  0 0 0.1 0 0 0 base_link_ekf base_scan \
  --ros-args -p use_sim_time:=true &
TF_PID=$!
sleep 1

echo "[3/5] Starting SLAM Toolbox..."
ros2 run slam_toolbox async_slam_toolbox_node \
  --ros-args \
  --params-file "$CONFIG_FILE" \
  -p use_sim_time:=true &
SLAM_PID=$!
sleep 3

echo "[4/5] Starting RViz2..."
if [ -f "$PWD/config/slam_rviz_config.rviz" ]; then
    rviz2 -d "$PWD/config/slam_rviz_config.rviz" --ros-args -p use_sim_time:=true &
else
    rviz2 --ros-args -p use_sim_time:=true &
fi
RVIZ_PID=$!
sleep 2

echo "[5/5] Playing rosbag with --clock..."
echo ""
echo "======================================================"
echo "ALL NODES STARTED - SLAM MAPPING IN PROGRESS"
echo "======================================================"
echo ""
echo "Watch RViz for map building!"
echo "Press Ctrl+C to stop all processes"
echo ""

ros2 bag play "$BAG_PATH" --clock --rate 1.0

echo ""
echo "Bag finished. Keeping nodes running for map inspection..."
echo "Press Ctrl+C to stop all nodes"
echo ""

# Wait for Ctrl+C
trap "echo 'Stopping all processes...'; kill $CONVERTER_PID $TF_PID $SLAM_PID $RVIZ_PID 2>/dev/null; exit" INT TERM

wait
