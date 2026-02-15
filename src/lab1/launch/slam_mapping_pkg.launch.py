#!/usr/bin/env python3
"""
SLAM Mapping Launch File - ROS2 Package Version
Uses proper package structure with get_package_share_directory
Usage: ros2 launch fra532_lab1 slam_mapping_pkg.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory (corrected package name)
    pkg_dir = get_package_share_directory('lab1')

    # Paths
    config_file = os.path.join(pkg_dir, "config", "slam_config.yaml")

    # Dataset path - portable solution with multiple fallbacks
    bag_dir = None

    # Priority 1: Environment variable
    if 'LAB1_DATASET_PATH' in os.environ:
        bag_dir = os.path.join(os.environ['LAB1_DATASET_PATH'], 'fibo_floor3_seq00')

    # Priority 2: Package share (if dataset was installed)
    if not bag_dir or not os.path.exists(bag_dir):
        share_dataset = os.path.join(pkg_dir, "FRA532_LAB1_DATASET", "fibo_floor3_seq00")
        if os.path.exists(share_dataset):
            bag_dir = share_dataset

    # Priority 3: Source workspace (development mode)
    if not bag_dir or not os.path.exists(bag_dir):
        workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(pkg_dir))))
        src_dataset = os.path.join(workspace_root, "src", "lab1", "FRA532_LAB1_DATASET", "fibo_floor3_seq00")
        if os.path.exists(src_dataset):
            bag_dir = src_dataset

    # Fallback default (will be overridable via launch argument)
    if not bag_dir:
        bag_dir = "/tmp/dataset_not_found"

    # RViz config
    rviz_config = os.path.join(pkg_dir, "config", "slam_rviz_config.rviz")
    if not os.path.exists(rviz_config):
        rviz_config = os.path.join(pkg_dir, "config", "icp.rviz")

    # Launch arguments
    declare_bag_arg = DeclareLaunchArgument(
        "bag_path", default_value=bag_dir, description="Path to rosbag directory"
    )

    declare_rate_arg = DeclareLaunchArgument(
        "playback_rate", default_value="1.0", description="Bag playback speed"
    )

    bag_path = LaunchConfiguration("bag_path")
    playback_rate = LaunchConfiguration("playback_rate")

    # Node 1: Converter (FIXED VERSION with all bug fixes!)
    converter_node = Node(
        package='lab1',
        executable='converter_no_tf',
        name='converter',
        output='screen',
        parameters=[{"use_sim_time": True}],
    )

    # Node 2: Static TF (base_link_ekf → base_scan)
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_laser_tf",
        arguments=["0", "0", "0.1", "0", "0", "0", "base_link_ekf", "base_scan"],
        parameters=[{"use_sim_time": True}],
    )

    # Node 3: SLAM Toolbox
    slam_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[config_file, {"use_sim_time": True}],
    )

    # Node 4: RViz2
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
    )

    # Process: Play rosbag
    play_bag = TimerAction(
        period=4.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "bag",
                    "play",
                    bag_path,
                    "--clock",
                    "--rate",
                    playback_rate,
                ],
                output="screen",
            )
        ],
    )

    return LaunchDescription(
        [
            declare_bag_arg,
            declare_rate_arg,
            converter_node,
            TimerAction(period=2.0, actions=[static_tf_node]),
            TimerAction(period=3.0, actions=[slam_node]),
            TimerAction(period=3.5, actions=[rviz_node]),
            play_bag,
        ]
    )
