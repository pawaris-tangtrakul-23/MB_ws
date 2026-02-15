#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix


def generate_launch_description():
    # Get package directories
    pkg_share = get_package_share_directory('lab1')
    pkg_prefix = get_package_prefix('lab1')

    # Config files (installed in share/lab1/config/)
    config_file = os.path.join(pkg_share, "config", "slam_config.yaml")

    # Dataset path - portable solution with multiple fallbacks
    bag_dir = None

    # Priority 1: Environment variable
    if 'LAB1_DATASET_PATH' in os.environ:
        bag_dir = os.path.join(os.environ['LAB1_DATASET_PATH'], 'fibo_floor3_seq00')

    # Priority 2: Package share (if dataset was installed)
    if not bag_dir or not os.path.exists(bag_dir):
        share_dataset = os.path.join(pkg_share, "FRA532_LAB1_DATASET", "fibo_floor3_seq00")
        if os.path.exists(share_dataset):
            bag_dir = share_dataset

    # Priority 3: Source workspace (development mode)
    if not bag_dir or not os.path.exists(bag_dir):
        workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(pkg_share))))
        src_dataset = os.path.join(workspace_root, "src", "lab1", "FRA532_LAB1_DATASET", "fibo_floor3_seq00")
        if os.path.exists(src_dataset):
            bag_dir = src_dataset

    # Fallback default (will be overridable via launch argument)
    if not bag_dir:
        bag_dir = "/tmp/dataset_not_found"

    # Python scripts (installed in lib/lab1/)
    lib_dir = os.path.join(pkg_prefix, 'lib', 'lab1')
    converter_script = os.path.join(lib_dir, "converter_no_tf.py")
    ekf_script = os.path.join(lib_dir, "slam_ekf_node.py")

    # RViz config
    rviz_config = os.path.join(pkg_share, "config", "slam_rviz_config.rviz")
    if not os.path.exists(rviz_config):
        # Fallback if specific config not found
        rviz_config = os.path.join(pkg_share, "config", "slam.rviz")

    # Launch arguments
    declare_bag_arg = DeclareLaunchArgument(
        "bag_path", default_value=bag_dir, description="Path to rosbag directory"
    )
    declare_rate_arg = DeclareLaunchArgument(
        "playback_rate", default_value="1.0", description="Bag playback speed"
    )

    bag_path = LaunchConfiguration("bag_path")
    playback_rate = LaunchConfiguration("playback_rate")

    # 1. Converter Node - TF DISABLED so EKF can do it
    converter_node = ExecuteProcess(
        cmd=[
            "python3",
            converter_script,
            "--ros-args",
            "-p",
            "use_sim_time:=true",
            "-p",
            "publish_tf:=false",
        ],
        output="screen",
        name="converter",
    )

    # 2. EKF Node - This will publish the TF
    ekf_node = ExecuteProcess(
        cmd=[
            "python3",
            ekf_script,
            "--ros-args",
            "-p",
            "use_sim_time:=true",
            "-p",
            "use_imu:=true",  # Set to true to use IMU
        ],
        output="screen",
        name="ekf_node",
    )

    # 3. Static TF (base_link_ekf -> base_scan)
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_laser_tf",
        arguments=["0", "0", "0.1", "0", "0", "0", "base_link_ekf", "base_scan"],
        parameters=[{"use_sim_time": True}],
    )

    # 4. SLAM Toolbox
    slam_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[config_file, {"use_sim_time": True}],
    )

    # 5. RViz2
    rviz_cmd = ["rviz2", "-d", rviz_config, "--ros-args", "-p", "use_sim_time:=true"]
    rviz_node = ExecuteProcess(cmd=rviz_cmd, output="screen", name="rviz2")

    # 6. Play Bag
    play_bag = TimerAction(
        period=5.0,
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
            ekf_node,  # Added the EKF!
            TimerAction(period=1.0, actions=[static_tf_node]),
            TimerAction(period=2.0, actions=[slam_node]),
            TimerAction(period=3.0, actions=[rviz_node]),
            play_bag,
        ]
    )
