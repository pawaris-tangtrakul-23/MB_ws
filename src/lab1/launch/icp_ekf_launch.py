import os
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix


def generate_launch_description():
    """
    Launch file for ICP + EKF Fusion SLAM

    Pipeline:
    1. converter.py: Joint states -> /odom_raw
    2. ekf_node.py: /odom_raw + /imu -> /odom_ekf
    3. icp_ekf_node.py: /scan + /odom_ekf -> /odom_icp + /map
    4. path_publisher_node.py: /odom_* -> /path_* (trajectory visualization)
    5. ROS2 bag playback
    6. RViz2 for visualization
    """

    # Get package directories
    pkg_share = get_package_share_directory('lab1')
    pkg_prefix = get_package_prefix('lab1')

    # Executable paths (installed in lib/lab1/)
    lib_dir = os.path.join(pkg_prefix, 'lib', 'lab1')
    converter_path = os.path.join(lib_dir, "converter.py")
    ekf_path = os.path.join(lib_dir, "ekf_node.py")
    icp_ekf_path = os.path.join(lib_dir, "icp_ekf_node.py")
    path_publisher_path = os.path.join(lib_dir, "path_publisher_node.py")

    # Config files (installed in share/lab1/config/)
    rviz_config_path = os.path.join(pkg_share, "config", "icp.rviz")

    # Declare bag file argument
    bag_arg = DeclareLaunchArgument(
        "bag_file", description="Full path to the ROS 2 bag file or folder"
    )

    # 1. Start Converter Node (Joint States -> Wheel Odometry)
    converter_node = Node(
        package="",
        executable=sys.executable,
        arguments=[converter_path, "--ros-args", "-p", "use_sim_time:=true"],
        output="screen",
        name="converter_node",
        parameters=[{"use_sim_time": True}],
        prefix='gnome-terminal -- bash -c \'echo -e "\\n=== CONVERTER NODE ===\\n"; ',
    )

    converter_node = ExecuteProcess(
        cmd=[sys.executable, converter_path, "--ros-args", "-p", "use_sim_time:=true"],
        output="screen",
        shell=False,
    )

    # 2. Start EKF Node (Wheel Odom + IMU Fusion)
    ekf_node = ExecuteProcess(
        cmd=[sys.executable, ekf_path, "--ros-args", "-p", "use_sim_time:=true"],
        output="screen",
        shell=False,
    )

    # 3. Start ICP+EKF Fusion Node (LiDAR SLAM with EKF initial guess)
    icp_ekf_node = ExecuteProcess(
        cmd=[sys.executable, icp_ekf_path, "--ros-args", "-p", "use_sim_time:=true"],
        output="screen",
        shell=False,
    )

    # 4. Start Path Publisher Node (Odometry -> Path visualization)
    path_publisher_node = ExecuteProcess(
        cmd=[
            sys.executable,
            path_publisher_path,
            "--ros-args",
            "-p",
            "use_sim_time:=true",
        ],
        output="screen",
        shell=False,
    )

    # 5. Play ROS2 Bag with --clock and --loop
    bag_play_process = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            LaunchConfiguration("bag_file"),
            "--clock",
            "--rate",
            "1.0",  # Real-time playback
        ],
        output="screen",
        shell=False,
    )

    # 6. RViz2 with Sim Time and Config File
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_path],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # Delay RViz start to let other nodes initialize
    delayed_rviz = TimerAction(period=3.0, actions=[rviz_node])

    return LaunchDescription(
        [
            LogInfo(msg=["\n" + "=" * 60]),
            LogInfo(msg=["Starting ICP + EKF Fusion SLAM System"]),
            LogInfo(msg=["=" * 60 + "\n"]),
            LogInfo(msg=["Pipeline:"]),
            LogInfo(msg=["  1. Converter: /joint_states -> /odom_raw"]),
            LogInfo(msg=["  2. EKF: /odom_raw + /imu -> /odom_ekf"]),
            LogInfo(msg=["  3. ICP+EKF: /scan + /odom_ekf -> /odom_icp + /map"]),
            LogInfo(msg=["  4. Path Publisher: /odom_* -> /path_*"]),
            LogInfo(msg=["  5. Bag Playback"]),
            LogInfo(msg=["  6. RViz2 (delayed 3s)"]),
            LogInfo(msg=["=" * 60 + "\n"]),
            bag_arg,
            converter_node,
            ekf_node,
            icp_ekf_node,
            path_publisher_node,
            bag_play_process,
            delayed_rviz,
        ]
    )
