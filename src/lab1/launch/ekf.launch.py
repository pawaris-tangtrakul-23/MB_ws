import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix

def generate_launch_description():
    # Get package directories
    pkg_share = get_package_share_directory('lab1')
    pkg_prefix = get_package_prefix('lab1')

    # Python scripts (installed in lib/lab1/)
    lib_dir = os.path.join(pkg_prefix, 'lib', 'lab1')
    converter_script = os.path.join(lib_dir, 'converter.py')
    ekf_script = os.path.join(lib_dir, 'ekf_node.py')

    # Dataset path - portable solution with multiple fallbacks
    bag_path = None

    # Priority 1: Environment variable
    if 'LAB1_DATASET_PATH' in os.environ:
        bag_path = os.path.join(os.environ['LAB1_DATASET_PATH'], 'fibo_floor3_seq00')

    # Priority 2: Package share (if dataset was installed)
    if not bag_path or not os.path.exists(bag_path):
        share_dataset = os.path.join(pkg_share, "FRA532_LAB1_DATASET", "fibo_floor3_seq00")
        if os.path.exists(share_dataset):
            bag_path = share_dataset

    # Priority 3: Source workspace (development mode)
    if not bag_path or not os.path.exists(bag_path):
        workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(pkg_share))))
        src_dataset = os.path.join(workspace_root, "src", "lab1", "FRA532_LAB1_DATASET", "fibo_floor3_seq00")
        if os.path.exists(src_dataset):
            bag_path = src_dataset

    # Fallback default
    if not bag_path:
        bag_path = "/tmp/dataset_not_found"

    return LaunchDescription([
        # --- ACTION 1: Play Rosbag ---
        # --clock: Publishes /clock for time synchronization
        # --loop: Restarts the bag when it finishes
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag_path, '--clock', '--loop'],
            output='screen'
        ),

        # --- ACTION 2: Run Odom Converter ---
        # We run this using python3 directly since it's a standalone script
        ExecuteProcess(
            cmd=['python3', converter_script],
            output='screen',
            name='converter_node'
        ),

        # --- ACTION 3: Run EKF Node ---
        ExecuteProcess(
            cmd=['python3', ekf_script],
            output='screen',
            name='ekf_node'
        ),

        # --- ACTION 4: Open RViz2 (Optional) ---
        # Useful for visualizing /odom_raw vs /odom_ekf
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
            # If you have a saved rviz config, you can add:
            # arguments=['-d', os.path.join(base_path, 'my_config.rviz')]
        )
    ])