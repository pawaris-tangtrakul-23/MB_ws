#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
import csv
import os
import math
from datetime import datetime


def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = max(-1.0, min(1.0, t2))
    pitch_y = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return roll_x, pitch_y, yaw_z


class TrajectoryPlotter(Node):
    def __init__(self):
        super().__init__('trajectory_plotter')

        # Subscribers
        self.sub_curr_xyz = self.create_subscription(Vector3, '/debug/current_xyz', self.curr_xyz_cb, 10)
        self.sub_tgt_xyz = self.create_subscription(Vector3, '/debug/target_xyz', self.tgt_xyz_cb, 10)
        self.sub_curr_rpy = self.create_subscription(Vector3, '/debug/current_rpy', self.curr_rpy_cb, 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.sub_mode = self.create_subscription(String, '/set_flight_mode', self.mode_cb, 10)
        self.sub_target_cmd = self.create_subscription(Vector3, '/set_target_xyz', self.target_cmd_cb, 10)

        self.lock = threading.Lock()

        # Plot history
        self.curr_xyz_history = []
        self.tgt_xyz_history = []

        # Latest state for CSV
        self.curr_pos = (0.0, 0.0, 0.0)
        self.curr_rpy = (0.0, 0.0, 0.0)
        self.curr_vel = (0.0, 0.0, 0.0)
        self.tgt_pos = (0.0, 0.0, 0.0)

        # CSV recording state
        self.recording = False
        self.csv_rows = []
        self.record_start_time = None
        self.flight_mode = "HOVER"  # current flight mode

        # Reached detection (for GOTO_XYZ)
        self.reach_threshold = 0.15  # position error to consider "reached"
        self.reached_time = None
        self.reach_delay = 3.0  # seconds after reaching before stopping

        # Environment tag — included in CSV filenames to distinguish conditions
        # e.g. "nowind", "wind"
        self.declare_parameter('env_tag', 'nowind')
        self.env_tag = self.get_parameter('env_tag').get_parameter_value().string_value

        # CSV output directory — find workspace root from this script's location
        # Script lives at <ws>/src/lab2/scripts/plot_path_mpc.py or installed to <ws>/install/...
        # Use ROS parameter so users can override, default to ~/flight_data
        self.declare_parameter('csv_dir', '')
        csv_dir_param = self.get_parameter('csv_dir').get_parameter_value().string_value
        if csv_dir_param:
            self.csv_dir = os.path.expanduser(csv_dir_param)
        else:
            # Auto-detect: walk up from script location to find workspace docs/
            script_dir = os.path.dirname(os.path.abspath(__file__))
            ws_root = script_dir
            for _ in range(5):
                candidate = os.path.join(ws_root, 'docs')
                if os.path.isdir(candidate):
                    self.csv_dir = candidate
                    break
                ws_root = os.path.dirname(ws_root)
            else:
                self.csv_dir = os.path.expanduser('~/flight_data')
        os.makedirs(self.csv_dir, exist_ok=True)
        self.get_logger().info(f'CSV output directory: {self.csv_dir}')

        # Timer to sample data for CSV at 10 Hz
        self.csv_timer = self.create_timer(0.1, self.csv_sample_cb)

    def curr_xyz_cb(self, msg):
        with self.lock:
            self.curr_pos = (msg.x, msg.y, msg.z)
            self.curr_xyz_history.append(self.curr_pos)

    def tgt_xyz_cb(self, msg):
        with self.lock:
            self.tgt_pos = (msg.x, msg.y, msg.z)
            self.tgt_xyz_history.append(self.tgt_pos)

    # Modes that are just hover/setup — don't record these
    NON_TRAJ_MODES = {"IDLE", "2D", "3D", "HOVER"}

    def mode_cb(self, msg):
        """Detect flight mode changes from /set_flight_mode."""
        with self.lock:
            new_mode = msg.data.upper()
            if new_mode == self.flight_mode:
                return

            old_mode = self.flight_mode
            self.flight_mode = new_mode

            # Stop previous recording if switching modes
            if self.recording:
                self._stop_recording()

            # Only start recording for actual trajectory modes
            if new_mode not in self.NON_TRAJ_MODES:
                self._start_recording_mode(new_mode)
            self.get_logger().info(f'Flight mode: {old_mode} -> {new_mode}')

    def target_cmd_cb(self, msg):
        """Detect GOTO_XYZ commands from /set_target_xyz."""
        with self.lock:
            # Don't record if in IDLE (motors off)
            if self.flight_mode == "IDLE":
                return
            self.flight_mode = "GOTO_XYZ"
            if self.recording:
                self._stop_recording()
            target = (msg.x, msg.y, msg.z)
            self._start_recording_goto(target)

    def curr_rpy_cb(self, msg):
        with self.lock:
            self.curr_rpy = (msg.x, msg.y, msg.z)

    def odom_cb(self, msg):
        with self.lock:
            v = msg.twist.twist.linear
            # Odom twist may be body-frame; compute world-frame velocity from quaternion
            q = msg.pose.pose.orientation
            roll, pitch, yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
            # Rotate body velocity to world frame
            cos_y = math.cos(yaw)
            sin_y = math.sin(yaw)
            cos_p = math.cos(pitch)
            sin_p = math.sin(pitch)
            cos_r = math.cos(roll)
            sin_r = math.sin(roll)
            # Full rotation (simplified for small angles, but exact here)
            vx_w = (cos_y*cos_p)*v.x + (cos_y*sin_p*sin_r - sin_y*cos_r)*v.y + (cos_y*sin_p*cos_r + sin_y*sin_r)*v.z
            vy_w = (sin_y*cos_p)*v.x + (sin_y*sin_p*sin_r + cos_y*cos_r)*v.y + (sin_y*sin_p*cos_r - cos_y*sin_r)*v.z
            vz_w = (-sin_p)*v.x + (cos_p*sin_r)*v.y + (cos_p*cos_r)*v.z
            self.curr_vel = (vx_w, vy_w, vz_w)

    def _start_recording_goto(self, target):
        """Start CSV recording for GOTO_XYZ (called with lock held)."""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        fname = f"flight_{timestamp}_GOTO_{target[0]:.1f}_{target[1]:.1f}_{target[2]:.1f}_{self.env_tag}.csv"
        self._begin_recording(fname)

    def _start_recording_mode(self, mode):
        """Start CSV recording for trajectory modes (called with lock held)."""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        fname = f"flight_{timestamp}_{mode}_{self.env_tag}.csv"
        self._begin_recording(fname)

    def _begin_recording(self, fname):
        """Common recording setup (called with lock held)."""
        self.csv_path = os.path.join(self.csv_dir, fname)
        self.csv_rows = []
        self.recording = True
        self.reached_time = None
        self.record_start_time = self.get_clock().now()
        self.get_logger().info(f'CSV recording STARTED -> {fname}')

    def _stop_recording(self):
        """Write buffered rows to CSV file and stop recording (called with lock held)."""
        if not self.csv_rows:
            self.recording = False
            return

        with open(self.csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'time_s',
                'curr_x', 'curr_y', 'curr_z',
                'tgt_x', 'tgt_y', 'tgt_z',
                'vel_x', 'vel_y', 'vel_z',
                'roll_rad', 'pitch_rad', 'yaw_rad',
            ])
            writer.writerows(self.csv_rows)

        self.get_logger().info(f'CSV recording ENDED -> {os.path.basename(self.csv_path)} ({len(self.csv_rows)} samples)')
        self.csv_rows = []
        self.recording = False

    def csv_sample_cb(self):
        """Timer callback at 10 Hz to sample data into CSV buffer."""
        with self.lock:
            if not self.recording:
                return

            now = self.get_clock().now()
            t = (now - self.record_start_time).nanoseconds / 1e9

            self.csv_rows.append([
                f'{t:.3f}',
                f'{self.curr_pos[0]:.4f}', f'{self.curr_pos[1]:.4f}', f'{self.curr_pos[2]:.4f}',
                f'{self.tgt_pos[0]:.4f}', f'{self.tgt_pos[1]:.4f}', f'{self.tgt_pos[2]:.4f}',
                f'{self.curr_vel[0]:.4f}', f'{self.curr_vel[1]:.4f}', f'{self.curr_vel[2]:.4f}',
                f'{self.curr_rpy[0]:.4f}', f'{self.curr_rpy[1]:.4f}', f'{self.curr_rpy[2]:.4f}',
            ])

            # For GOTO_XYZ: stop recording 3s after reaching setpoint
            # For trajectory modes: recording stops when mode changes (via mode_cb)
            if self.flight_mode == "GOTO_XYZ":
                dx = self.curr_pos[0] - self.tgt_pos[0]
                dy = self.curr_pos[1] - self.tgt_pos[1]
                dz = self.curr_pos[2] - self.tgt_pos[2]
                dist = math.sqrt(dx*dx + dy*dy + dz*dz)

                if dist < self.reach_threshold:
                    if self.reached_time is None:
                        self.reached_time = now
                        self.get_logger().info(f'Setpoint reached (err={dist:.3f}m), recording for {self.reach_delay}s more...')
                    else:
                        elapsed = (now - self.reached_time).nanoseconds / 1e9
                        if elapsed >= self.reach_delay:
                            self._stop_recording()
                else:
                    self.reached_time = None

    def get_snapshot(self):
        with self.lock:
            return list(self.curr_xyz_history), list(self.tgt_xyz_history)


# Background thread for ROS 2 spin
def ros_spin_thread(node):
    rclpy.spin(node)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlotter()

    thread = threading.Thread(target=ros_spin_thread, args=(node,), daemon=True)
    thread.start()

    # --- Matplotlib ---
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    def update_plot(frame):
        ax.cla()

        ax.set_xlabel('X (North)')
        ax.set_ylabel('Y (West)')
        ax.set_zlabel('Z (Up)')
        ax.set_title('UAV 3D Trajectory (Actual vs Target)')

        curr_hist, tgt_hist = node.get_snapshot()

        if tgt_hist:
            tx, ty, tz = zip(*tgt_hist)
            ax.plot(tx, ty, tz, 'g--', label='Target Path', alpha=0.6)
            ax.scatter(tx[-1], ty[-1], tz[-1], color='green', marker='x', s=100, label='Current Target')

        if curr_hist:
            cx, cy, cz = zip(*curr_hist)
            ax.plot(cx, cy, cz, 'b-', label='Actual Path', linewidth=1.5)
            ax.scatter(cx[-1], cy[-1], cz[-1], color='blue', marker='o', s=50, label='Current Position')

        if tgt_hist or curr_hist:
            ax.legend()

    ani = FuncAnimation(fig, update_plot, interval=200, cache_frame_data=False)

    plt.show()

    # Finalize any in-progress recording on exit
    with node.lock:
        if node.recording:
            node._stop_recording()

    node.destroy_node()
    rclpy.shutdown()
    thread.join(timeout=1.0)


if __name__ == '__main__':
    main()
