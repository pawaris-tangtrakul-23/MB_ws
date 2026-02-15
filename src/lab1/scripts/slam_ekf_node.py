#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
import math
from tf_transformations import quaternion_from_euler, euler_from_quaternion


class EKFNode(Node):
    def __init__(self):
        super().__init__("ekf_node")

        # Declare parameters
        self.declare_parameter("use_imu", False)  # DISABLE IMU by default to test
        self.use_imu = self.get_parameter("use_imu").value

        self.x = np.zeros((3, 1))
        self.P = np.eye(3) * 0.001
        self.Q = np.diag([0.00001, 0.00001, 0.00001])
        self.R = np.array([[0.01]])

        self.initialized = False
        self.last_odom_msg = None

        self.get_logger().info(
            f"EKF initialized - IMU updates: {'ENABLED' if self.use_imu else 'DISABLED'}"
        )

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(Odometry, "/odom_raw", self.odom_callback, qos)
        self.create_subscription(Imu, "/imu", self.imu_callback, qos)
        self.pub = self.create_publisher(Odometry, "/odom_ekf", 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.last_time = None
        self.count = 0
        self.imu_count = 0
        print("=" * 60)
        print(f"DEBUG: EKF Node Started")
        print(
            f"DEBUG: IMU updates: {'ENABLED' if self.use_imu else 'DISABLED (odom-only mode)'}"
        )
        print("=" * 60)

    def odom_callback(self, msg):
        self.count += 1
        current_time = self.get_clock().now()

        # Initialize from first message
        if not self.initialized:
            q = msg.pose.pose.orientation
            (_, _, initial_theta) = euler_from_quaternion([q.x, q.y, q.z, q.w])

            self.x[0, 0] = msg.pose.pose.position.x
            self.x[1, 0] = msg.pose.pose.position.y
            self.x[2, 0] = initial_theta

            self.initialized = True
            self.last_time = current_time
            self.last_odom_msg = msg

            print(f"DEBUG: EKF initialized from first odom:")
            print(
                f"  Converter x={msg.pose.pose.position.x:.4f}, y={msg.pose.pose.position.y:.4f}, θ={initial_theta:.4f}"
            )
            print(
                f"  EKF State x={self.x[0,0]:.4f}, y={self.x[1,0]:.4f}, θ={self.x[2,0]:.4f}"
            )
            return

        if self.count % 20 == 0:
            # Compare converter vs EKF positions AND THETA
            conv_x = msg.pose.pose.position.x
            conv_y = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            (_, _, conv_theta) = euler_from_quaternion([q.x, q.y, q.z, q.w])

            ekf_x = self.x[0, 0]
            ekf_y = self.x[1, 0]
            ekf_theta = self.x[2, 0]

            # Theta difference (handle wrap-around)
            theta_diff = ekf_theta - conv_theta
            theta_diff = math.atan2(math.sin(theta_diff), math.cos(theta_diff))

            print("=" * 60)
            print(f"Position Comparison [{self.count}]:")
            print(f"  Converter: x={conv_x:.3f}, y={conv_y:.3f}, θ={conv_theta:.4f}")
            print(f"  EKF:       x={ekf_x:.3f}, y={ekf_y:.3f}, θ={ekf_theta:.4f}")
            print(
                f"  Diff:      Δx={ekf_x-conv_x:.3f}, Δy={ekf_y-conv_y:.3f}, Δθ={theta_diff:.4f}"
            )
            if abs(ekf_x - conv_x) > 0.5 or abs(ekf_y - conv_y) > 0.5:
                print("  ⚠️  WARNING: Large position difference!")
            if abs(theta_diff) > 0.1:  # > ~6 degrees
                print("  ⚠️  WARNING: Large heading difference!")
            print("=" * 60)

        if self.last_time is None:
            self.last_time = current_time
            return

        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if dt > 1.0 or dt < 0:
            return

        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z
        theta = self.x[2, 0]

        # Predict
        dx = v * math.cos(theta) * dt
        dy = v * math.sin(theta) * dt

        self.x[0, 0] += dx
        self.x[1, 0] += dy
        self.x[2, 0] += w * dt
        self.x[2, 0] = math.atan2(math.sin(self.x[2, 0]), math.cos(self.x[2, 0]))

        # Jacobian
        F = np.eye(3)
        F[0, 2] = -v * math.sin(theta) * dt
        F[1, 2] = v * math.cos(theta) * dt
        self.P = F @ self.P @ F.T + self.Q

        # Clamp covariance
        self.P[0, 0] = min(self.P[0, 0], 0.5)
        self.P[1, 1] = min(self.P[1, 1], 0.5)
        self.P[2, 2] = min(self.P[2, 2], 0.01)

        self.last_odom_msg = msg
        self.publish_ekf(current_time)

    def imu_callback(self, msg):
        if not self.initialized or not self.use_imu:
            return

        self.imu_count += 1

        q = msg.orientation
        (_, _, yaw_measured) = euler_from_quaternion([q.x, q.y, q.z, q.w])

        # Show IMU vs EKF theta comparison
        if self.imu_count % 50 == 0:
            print(
                f"IMU Update [{self.imu_count}]: IMU θ={yaw_measured:.4f}, EKF θ={self.x[2,0]:.4f}, diff={yaw_measured - self.x[2,0]:.4f}"
            )

        z = np.array([[yaw_measured]])
        H = np.array([[0, 0, 1]])
        theta_predicted = self.x[2, 0]
        y = z - theta_predicted
        y[0, 0] = math.atan2(math.sin(y[0, 0]), math.cos(y[0, 0]))

        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)

        old_theta = self.x[2, 0]
        self.x = self.x + K @ y
        self.x[2, 0] = math.atan2(math.sin(self.x[2, 0]), math.cos(self.x[2, 0]))
        self.P = (np.eye(3) - K @ H) @ self.P

        if self.imu_count % 50 == 0:
            print(
                f"  → After correction: θ={self.x[2,0]:.4f} (changed by {self.x[2,0]-old_theta:.4f})"
            )

    def publish_ekf(self, current_time):
        msg = Odometry()
        msg.header.stamp = current_time.to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link_ekf"
        msg.pose.pose.position.x = self.x[0, 0]
        msg.pose.pose.position.y = self.x[1, 0]

        q = quaternion_from_euler(0, 0, self.x[2, 0])
        (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ) = q

        cov = [0.0] * 36
        cov[0] = self.P[0, 0]
        cov[7] = self.P[1, 1]
        cov[35] = self.P[2, 2]
        msg.pose.covariance = cov

        self.pub.publish(msg)

        # Broadcast TF
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link_ekf"
        t.transform.translation.x = self.x[0, 0]
        t.transform.translation.y = self.x[1, 0]
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(EKFNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
