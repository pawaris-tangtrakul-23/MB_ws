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

        self.x = np.zeros((3, 1))
        self.P = np.eye(3) * 0.001  # Very low initial uncertainty
        self.Q = np.diag([0.00001, 0.00001, 0.00001])  # Extremely low process noise
        self.R = np.array([[0.01]])  # Low measurement noise

        self.get_logger().info("EKF initialized with covariance clamping enabled")

        # QoS MUST MATCH CONVERTER (Best Effort)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(Odometry, "/odom_raw", self.odom_callback, qos)
        self.create_subscription(Imu, "/imu", self.imu_callback, qos)
        self.pub = self.create_publisher(Odometry, "/odom_ekf", 10)

        # Add TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        self.last_time = None
        self.count = 0
        print("DEBUG: EKF Node Started. Waiting for /odom_raw...")

    def odom_callback(self, msg):
        self.count += 1
        current_time = self.get_clock().now()

        if self.count % 10 == 0:
            v_in = msg.twist.twist.linear.x
            print(f"DEBUG: EKF received /odom_raw. v={v_in:.4f}")

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
        self.x[0, 0] += v * math.cos(theta) * dt
        self.x[1, 0] += v * math.sin(theta) * dt
        self.x[2, 0] += w * dt
        self.x[2, 0] = math.atan2(math.sin(self.x[2, 0]), math.cos(self.x[2, 0]))

        # Jacobian
        F = np.eye(3)
        F[0, 2] = -v * math.sin(theta) * dt
        F[1, 2] = v * math.cos(theta) * dt
        self.P = F @ self.P @ F.T + self.Q

        # CRITICAL: Clamp covariance to prevent unbounded growth
        # This prevents filter divergence in long runs
        MAX_POSITION_COV = 0.5  # Maximum 0.5m uncertainty
        MAX_ANGLE_COV = 0.01  # Maximum ~5.7° uncertainty

        self.P[0, 0] = min(self.P[0, 0], MAX_POSITION_COV)
        self.P[1, 1] = min(self.P[1, 1], MAX_POSITION_COV)
        self.P[2, 2] = min(self.P[2, 2], MAX_ANGLE_COV)

        self.publish_ekf(current_time)

    def imu_callback(self, msg):
        q = msg.orientation
        (_, _, yaw_measured) = euler_from_quaternion([q.x, q.y, q.z, q.w])

        z = np.array([[yaw_measured]])
        H = np.array([[0, 0, 1]])
        theta_predicted = self.x[2, 0]
        y = z - theta_predicted
        y[0, 0] = math.atan2(math.sin(y[0, 0]), math.cos(y[0, 0]))

        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.x[2, 0] = math.atan2(math.sin(self.x[2, 0]), math.cos(self.x[2, 0]))
        self.P = (np.eye(3) - K @ H) @ self.P

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

        # Populate Covariance
        cov = [0.0] * 36
        cov[0] = self.P[0, 0]
        cov[7] = self.P[1, 1]
        cov[35] = self.P[2, 2]
        msg.pose.covariance = cov

        if self.count % 10 == 0:
            print(
                f"DEBUG: EKF Updated State -> x={self.x[0,0]:.3f}, y={self.x[1,0]:.3f}, "
                f"P_xx={self.P[0,0]:.4f}, P_yy={self.P[1,1]:.4f}, P_θθ={self.P[2,2]:.4f}"
            )

        self.pub.publish(msg)

        # Broadcast TF transform
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
