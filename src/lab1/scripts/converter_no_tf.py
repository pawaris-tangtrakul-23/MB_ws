#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Quaternion, TransformStamped, PoseStamped
from tf2_ros import TransformBroadcaster
import math


class OdomPublisher(Node):
    def __init__(self):
        super().__init__("odom_publisher")

        # --- NEW PARAMETER TO DISABLE TF ---
        self.declare_parameter("publish_tf", True)
        self.publish_tf = self.get_parameter("publish_tf").value
        # -----------------------------------

        try:
            self.declare_parameter("use_sim_time", True)
        except:
            pass
        use_sim_time = self.get_parameter("use_sim_time").value
        self.get_logger().info(f"Using sim time: {use_sim_time}")
        self.get_logger().info(f"Publishing TF: {self.publish_tf}")

        self.wheel_radius = 0.033
        self.wheel_separation = 0.160

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.prev_left_wheel_pos = 0.0
        self.prev_right_wheel_pos = 0.0
        self.prev_time = None
        self.first_measurement = True

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.subscription = self.create_subscription(
            JointState, "/joint_states", self.joint_states_callback, qos
        )
        self.publisher_ = self.create_publisher(Odometry, "/odom_raw", 10)
        self.path_publisher = self.create_publisher(Path, "/path_raw", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.path = Path()
        self.path.header.frame_id = "odom"
        self.count = 0

    def euler_to_quaternion(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q

    def joint_states_callback(self, msg):
        self.count += 1
        current_time = (
            self.get_clock().now()
            if msg.header.stamp.sec == 0
            else Time.from_msg(msg.header.stamp)
        )

        if len(msg.position) < 2:
            return

        left_wheel_pos = msg.position[0]
        right_wheel_pos = msg.position[1]

        if self.first_measurement:
            # Only publish backdated TF if publish_tf is True
            if self.publish_tf:
                backdated_time = (
                    Time.from_msg(msg.header.stamp)
                    if msg.header.stamp.sec != 0
                    else current_time
                )
                backdated_time = Time(nanoseconds=backdated_time.nanoseconds - int(2e9))

                t = TransformStamped()
                t.header.stamp = backdated_time.to_msg()
                t.header.frame_id = "odom"
                t.child_frame_id = "base_link_ekf"
                t.transform.translation.x = 0.0
                t.transform.translation.y = 0.0
                t.transform.translation.z = 0.0
                t.transform.rotation = self.euler_to_quaternion(0, 0, 0)
                self.tf_broadcaster.sendTransform(t)

            self.prev_left_wheel_pos = left_wheel_pos
            self.prev_right_wheel_pos = right_wheel_pos
            self.prev_time = current_time
            self.first_measurement = False
            return

        dt = (current_time - self.prev_time).nanoseconds / 1e9
        if dt > 1.0 or dt <= 0:
            self.prev_time = current_time
            return

        delta_left = left_wheel_pos - self.prev_left_wheel_pos
        delta_right = right_wheel_pos - self.prev_right_wheel_pos
        left_distance = delta_left * self.wheel_radius
        right_distance = delta_right * self.wheel_radius
        distance = (left_distance + right_distance) / 2.0
        delta_theta = (right_distance - left_distance) / self.wheel_separation

        if abs(delta_theta) > 1e-6:
            radius = distance / delta_theta
            dx = radius * (math.sin(self.theta + delta_theta) - math.sin(self.theta))
            dy = radius * (-math.cos(self.theta + delta_theta) + math.cos(self.theta))
        else:
            dx = distance * math.cos(self.theta)
            dy = distance * math.sin(self.theta)

        self.x += dx
        self.y += dy
        self.theta += delta_theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        v = distance / dt
        w = delta_theta / dt

        self.publish_odometry(current_time, v, w)

        self.prev_left_wheel_pos = left_wheel_pos
        self.prev_right_wheel_pos = right_wheel_pos
        self.prev_time = current_time

    def publish_odometry(self, current_time, linear_vel, angular_vel):
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link_ekf"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        quat = self.euler_to_quaternion(0, 0, self.theta)
        odom.pose.pose.orientation = quat

        odom.twist.twist.linear.x = linear_vel
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = angular_vel

        # Covariance setup (simplified for brevity)
        odom.pose.covariance = [0.0] * 36
        odom.pose.covariance[0] = 0.1
        odom.pose.covariance[7] = 0.1
        odom.pose.covariance[35] = 0.2
        odom.twist.covariance = [0.0] * 36
        odom.twist.covariance[0] = 0.1
        odom.twist.covariance[35] = 0.2

        self.publisher_.publish(odom)

        if self.count % 5 == 0:
            pose = PoseStamped()
            pose.header = odom.header
            pose.pose = odom.pose.pose
            self.path.poses.append(pose)
            self.path.header.stamp = current_time.to_msg()
            self.path_publisher.publish(self.path)

        # --- ONLY PUBLISH TF IF ENABLED ---
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = "odom"
            t.child_frame_id = "base_link_ekf"
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = quat
            self.tf_broadcaster.sendTransform(t)
        # ----------------------------------


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(OdomPublisher())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
