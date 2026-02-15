import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster
import math


class OdomPublisher(Node):
    def __init__(self):
        super().__init__("odom_publisher")

        # Read use_sim_time parameter (set by launch file)
        try:
            use_sim_time = self.get_parameter("use_sim_time").value
            self.get_logger().info(f"✓ Using sim time: {use_sim_time}")
            # Also check what clock type we're actually using
            clock_type = self.get_clock().clock_type
            self.get_logger().info(f"✓ Clock type: {clock_type}")
        except Exception as e:
            self.get_logger().error(f"✗ Could not read use_sim_time parameter: {e}")

        self.wheel_separation = 0.160

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.subscription = self.create_subscription(
            JointState, "/joint_states", self.listener_callback, qos
        )
        self.publisher_ = self.create_publisher(Odometry, "/odom_raw", 10)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        self.x, self.y, self.th = 0.0, 0.0, 0.0
        self.last_time = None
        self.count = 0
        self.debug_count = 0

        print("=" * 60)
        print("DEBUG CONVERTER STARTED")
        print("=" * 60)

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

    def listener_callback(self, msg):
        self.count += 1
        self.debug_count += 1

        # CRITICAL: Use message timestamp, not current time!
        # This ensures TF timestamps match the original recording
        current_time = (
            self.get_clock().now()
            if msg.header.stamp.sec == 0
            else Time.from_msg(msg.header.stamp)
        )

        # Debug print every 10 messages
        if self.debug_count == 10:
            self.debug_count = 0
            if len(msg.velocity) >= 2:
                print(
                    f"[{self.count}] Joint vel: [{msg.velocity[0]:.4f}, {msg.velocity[1]:.4f}]"
                )
            else:
                print(f"[{self.count}] NO VELOCITY DATA!")

        if len(msg.velocity) < 2:
            return

        if self.last_time is None:
            # CRITICAL: Publish backdated TF to cover scans that arrived before converter started
            backdated_time = Time.from_msg(msg.header.stamp)
            backdated_time = Time(
                nanoseconds=backdated_time.nanoseconds - int(2e9)
            )  # 2 seconds earlier

            t = TransformStamped()
            t.header.stamp = backdated_time.to_msg()
            t.header.frame_id = "odom"
            t.child_frame_id = "base_link"
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            quat = self.euler_to_quaternion(0, 0, 0)
            t.transform.rotation = quat
            self.tf_broadcaster.sendTransform(t)
            print(
                f"[{self.count}] Published BACKDATED TF at {backdated_time.nanoseconds / 1e9:.3f}s"
            )

            self.last_time = current_time
            print(f"[{self.count}] Initialized time")
            print(
                f"  Using message timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}"
            )
            print(f"  Current ROS time: {self.get_clock().now().nanoseconds / 1e9:.3f}")
            return

        dt = (current_time - self.last_time).nanoseconds / 1e9

        # Debug time step
        if dt > 1.0 or dt <= 0:
            if self.count % 10 == 0:
                print(f"[{self.count}] BAD dt={dt:.4f}s - SKIPPING!")
            self.last_time = current_time
            return

        self.last_time = current_time

        # Joint states already contain linear velocities in m/s, not angular velocities
        vl = msg.velocity[0]
        vr = msg.velocity[1]
        v = (vr + vl) / 2.0
        w = (vr - vl) / self.wheel_separation

        # Debug velocities every 10 messages
        if self.count % 10 == 0:
            print(f"[{self.count}] dt={dt:.4f}s, v={v:.4f}m/s, w={w:.4f}rad/s")

        # Update position
        dx = v * dt * math.cos(self.th)
        dy = v * dt * math.sin(self.th)
        dth = w * dt

        self.x += dx
        self.y += dy
        self.th += dth

        # Debug position every 50 messages
        if self.count % 50 == 0:
            print(f"{'='*60}")
            print(f"[{self.count}] POSITION UPDATE:")
            print(f"  dx={dx:.6f}, dy={dy:.6f}, dth={dth:.6f}")
            print(f"  x={self.x:.4f}, y={self.y:.4f}, th={self.th:.4f}")
            print(f"  timestamp={current_time.nanoseconds / 1e9:.3f}s")
            print(f"{'='*60}")

        # Create odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        quat = self.euler_to_quaternion(0, 0, self.th)
        odom.pose.pose.orientation = quat

        odom.twist.twist.linear.x = v
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = w

        speed = abs(v)
        angular_speed = abs(w)

        pos_cov = 0.01 + 0.05 * speed
        yaw_cov = 0.01 + 0.1 * angular_speed
        vel_cov = 0.01 + 0.02 * speed
        ang_vel_cov = 0.01 + 0.05 * angular_speed

        odom.pose.covariance = [0.0] * 36
        odom.pose.covariance[0] = pos_cov
        odom.pose.covariance[7] = pos_cov
        odom.pose.covariance[14] = 999999.0
        odom.pose.covariance[21] = 999999.0
        odom.pose.covariance[28] = 999999.0
        odom.pose.covariance[35] = yaw_cov

        odom.twist.covariance = [0.0] * 36
        odom.twist.covariance[0] = vel_cov
        odom.twist.covariance[7] = 999999.0
        odom.twist.covariance[14] = 999999.0
        odom.twist.covariance[21] = 999999.0
        odom.twist.covariance[28] = 999999.0
        odom.twist.covariance[35] = ang_vel_cov

        self.publisher_.publish(odom)

        # Publish TF
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        t.transform.rotation = quat

        self.tf_broadcaster.sendTransform(t)


def main():
    # CRITICAL: Explicitly pass sys.argv so rclpy.init() sees --ros-args
    import sys

    rclpy.init(args=sys.argv)
    rclpy.spin(OdomPublisher())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
