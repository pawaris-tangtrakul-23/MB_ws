#!/usr/bin/env python3
"""
Path Publisher Node
Subscribes to multiple odometry topics and publishes their trajectories as Path messages.
Allows visualization of different odometry sources in RViz.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class PathPublisherNode(Node):
    def __init__(self):
        super().__init__("path_publisher_node")

        self.get_logger().info("=" * 60)
        self.get_logger().info("Path Publisher Node Starting...")
        self.get_logger().info("=" * 60)

        # QoS for odometry (Best Effort to match sources)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Subscribe to all odometry topics
        self.create_subscription(Odometry, "/odom_raw", self.odom_raw_callback, qos)
        self.create_subscription(Odometry, "/odom_ekf", self.odom_ekf_callback, qos)
        self.create_subscription(Odometry, "/odom_icp", self.odom_icp_callback, qos)

        # Publishers for paths
        self.path_raw_pub = self.create_publisher(Path, "/path_raw", 10)
        self.path_ekf_pub = self.create_publisher(Path, "/path_ekf", 10)
        self.path_icp_pub = self.create_publisher(Path, "/path_icp", 10)

        # Path storage
        self.path_raw = Path()
        self.path_raw.header.frame_id = "odom"

        self.path_ekf = Path()
        self.path_ekf.header.frame_id = "odom"

        self.path_icp = Path()
        self.path_icp.header.frame_id = "odom"

        # Counters for logging
        self.count_raw = 0
        self.count_ekf = 0
        self.count_icp = 0

        # Path decimation (publish every Nth pose to avoid huge paths)
        self.decimation = 5  # Publish every 5th pose
        self.decimation_counter_raw = 0
        self.decimation_counter_ekf = 0
        self.decimation_counter_icp = 0

        self.get_logger().info("Subscribed to: /odom_raw, /odom_ekf, /odom_icp")
        self.get_logger().info("Publishing to: /path_raw, /path_ekf, /path_icp")
        self.get_logger().info(f"Path decimation: {self.decimation} (every {self.decimation}th pose)")
        self.get_logger().info("=" * 60)

    def odom_raw_callback(self, msg):
        """Handle raw odometry from wheel encoders"""
        self.count_raw += 1
        self.decimation_counter_raw += 1

        # Decimate to reduce path size
        if self.decimation_counter_raw < self.decimation:
            return
        self.decimation_counter_raw = 0

        # Create PoseStamped from Odometry
        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = "odom"
        pose.pose = msg.pose.pose

        # Add to path
        self.path_raw.poses.append(pose)
        self.path_raw.header.stamp = msg.header.stamp

        # Publish path
        self.path_raw_pub.publish(self.path_raw)

        if self.count_raw % 50 == 0:
            self.get_logger().info(
                f"Raw Path: {len(self.path_raw.poses)} poses "
                f"(x={pose.pose.position.x:.3f}, y={pose.pose.position.y:.3f})"
            )

    def odom_ekf_callback(self, msg):
        """Handle EKF fused odometry"""
        self.count_ekf += 1
        self.decimation_counter_ekf += 1

        # Decimate to reduce path size
        if self.decimation_counter_ekf < self.decimation:
            return
        self.decimation_counter_ekf = 0

        # Create PoseStamped from Odometry
        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = "odom"
        pose.pose = msg.pose.pose

        # Add to path
        self.path_ekf.poses.append(pose)
        self.path_ekf.header.stamp = msg.header.stamp

        # Publish path
        self.path_ekf_pub.publish(self.path_ekf)

        if self.count_ekf % 50 == 0:
            self.get_logger().info(
                f"EKF Path: {len(self.path_ekf.poses)} poses "
                f"(x={pose.pose.position.x:.3f}, y={pose.pose.position.y:.3f})"
            )

    def odom_icp_callback(self, msg):
        """Handle ICP SLAM odometry"""
        self.count_icp += 1
        self.decimation_counter_icp += 1

        # Decimate to reduce path size
        if self.decimation_counter_icp < self.decimation:
            return
        self.decimation_counter_icp = 0

        # Create PoseStamped from Odometry
        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = "odom"
        pose.pose = msg.pose.pose

        # Add to path
        self.path_icp.poses.append(pose)
        self.path_icp.header.stamp = msg.header.stamp

        # Publish path
        self.path_icp_pub.publish(self.path_icp)

        if self.count_icp % 50 == 0:
            self.get_logger().info(
                f"ICP Path: {len(self.path_icp.poses)} poses "
                f"(x={pose.pose.position.x:.3f}, y={pose.pose.position.y:.3f})"
            )


def main(args=None):
    rclpy.init(args=args)
    node = PathPublisherNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("=" * 60)
        node.get_logger().info("Path Publisher Node Shutting Down")
        node.get_logger().info(f"Final path sizes:")
        node.get_logger().info(f"  Raw: {len(node.path_raw.poses)} poses")
        node.get_logger().info(f"  EKF: {len(node.path_ekf.poses)} poses")
        node.get_logger().info(f"  ICP: {len(node.path_icp.poses)} poses")
        node.get_logger().info("=" * 60)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
