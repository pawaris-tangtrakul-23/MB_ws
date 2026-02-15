import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import TransformStamped, Quaternion, PoseStamped
from tf2_ros import TransformBroadcaster
from tf_transformations import euler_from_quaternion
import numpy as np
import math
from scipy.spatial import KDTree

# --- CONFIGURATION ---
MAP_RESOLUTION = 0.05  # Meters per pixel
MAP_SIZE_X = 1000  # Width in pixels
MAP_SIZE_Y = 1000  # Height in pixels
MAP_ORIGIN_X = -25.0  # Real-world X at bottom-left
MAP_ORIGIN_Y = -25.0  # Real-world Y at bottom-left

# ICP Parameters
ICP_MAX_ITERATIONS = 100
ICP_TOLERANCE = 0.00001
ICP_OUTLIER_THRESHOLD = 0.2


# --- MATH FUNCTIONS ---
def bresenham_line(x0, y0, x1, y1):
    """Bresenham line algorithm for ray tracing"""
    points = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy

    while True:
        points.append((x0, y0))
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy
    return points


def get_cartesian_points(scan_msg):
    """Convert laser scan to Cartesian coordinates"""
    points = []
    angle = scan_msg.angle_min
    for r in scan_msg.ranges:
        if (
            scan_msg.range_min < r < scan_msg.range_max
            and not math.isinf(r)
            and not math.isnan(r)
        ):
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            points.append([x, y])
        angle += scan_msg.angle_increment
    return np.array(points)


def best_fit_transform(A, B):
    """Calculate best-fit rigid transformation between point sets A and B"""
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    AA = A - centroid_A
    BB = B - centroid_B
    H = np.dot(AA.T, BB)
    U, S, Vt = np.linalg.svd(H)
    R = np.dot(Vt.T, U.T)
    
    # Ensure proper rotation (det(R) = 1)
    if np.linalg.det(R) < 0:
        Vt[1, :] *= -1
        R = np.dot(Vt.T, U.T)
    
    t = centroid_B.T - np.dot(R, centroid_A.T)
    
    # Build 3x3 transformation matrix
    T = np.identity(3)
    T[:2, :2] = R
    T[:2, 2] = t
    return T


def icp(source, target, initial_guess=None, max_iterations=ICP_MAX_ITERATIONS, 
        tolerance=ICP_TOLERANCE, outlier_threshold=ICP_OUTLIER_THRESHOLD):
    """
    Iterative Closest Point algorithm with initial guess
    
    Args:
        source: Nx2 array of source points
        target: Mx2 array of target points
        initial_guess: 3x3 transformation matrix for initial alignment
        max_iterations: Maximum ICP iterations
        tolerance: Convergence threshold
        outlier_threshold: Distance threshold for outlier rejection
    
    Returns:
        T_final: 3x3 transformation matrix from source to target
        mean_error: Final mean error
        num_inliers: Number of inlier correspondences
    """
    src = np.copy(source)
    
    # Apply initial guess if provided
    if initial_guess is not None:
        src_h = np.ones((src.shape[0], 3))
        src_h[:, :2] = src
        src = (np.dot(initial_guess, src_h.T).T)[:, :2]
        T_final = np.copy(initial_guess)
    else:
        T_final = np.identity(3)
    
    # Build KD-tree for fast nearest neighbor search
    tree = KDTree(target)
    prev_error = float('inf')
    
    for i in range(max_iterations):
        # Find nearest neighbors
        distances, indices = tree.query(src)
        
        # Outlier rejection: remove points with distance > threshold
        inlier_mask = distances < outlier_threshold
        num_inliers = np.sum(inlier_mask)
        
        if num_inliers < 10:
            # Not enough inliers, return current estimate
            break
        
        # Compute transformation for this iteration
        T_step = best_fit_transform(src[inlier_mask], target[indices[inlier_mask]])
        
        # Apply transformation to source points
        src_h = np.ones((src.shape[0], 3))
        src_h[:, :2] = src
        src = (np.dot(T_step, src_h.T).T)[:, :2]
        
        # Update total transformation
        T_final = np.dot(T_step, T_final)
        
        # Check convergence
        mean_error = np.mean(distances[inlier_mask])
        if abs(prev_error - mean_error) < tolerance:
            break
        prev_error = mean_error
    
    return T_final, mean_error, num_inliers


def get_quaternion_from_yaw(yaw):
    """Convert yaw angle to quaternion"""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def pose_to_transform_matrix(x, y, yaw):
    """Convert pose (x, y, yaw) to 3x3 transformation matrix"""
    T = np.identity(3)
    T[0, 0] = math.cos(yaw)
    T[0, 1] = -math.sin(yaw)
    T[1, 0] = math.sin(yaw)
    T[1, 1] = math.cos(yaw)
    T[0, 2] = x
    T[1, 2] = y
    return T


def transform_matrix_to_pose(T):
    """Convert 3x3 transformation matrix to pose (x, y, yaw)"""
    x = T[0, 2]
    y = T[1, 2]
    yaw = math.atan2(T[1, 0], T[0, 0])
    return x, y, yaw


# --- NODE ---
class ICPEKFSlamNode(Node):
    def __init__(self):
        super().__init__("icp_ekf_slam_node")
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("ICP + EKF Fusion SLAM Node Starting...")
        self.get_logger().info("=" * 60)
        
        # Sensor QoS (Best Effort to match LiDAR and EKF)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        
        # Subscribe to laser scan
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, qos_profile=sensor_qos
        )
        
        # Subscribe to EKF odometry for initial guess
        self.ekf_sub = self.create_subscription(
            Odometry, "/odom_ekf", self.ekf_callback, qos_profile=sensor_qos
        )
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, "/odom_icp", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Map QoS (Transient Local for RViz compatibility)
        map_qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.map_pub = self.create_publisher(OccupancyGrid, "/map", qos_profile=map_qos)
        self.path_publisher = self.create_publisher(Path, "/path_icp", 10)
        
        # State variables
        self.grid_data = np.full((MAP_SIZE_Y, MAP_SIZE_X), -1, dtype=np.int8)
        self.prev_scan = None
        self.global_pose = np.identity(3)  # ICP global pose
        self.last_map_update_pose = np.identity(3)
        
        # EKF tracking
        self.ekf_pose = None  # Current EKF pose (x, y, yaw)
        self.prev_ekf_pose = None  # Previous EKF pose at last scan
        self.initialized_from_ekf = False  # Flag to initialize from first EKF reading
        
        # Statistics
        self.scan_count = 0
        self.icp_count = 0
        
        # Path tracking
        self.path = Path()
        self.path.header.frame_id = "odom"
        
        self.get_logger().info("Waiting for /scan and /odom_ekf...")
        self.get_logger().info("Will initialize ICP position from first EKF reading")
    
    def ekf_callback(self, msg):
        """Track EKF odometry for initial guess"""
        q = msg.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        self.ekf_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw
        )
    
    def scan_callback(self, msg):
        """Process laser scan with EKF-guided ICP"""
        self.scan_count += 1
        
        # Convert scan to Cartesian points
        current_scan = get_cartesian_points(msg)
        if len(current_scan) < 10:
            self.get_logger().warn(f"Scan {self.scan_count}: Too few points ({len(current_scan)})")
            return
        
        # Initialize ICP global pose from first EKF reading
        if not self.initialized_from_ekf and self.ekf_pose is not None:
            x_ekf, y_ekf, yaw_ekf = self.ekf_pose
            self.global_pose = pose_to_transform_matrix(x_ekf, y_ekf, yaw_ekf)
            self.last_map_update_pose = np.copy(self.global_pose)
            self.initialized_from_ekf = True
            self.get_logger().info(
                f"ICP initialized from EKF position: x={x_ekf:.3f}m, y={y_ekf:.3f}m, "
                f"yaw={math.degrees(yaw_ekf):.1f}deg"
            )
        
        # First scan - just store it
        if self.prev_scan is None:
            self.prev_scan = current_scan
            self.prev_ekf_pose = self.ekf_pose
            self.get_logger().info(f"Scan {self.scan_count}: Initialized with {len(current_scan)} points")
            return
        
        # Compute initial guess from EKF
        initial_guess = None
        if self.ekf_pose is not None and self.prev_ekf_pose is not None:
            # Get EKF poses as transformation matrices
            T_prev_ekf = pose_to_transform_matrix(*self.prev_ekf_pose)
            T_curr_ekf = pose_to_transform_matrix(*self.ekf_pose)
            
            # Compute relative motion from EKF
            T_ekf_relative = np.linalg.inv(T_prev_ekf) @ T_curr_ekf
            initial_guess = T_ekf_relative
            
            dx_ekf, dy_ekf, dyaw_ekf = transform_matrix_to_pose(T_ekf_relative)
            
            if self.scan_count % 10 == 0:
                self.get_logger().info(
                    f"Scan {self.scan_count}: EKF guess: dx={dx_ekf:.3f}m, dy={dy_ekf:.3f}m, "
                    f"dyaw={math.degrees(dyaw_ekf):.1f}deg"
                )
        
        # Run ICP with initial guess
        step_T, mean_error, num_inliers = icp(
            current_scan, 
            self.prev_scan, 
            initial_guess=initial_guess
        )
        
        # Update global pose
        self.global_pose = np.dot(self.global_pose, step_T)
        self.icp_count += 1
        
        # Get ICP motion for logging
        dx_icp, dy_icp, dyaw_icp = transform_matrix_to_pose(step_T)
        
        # Log ICP results periodically
        if self.scan_count % 10 == 0:
            self.get_logger().info(
                f"Scan {self.scan_count}: ICP refined: dx={dx_icp:.3f}m, dy={dy_icp:.3f}m, "
                f"dyaw={math.degrees(dyaw_icp):.1f}deg | "
                f"error={mean_error:.4f}m, inliers={num_inliers}"
            )
        
        # Publish odometry
        self.publish_odometry(msg.header.stamp)
        
        # Update map if robot moved significantly
        if self.should_update_map(self.global_pose):
            self.update_grid(current_scan)
            self.publish_map(msg.header.stamp)
            self.last_map_update_pose = np.copy(self.global_pose)
        
        # Update for next iteration
        self.prev_scan = current_scan
        self.prev_ekf_pose = self.ekf_pose
    
    def should_update_map(self, current_pose):
        """Check if robot moved enough to update map (>10cm)"""
        dx = current_pose[0, 2] - self.last_map_update_pose[0, 2]
        dy = current_pose[1, 2] - self.last_map_update_pose[1, 2]
        return math.sqrt(dx**2 + dy**2) > 0.1
    
    def update_grid(self, local_points):
        """Update occupancy grid with ray tracing"""
        # Transform points to global frame
        R = self.global_pose[:2, :2]
        t = self.global_pose[:2, 2]
        global_points = np.dot(local_points, R.T)
        global_points[:, 0] += t[0]
        global_points[:, 1] += t[1]
        
        # Robot position in pixel coordinates
        robot_px = int((t[0] - MAP_ORIGIN_X) / MAP_RESOLUTION)
        robot_py = int((t[1] - MAP_ORIGIN_Y) / MAP_RESOLUTION)
        
        # Ray tracing for each laser point
        for point in global_points:
            # Endpoint in pixel coordinates
            end_px = int((point[0] - MAP_ORIGIN_X) / MAP_RESOLUTION)
            end_py = int((point[1] - MAP_ORIGIN_Y) / MAP_RESOLUTION)
            
            # Check bounds
            if not (0 <= end_px < MAP_SIZE_X and 0 <= end_py < MAP_SIZE_Y):
                continue
            
            # Get all cells along the ray
            ray_cells = bresenham_line(robot_px, robot_py, end_px, end_py)
            
            # Mark all cells except last as free (0)
            for i, (px, py) in enumerate(ray_cells[:-1]):
                if 0 <= px < MAP_SIZE_X and 0 <= py < MAP_SIZE_Y:
                    self.grid_data[py, px] = 0
            
            # Mark endpoint as occupied (100)
            if 0 <= end_px < MAP_SIZE_X and 0 <= end_py < MAP_SIZE_Y:
                self.grid_data[end_py, end_px] = 100
    
    def publish_map(self, timestamp):
        """Publish occupancy grid map"""
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = timestamp
        grid_msg.header.frame_id = "odom"
        grid_msg.info.resolution = MAP_RESOLUTION
        grid_msg.info.width = MAP_SIZE_X
        grid_msg.info.height = MAP_SIZE_Y
        grid_msg.info.origin.position.x = MAP_ORIGIN_X
        grid_msg.info.origin.position.y = MAP_ORIGIN_Y
        grid_msg.info.origin.orientation.w = 1.0
        grid_msg.data = self.grid_data.flatten().tolist()
        self.map_pub.publish(grid_msg)
    
    def publish_odometry(self, timestamp):
        """Publish ICP-based odometry"""
        x, y, yaw = transform_matrix_to_pose(self.global_pose)
        q = get_quaternion_from_yaw(yaw)
        
        # Odometry message
        odom = Odometry()
        odom.header.stamp = timestamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link_icp"
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation = q
        self.odom_pub.publish(odom)
        
        # Add to path (every 2nd scan to avoid huge paths)
        if self.scan_count % 2 == 0:
            pose = PoseStamped()
            pose.header = odom.header
            pose.pose = odom.pose.pose
            self.path.poses.append(pose)
            self.path.header.stamp = timestamp
            self.path_publisher.publish(self.path)
        
        # TF transform
        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link_icp"
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = ICPEKFSlamNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f"Shutting down. Processed {node.scan_count} scans, {node.icp_count} ICP iterations")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
