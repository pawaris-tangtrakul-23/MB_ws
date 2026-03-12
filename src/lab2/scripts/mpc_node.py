#!/usr/bin/env python3

import math
import numpy as np
from scipy.linalg import expm, solve_discrete_are
from scipy import sparse
import osqp

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from actuator_msgs.msg import Actuators
from geometry_msgs.msg import Vector3
from std_msgs.msg import String


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


class MPCNode(Node):
    """
    Constrained Linear MPC controller for the quadrotor drone.

    Model (linearized around hover, small-angle approximation):
      State  x : [px, py, pz, roll, pitch, yaw, vx, vy, vz, p, q, r]  (12)
      Input du : [dF, tau_x, tau_y, tau_z]                              (4)
               where dF = F_total - m*g  (thrust deviation)

    Strategy:
      1. Build continuous-time A, B matrices.
      2. Discretize via Zero-Order Hold (ZOH).
      3. Build prediction matrices Sx, Su and QP cost/constraint matrices.
      4. At every step: solve a constrained QP using OSQP with input AND state constraints.
      5. Convert optimal [dF, tau_x, tau_y, tau_z] to motor speeds via the mixing matrix.

    Constraints (enforced INSIDE the optimization):
      - Input constraints:  u_min <= du_k <= u_max  (thrust, torque limits)
      - State constraints:  x_min <= x_k  <= x_max  (tilt angle, velocity limits)
    """

    # -----------------------------------------------------------------------
    # Init
    # -----------------------------------------------------------------------
    def __init__(self):
        super().__init__('mpc_node')

        # --- ROS I/O ---
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.motor_pub = self.create_publisher(
            Actuators, '/motor_commands', 10)
        self.target_sub = self.create_subscription(
            Vector3, '/set_target_xyz', self.target_callback, 10)
        self.mode_sub = self.create_subscription(
            String, '/set_flight_mode', self.mode_callback, 10)

        self.pub_curr_xyz = self.create_publisher(Vector3, '/debug/current_xyz', 10)
        self.pub_tgt_xyz  = self.create_publisher(Vector3, '/debug/target_xyz',  10)
        self.pub_curr_rpy = self.create_publisher(Vector3, '/debug/current_rpy', 10)
        self.pub_tgt_rpy  = self.create_publisher(Vector3, '/debug/target_rpy',  10)

        # --- Drone physical parameters (from robot_params.xacro) ---
        self.mass      = 1.5
        self.gravity   = 9.81
        self.I_xx      = 0.0347563
        self.I_yy      = 0.07
        self.I_zz      = 0.0977
        self.k_F       = 8.54858e-06   # thrust coefficient  F = k_F * omega^2
        self.k_M       = 0.06          # drag-torque / thrust ratio
        self.L_x       = 0.13          # rotor arm in x (m)
        self.L_y       = 0.22          # rotor arm in y (m)
        self.omega_max = 1500.0        # max motor speed (rad/s)

        # Equilibrium thrust (hover)
        self.F_eq = self.mass * self.gravity   # 14.715 N
        # Equilibrium deviation input (all zeros)
        self.u_eq = np.array([self.F_eq, 0.0, 0.0, 0.0])

        # --- Motor mixing: [F, tau_x, tau_y, tau_z] -> [T0, T1, T2, T3] ---
        #   Rotor layout (from URDF):
        #     0: front-right  ( Lx, -Ly)  CCW  (+yaw)
        #     1: rear-left   (-Lx, +Ly)  CCW  (+yaw)
        #     2: front-left  ( Lx, +Ly)  CW   (-yaw)
        #     3: rear-right  (-Lx, -Ly)  CW   (-yaw)
        self.M = np.array([
            [ 1.0,       1.0,      1.0,       1.0    ],
            [-self.L_y,  self.L_y, self.L_y, -self.L_y],
            [-self.L_x,  self.L_x,-self.L_x,  self.L_x],
            [-self.k_M, -self.k_M, self.k_M,  self.k_M],
        ])
        self.M_inv = np.linalg.inv(self.M)

        # --- MPC horizon & sampling time ---
        self.dt = 0.01   # 100 Hz  (same as LQR node)
        self.N  = 20     # prediction horizon steps (0.2 s)

        # --- Flight Envelope Limits ---
        self.max_tilt_rad    = math.radians(40)   # emergency cutoff: |roll| or |pitch| > 40°
        self.safe_tilt_rad   = math.radians(30)   # soft limit fed into tau clamp
        self.max_rate_rad    = math.radians(180)  # max angular rate allowed (rad/s)

        # --- Trajectory / target state ---
        self.trajectory_type = "NONE"
        self.flight_mode = "NONE"
        self.motors_active = False
        self.mode_start_time  = None
        self.traj_start_x = 0.0
        self.traj_start_y = 0.0
        self.traj_start_z = 2.0

        self.target_x   = 0.0
        self.target_y   = 0.0
        self.target_z   = 2.0
        self.target_yaw = 0.0

        # --- Current odometry state ---
        self.curr_x = 0.0;  self.curr_y = 0.0;  self.curr_z = 0.0
        self.curr_roll  = 0.0; self.curr_pitch = 0.0; self.curr_yaw = 0.0
        self.curr_vx = 0.0; self.curr_vy = 0.0;  self.curr_vz = 0.0
        self.curr_p  = 0.0; self.curr_q  = 0.0;  self.curr_r  = 0.0
        self.odom_ready = False

        # Velocity estimation via position differentiation (world-frame guaranteed)
        self.prev_x = 0.0; self.prev_y = 0.0; self.prev_z = 0.0
        self.prev_odom_time = None
        self.vel_alpha = 0.2  # low-pass filter: v_filtered = alpha*v_new + (1-alpha)*v_old

        self.last_time = self.get_clock().now()
        self.loop_count = 0

        # Altitude integral (eliminates steady-state z error)
        self.integral_z = 0.0
        self.ki_z = 2.0
        self.max_i_z = 5.0

        # --- Reference governor: limits how fast the target can move per step ---
        # Prevents large setpoint jumps from causing infeasibility
        self.gov_x = 0.0
        self.gov_y = 0.0
        self.gov_z = 2.0
        self.gov_max_step_xy = 0.02   # max 0.02 m/step → 2.0 m/s at 100 Hz
        self.gov_max_step_z  = 0.01   # max 0.01 m/step → 1.0 m/s at 100 Hz

        # --- Pre-compute MPC gain matrices ---
        self._setup_mpc()

        self.control_timer = self.create_timer(self.dt, self.control_loop)

    # -----------------------------------------------------------------------
    # MPC setup  (called once in __init__)
    # -----------------------------------------------------------------------
    def _build_linear_model(self):
        """
        Continuous-time linearized quadrotor around hover.

        State  x : [px, py, pz, roll, pitch, yaw, vx, vy, vz, p, q, r]
        Input du : [dF, tau_x, tau_y, tau_z]   (deviations from equilibrium)

        Dynamics (small-angle, hover linearisation):
          px_dot   = vx
          py_dot   = vy
          pz_dot   = vz
          roll_dot = p
          pitch_dot= q
          yaw_dot  = r
          vx_dot   =  g * pitch
          vy_dot   = -g * roll
          vz_dot   =  dF / m
          p_dot    =  tau_x / I_xx
          q_dot    =  tau_y / I_yy
          r_dot    =  tau_z / I_zz
        """
        g   = self.gravity
        m   = self.mass
        Ixx = self.I_xx
        Iyy = self.I_yy
        Izz = self.I_zz

        A = np.zeros((12, 12))
        # kinematics
        A[0, 6]  = 1.0   # px_dot   = vx
        A[1, 7]  = 1.0   # py_dot   = vy
        A[2, 8]  = 1.0   # pz_dot   = vz
        A[3, 9]  = 1.0   # roll_dot = p
        A[4, 10] = 1.0   # pitch_dot= q
        A[5, 11] = 1.0   # yaw_dot  = r
        # velocity coupling with attitude (signs from LQR outer loop convention)
        # Verified: positive pitch → +vx, negative roll → +vy
        A[6, 4]  =  g    # vx_dot = +g * pitch
        A[7, 3]  = -g    # vy_dot = -g * roll

        B = np.zeros((12, 4))
        B[8,  0] = 1.0 / m    # vz_dot  = dF / m
        B[9,  1] = 1.0 / Ixx  # p_dot   = tau_x / I_xx
        B[10, 2] = 1.0 / Iyy  # q_dot   = tau_y / I_yy
        B[11, 3] = 1.0 / Izz  # r_dot   = tau_z / I_zz

        return A, B

    def _discretize_zoh(self, A, B, dt):
        """Zero-Order Hold discretization via matrix exponential."""
        n, m = A.shape[0], B.shape[1]
        M_aug = np.zeros((n + m, n + m))
        M_aug[:n, :n] = A
        M_aug[:n, n:] = B
        M_exp = expm(M_aug * dt)
        Ad = M_exp[:n, :n]
        Bd = M_exp[:n, n:]
        return Ad, Bd

    def _setup_mpc(self):
        """
        Build the constrained MPC as a QP solved online at each step via OSQP.

        The constrained MPC problem:
          min_{DeltaU}  sum_k  ||x_k - x_ref||^2_Q  +  ||du_k||^2_R

          s.t.  x_{k+1} = Ad x_k + Bd du_k       (dynamics)
                du_min  <= du_k <= du_max           (input constraints)
                x_min   <= x_k  <= x_max            (state constraints)

        QP form (decision variable: DeltaU, the stacked input sequence):
          min  0.5 * DeltaU^T H DeltaU + q^T DeltaU
          s.t. l <= A_ineq * DeltaU <= u

        Prediction model:  X = Sx * x0 + Su * DeltaU
        """
        A, B = self._build_linear_model()
        Ad, Bd = self._discretize_zoh(A, B, self.dt)
        self.Ad = Ad
        self.Bd = Bd

        n, m, N = 12, 4, self.N

        # --- Build Sx (N*n x n) and Su (N*n x N*m) ---
        Sx = np.zeros((N * n, n))
        Su = np.zeros((N * n, N * m))

        Ad_pow = np.eye(n)
        for i in range(N):
            Ad_pow = Ad @ Ad_pow
            Sx[i*n:(i+1)*n, :] = Ad_pow

        for i in range(N):
            for j in range(i + 1):
                Su[i*n:(i+1)*n, j*m:(j+1)*m] = (
                    np.linalg.matrix_power(Ad, i - j) @ Bd
                )

        self.Sx = Sx
        self.Su = Su

        # --- Cost matrices (same tuning as before) ---
        q_diag = np.array([
              3.0,    3.0,   50.0,  # position  (low x-y, strong z)
              5.0,    5.0,    2.0,  # attitude  (moderate)
              0.01,   0.01,   0.1,  # velocity  (≈0 — avoid noise coupling)
              0.05,   0.05,  0.005, # angular rate (light damping)
        ])
        Q = np.diag(q_diag)

        r_diag = np.array([1.0, 0.5, 0.5, 0.5])
        R = np.diag(r_diag)

        # Terminal cost via DARE
        P_inf = solve_discrete_are(Ad, Bd, Q, R)

        Q_bar = np.kron(np.eye(N), Q)
        Q_bar[-n:, -n:] = P_inf

        R_bar = np.kron(np.eye(N), R)

        # --- QP cost: H and f_factor (f = f_factor @ (X_ref - Sx @ x0)) ---
        #   J = 0.5 * DU^T H DU + f^T DU
        #   where f = -Su^T Q_bar (X_ref - Sx x0)
        self.H = Su.T @ Q_bar @ Su + R_bar           # (N*m x N*m)
        self.f_factor = Su.T @ Q_bar                  # (N*m x N*n)

        # =====================================================================
        # CONSTRAINTS
        # =====================================================================

        # --- 1. Input constraints: du_min <= du_k <= du_max ---
        #   du = [dF, tau_x, tau_y, tau_z]
        #   dF = F_total - F_eq, so dF_min = -F_eq (motor off), dF_max = 1.5*F_eq
        du_min = np.array([-self.F_eq, -3.0, -3.0, -1.0])
        du_max = np.array([ 1.5 * self.F_eq,  3.0,  3.0,  1.0])

        # Stack for N steps: DU_min/max shape (N*m,)
        DU_min = np.tile(du_min, N)
        DU_max = np.tile(du_max, N)

        # Input constraint matrix: I_{N*m} * DeltaU
        A_input = np.eye(N * m)

        # --- 2. State constraints: x_min <= x_k <= x_max ---
        #   Constrained states: roll (idx 3), pitch (idx 4), vz (idx 8)
        #   Unconstrained states get -inf/+inf bounds
        INF = 1e6
        tilt_max = math.radians(30)   # ±30° tilt limit (keeps linear model valid)
        vz_max = 2.0                  # ±2 m/s vertical speed limit

        x_min_single = np.array([
            -INF, -INF, -INF,           # px, py, pz: no constraint
            -tilt_max, -tilt_max, -INF,  # roll, pitch constrained; yaw free
            -INF, -INF, -vz_max,         # vx, vy free; vz constrained
            -INF, -INF, -INF,            # p, q, r: no constraint
        ])
        x_max_single = np.array([
            INF, INF, INF,
            tilt_max, tilt_max, INF,
            INF, INF, vz_max,
            INF, INF, INF,
        ])

        # Stack for N steps
        X_min = np.tile(x_min_single, N)
        X_max = np.tile(x_max_single, N)

        # State constraint in terms of DeltaU:
        #   X = Sx x0 + Su DeltaU
        #   X_min <= Su DeltaU + Sx x0 <= X_max
        #   X_min - Sx x0 <= Su DeltaU <= X_max - Sx x0
        # The Sx x0 part is updated at each step (it depends on current state)
        A_state = Su   # (N*n x N*m)

        # --- Combine input + state constraints ---
        #   [ I    ] DeltaU   in  [DU_min,   DU_max  ]     (input)
        #   [ Su   ] DeltaU   in  [X_min-Sx*x0, X_max-Sx*x0]  (state)
        self.A_qp = np.vstack([A_input, A_state])   # ((N*m + N*n) x N*m)

        # Store bounds templates (state bounds are adjusted each step)
        self.DU_min = DU_min
        self.DU_max = DU_max
        self.X_min = X_min
        self.X_max = X_max

        # --- Create OSQP solver instance (setup once, update each step) ---
        H_sparse = sparse.csc_matrix(self.H)
        A_sparse = sparse.csc_matrix(self.A_qp)

        # Initial bounds (will be updated each step)
        l_init = np.concatenate([DU_min, X_min])
        u_init = np.concatenate([DU_max, X_max])
        q_init = np.zeros(N * m)

        self.solver = osqp.OSQP()
        self.solver.setup(
            P=H_sparse,
            q=q_init,
            A=A_sparse,
            l=l_init,
            u=u_init,
            warm_starting=True,
            verbose=False,
            eps_abs=1e-4,
            eps_rel=1e-4,
            max_iter=200,       # keep solve time bounded for 100 Hz
            polish=True,
        )

        # Store last solution for warm-starting fallback
        self.du_prev = np.zeros(m)

        self.get_logger().info(
            f'Constrained MPC ready: N={N}, dt={self.dt}s, '
            f'QP size: {N*m} vars, {self.A_qp.shape[0]} constraints '
            f'(input: {N*m}, state: {N*n})'
        )

    # -----------------------------------------------------------------------
    # QP solve  (called each control step)
    # -----------------------------------------------------------------------
    def _solve_mpc(self, x0, X_ref):
        """
        Solve the constrained QP for the current state x0 and reference X_ref.
        Returns the first optimal input du* (4,).
        """
        n, m, N = 12, 4, self.N

        # --- Update QP linear cost: q = -f_factor @ (X_ref - Sx @ x0) ---
        q_vec = -self.f_factor @ (X_ref - self.Sx @ x0)

        # --- Update state constraint bounds ---
        #   X_min - Sx x0 <= Su DeltaU <= X_max - Sx x0
        Sx_x0 = self.Sx @ x0
        l_state = self.X_min - Sx_x0
        u_state = self.X_max - Sx_x0

        l_new = np.concatenate([self.DU_min, l_state])
        u_new = np.concatenate([self.DU_max, u_state])

        # Update OSQP problem data
        self.solver.update(q=q_vec, l=l_new, u=u_new)

        # Solve QP
        result = self.solver.solve()

        if result.info.status == 'solved' or result.info.status == 'solved_inaccurate':
            DU_opt = result.x
            du_opt = DU_opt[:m]   # first input (receding horizon)
            self.du_prev = du_opt.copy()
            return du_opt
        else:
            # QP infeasible or failed — fall back to zero (hover), NOT previous input
            # Replaying an aggressive old input is what causes the cascade collapse
            self.get_logger().warn(
                f'QP status: {result.info.status} — falling back to hover')
            self.du_prev = np.zeros(m)
            return self.du_prev

    # -----------------------------------------------------------------------
    # ROS callbacks
    # -----------------------------------------------------------------------
    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        self.curr_x = pos.x
        self.curr_y = pos.y
        self.curr_z = pos.z

        q = msg.pose.pose.orientation
        self.curr_roll, self.curr_pitch, self.curr_yaw = (
            euler_from_quaternion(q.x, q.y, q.z, q.w)
        )

        # Velocity estimation: numerical differentiation of world-frame position
        # (avoids body-to-world rotation issues that caused slow divergence)
        now = self.get_clock().now()
        if self.prev_odom_time is not None:
            odom_dt = (now - self.prev_odom_time).nanoseconds / 1e9
            if odom_dt > 1e-6:
                vx_raw = (self.curr_x - self.prev_x) / odom_dt
                vy_raw = (self.curr_y - self.prev_y) / odom_dt
                vz_raw = (self.curr_z - self.prev_z) / odom_dt
                # Low-pass filter to reduce noise
                a = self.vel_alpha
                self.curr_vx = a * vx_raw + (1.0 - a) * self.curr_vx
                self.curr_vy = a * vy_raw + (1.0 - a) * self.curr_vy
                self.curr_vz = a * vz_raw + (1.0 - a) * self.curr_vz

        self.prev_x = self.curr_x
        self.prev_y = self.curr_y
        self.prev_z = self.curr_z
        self.prev_odom_time = now

        ang = msg.twist.twist.angular
        self.curr_p  = ang.x
        self.curr_q  = ang.y
        self.curr_r  = ang.z

        if not self.odom_ready:
            # Initialize governor to current position on first odom
            self.gov_x = self.curr_x
            self.gov_y = self.curr_y
            self.gov_z = self.curr_z
        self.odom_ready = True

        self.pub_curr_xyz.publish(
            Vector3(x=self.curr_x, y=self.curr_y, z=self.curr_z))
        self.pub_tgt_xyz.publish(
            Vector3(x=self.target_x, y=self.target_y, z=self.target_z))
        self.pub_curr_rpy.publish(
            Vector3(x=self.curr_roll, y=self.curr_pitch, z=self.curr_yaw))
        self.pub_tgt_rpy.publish(
            Vector3(x=0.0, y=0.0, z=self.target_yaw))

    def target_callback(self, msg):
        self.trajectory_type = "GOTO"
        self.target_x = msg.x
        self.target_y = msg.y
        self.target_z = msg.z
        self.get_logger().info(
            f"GOTO XYZ: ({msg.x:.2f}, {msg.y:.2f}, {msg.z:.2f})")

    def mode_callback(self, msg):
        new_mode = msg.data.upper()

        if new_mode == "IDLE":
            self.motors_active = False
            self.flight_mode = "NONE"
            self.trajectory_type = "NONE"
            self.get_logger().info("Mode -> IDLE (Motors OFF)")
            return

        if not self.motors_active:
            self.motors_active = True
            self.traj_start_x = self.curr_x
            self.traj_start_y = self.curr_y
            self.traj_start_z = 2.0

            self.target_x = self.curr_x
            self.target_y = self.curr_y
            self.target_z = 2.0

            # Reset governor to current position so it doesn't lag
            self.gov_x = self.curr_x
            self.gov_y = self.curr_y
            self.gov_z = self.curr_z

            self.integral_z = 0.0
            self.get_logger().info("Taking off to 2.0m Hover!")

        if new_mode == "2D":
            self.flight_mode = "2D"
            self.trajectory_type = "HOVER"
            self.get_logger().info("Mode -> 2D (Waiting for target X, Z...)")

        elif new_mode == "3D":
            self.flight_mode = "3D"
            self.trajectory_type = "HOVER"
            self.get_logger().info("Mode -> 3D (Waiting for target X, Y, Z...)")

        elif new_mode == "2D_SINE":
            self.flight_mode = "2D"
            self.trajectory_type = "SINE"
            self._init_traj_start()
            self.get_logger().info("Mode -> 2D SINE WAVE")

        elif new_mode == "2D_STRAIGHT_F":
            self.flight_mode = "2D"
            self.trajectory_type = "STRAIGHT_F"
            self._init_traj_start()
            self.get_logger().info("Mode -> 2D STRAIGHT FORWARD (+3 m/s)")

        elif new_mode == "2D_STRAIGHT_B":
            self.flight_mode = "2D"
            self.trajectory_type = "STRAIGHT_B"
            self._init_traj_start()
            self.get_logger().info("Mode -> 2D STRAIGHT BACKWARD (-3 m/s)")

        elif new_mode == "2D_RAMP_WAVE":
            self.flight_mode = "2D"
            self.trajectory_type = "RAMP_WAVE"
            self._init_traj_start()
            self.get_logger().info("Mode -> 2D RAMP WAVE (Triangle Zigzag)")

        elif new_mode == "3D_HELIX":
            self.flight_mode = "3D"
            self.trajectory_type = "HELIX"
            self._init_traj_start()
            self.get_logger().info("Mode -> 3D HELIX")

        elif new_mode == "3D_STRAIGHT":
            self.flight_mode = "3D"
            self.trajectory_type = "STRAIGHT_3D"
            self._init_traj_start()
            self.get_logger().info("Mode -> 3D STRAIGHT LINE")

        elif new_mode == "3D_FIGURE8":
            self.flight_mode = "3D"
            self.trajectory_type = "FIGURE8_3D"
            self._init_traj_start()
            self.get_logger().info("Mode -> 3D FIGURE-8")

        else:
            self.get_logger().warn(
                "Unknown mode! Use: IDLE, 2D, 3D, 2D_SINE, 2D_STRAIGHT_F, 2D_STRAIGHT_B, 3D_HELIX, 2D_RAMP_WAVE, 3D_STRAIGHT, 3D_FIGURE8"
            )

    def _init_traj_start(self):
        """Snapshot current position as trajectory origin and reset timer."""
        self.mode_start_time = self.get_clock().now()
        self.traj_start_x = self.curr_x
        self.traj_start_y = self.curr_y
        # Use target_z (hover altitude) if drone hasn't reached it yet
        self.traj_start_z = max(self.curr_z, self.target_z)

    # -----------------------------------------------------------------------
    # Trajectory generator
    # -----------------------------------------------------------------------
    def _generate_trajectory(self, current_time):
        if self.trajectory_type in ["NONE", "HOVER", "GOTO"]:
            return

        if self.mode_start_time is None:
            self.mode_start_time = current_time

        t_traj = (current_time - self.mode_start_time).nanoseconds / 1e9

        if self.trajectory_type == "SINE":
            self.target_x = self.traj_start_x + 0.3 * t_traj
            self.target_z = self.traj_start_z + 1.0 * math.sin(0.5 * t_traj)

        elif self.trajectory_type == "STRAIGHT_F":
            self.target_x = self.traj_start_x + 3.0 * t_traj
            self.target_z = self.traj_start_z

        elif self.trajectory_type == "STRAIGHT_B":
            self.target_x = self.traj_start_x - 3.0 * t_traj
            self.target_z = self.traj_start_z

        elif self.trajectory_type == "RAMP_WAVE":
            period = 4.0
            phase = (t_traj % period) / period

            if phase < 0.5:
                z_wave = 2.0 * phase
            else:
                z_wave = 2.0 * (1.0 - phase)

            self.target_x = self.traj_start_x + 0.8 * t_traj
            self.target_z = self.traj_start_z + (1.5 * z_wave)

        elif self.trajectory_type == "STRAIGHT_3D":
            vx, vy, vz = 1.0, 0.5, 0.2
            self.target_x = self.traj_start_x + vx * t_traj
            self.target_y = self.traj_start_y + vy * t_traj
            self.target_z = self.traj_start_z + vz * t_traj

        elif self.trajectory_type == "FIGURE8_3D":
            a = 2.0
            omega = 0.3
            # Parametric figure-8 using sin/cos (smooth, centered at start)
            self.target_x = self.traj_start_x + a * math.sin(omega * t_traj)
            self.target_y = self.traj_start_y + a * math.sin(omega * t_traj) * math.cos(omega * t_traj)
            self.target_z = self.traj_start_z + 0.5 * math.sin(omega * t_traj * 2.0)

        elif self.trajectory_type == "HELIX":
            radius = 2.0
            omega = 0.5
            self.target_x = self.traj_start_x + radius * math.sin(omega * t_traj)
            self.target_y = self.traj_start_y + radius * (
                1.0 - math.cos(omega * t_traj)
            )
            self.target_z = self.traj_start_z + 0.1 * t_traj

    # -----------------------------------------------------------------------
    # Main control loop  (100 Hz)
    # -----------------------------------------------------------------------
    def control_loop(self):
        if not self.odom_ready:
            return

        # IDLE: stop all motors
        if not self.motors_active:
            msg_out = Actuators()
            msg_out.header.stamp = self.get_clock().now().to_msg()
            msg_out.velocity = [0.0, 0.0, 0.0, 0.0]
            self.motor_pub.publish(msg_out)
            return

        curr_time = self.get_clock().now()
        dt = (curr_time - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            return
        self.last_time = curr_time

        # Update reference trajectory
        self._generate_trajectory(curr_time)

        # --- Current state vector x0 ---
        x0 = np.array([
            self.curr_x,     self.curr_y,     self.curr_z,
            self.curr_roll,  self.curr_pitch,  self.curr_yaw,
            self.curr_vx,    self.curr_vy,     self.curr_vz,
            self.curr_p,     self.curr_q,      self.curr_r,
        ])

        # --- Reference governor: rate-limit target to prevent QP infeasibility ---
        # Moves gov_xyz toward target_xyz at a bounded rate each step
        dx = self.target_x - self.gov_x
        dy = self.target_y - self.gov_y
        dz = self.target_z - self.gov_z

        dist_xy = math.sqrt(dx*dx + dy*dy)
        if dist_xy > self.gov_max_step_xy:
            scale_xy = self.gov_max_step_xy / dist_xy
            self.gov_x += dx * scale_xy
            self.gov_y += dy * scale_xy
        else:
            self.gov_x = self.target_x
            self.gov_y = self.target_y

        if abs(dz) > self.gov_max_step_z:
            self.gov_z += math.copysign(self.gov_max_step_z, dz)
        else:
            self.gov_z = self.target_z

        # --- Reference state (governed, rate-limited) ---
        x_ref = np.array([
            self.gov_x, self.gov_y, self.gov_z,
            0.0,        0.0,        self.target_yaw,
            0.0,        0.0,        0.0,
            0.0,        0.0,        0.0,
        ])
        X_ref = np.tile(x_ref, self.N)   # shape: (N*12,)

        # --- Solve constrained QP for optimal first input ---
        du_opt = self._solve_mpc(x0, X_ref)   # shape: (4,)

        # --- Convert deviation -> absolute input ---
        # u = [F, tau_x, tau_y, tau_z],  u = du + u_eq
        # Add altitude integral to eliminate steady-state z error
        error_z = self.gov_z - self.curr_z
        self.integral_z = float(np.clip(
            self.integral_z + error_z * dt, -self.max_i_z, self.max_i_z))
        F     = du_opt[0] + self.F_eq + self.ki_z * self.integral_z
        tau_x = du_opt[1]
        tau_y = du_opt[2]
        tau_z = du_opt[3]

        # --- Flight Envelope Check (hard safety cutoff) ---
        # If attitude exceeds 45° the linear model is invalid.
        # Apply strong PD recovery torques to level the drone.
        if (abs(self.curr_roll)  > math.radians(45) or
                abs(self.curr_pitch) > math.radians(45)):
            self.get_logger().warn(
                f'Envelope exceeded! roll={math.degrees(self.curr_roll):.1f}° '
                f'pitch={math.degrees(self.curr_pitch):.1f}° — applying recovery')
            F     = self.F_eq
            tau_x = -8.0 * self.curr_roll  - 3.0 * self.curr_p
            tau_y = -8.0 * self.curr_pitch - 3.0 * self.curr_q
            tau_z = -1.0 * self.curr_r

        # Safety-net clamps (should rarely activate — constraints are in the QP)
        F     = float(np.clip(F,     0.0,  2.5 * self.F_eq))
        tau_x = float(np.clip(tau_x, -3.0,  3.0))
        tau_y = float(np.clip(tau_y, -3.0,  3.0))
        tau_z = float(np.clip(tau_z, -1.0,  1.0))

        # --- Thrust-aware motor mixing ---
        # Priority: preserve total thrust F (altitude), scale down torques if needed.
        # Step 1: Compute thrust-only and torque-only motor allocations
        T_f   = self.M_inv @ np.array([F, 0.0, 0.0, 0.0])    # F-only per motor
        T_tau = self.M_inv @ np.array([0.0, tau_x, tau_y, tau_z])  # torque-only

        # Step 2: Find max scale so that T_f + scale * T_tau >= 0 for all motors
        scale = 1.0
        for i in range(4):
            if T_tau[i] < 0 and T_f[i] + T_tau[i] < 0:
                # T_f[i] + scale * T_tau[i] = 0  =>  scale = T_f[i] / (-T_tau[i])
                s = max(0.0, T_f[i] / (-T_tau[i]))
                scale = min(scale, s)

        T_motors = T_f + scale * T_tau
        # Clip any tiny negatives from float precision
        T_motors = np.maximum(T_motors, 0.0)

        w_cmds = []
        for Ti in T_motors:
            w    = math.sqrt(Ti / self.k_F)
            w    = min(self.omega_max, float(w))
            w_cmds.append(w)

        # --- Publish motor commands ---
        msg_out = Actuators()
        msg_out.header.stamp = curr_time.to_msg()
        msg_out.velocity = w_cmds
        self.motor_pub.publish(msg_out)

        # Debug logging every 2 seconds (or every cycle if tilt > 15°)
        self.loop_count += 1
        tilt_deg = max(abs(math.degrees(self.curr_roll)), abs(math.degrees(self.curr_pitch)))
        if self.loop_count % 200 == 0 or tilt_deg > 15:
            self.get_logger().info(
                f'pos=({self.curr_x:.2f},{self.curr_y:.2f},{self.curr_z:.2f}) '
                f'gov=({self.gov_x:.2f},{self.gov_y:.2f},{self.gov_z:.2f}) '
                f'tgt=({self.target_x:.2f},{self.target_y:.2f},{self.target_z:.2f}) '
                f'F={F:.1f} tx={tau_x:.2f} ty={tau_y:.2f} '
                f'roll={math.degrees(self.curr_roll):.1f}° '
                f'pitch={math.degrees(self.curr_pitch):.1f}° '
                f'w=[{w_cmds[0]:.0f},{w_cmds[1]:.0f},{w_cmds[2]:.0f},{w_cmds[3]:.0f}]'
            )


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = MPCNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
