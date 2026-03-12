#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from actuator_msgs.msg import Actuators
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
import math

# ฟังก์ชันสำหรับแปลง Quaternion เป็นมุม Euler (Roll, Pitch, Yaw)
def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z

class HoverController(Node):
    def __init__(self):
        super().__init__('hover_controller')
        
        # --- Subscribers & Publishers ---
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.motor_pub = self.create_publisher(Actuators, '/motor_commands', 10)

        self.pub_curr_z = self.create_publisher(Float64, '/debug/current_z', 10)
        self.pub_tgt_z = self.create_publisher(Float64, '/debug/target_z', 10)
        self.pub_curr_rpy = self.create_publisher(Vector3, '/debug/current_rpy', 10)
        self.pub_tgt_rpy = self.create_publisher(Vector3, '/debug/target_rpy', 10)

        # --- Targets (จุดเป้าหมายสำหรับการ Hover) ---
        self.target_z = 2.0
        self.target_roll = 0.0
        self.target_pitch = 0.0
        self.target_yaw = 0.0
        
        # ==========================================
        # 1. PID Gains สำหรับความสูง Z (Outer Loop)
        # ==========================================
        self.kp_z = 100.0; self.ki_z = 0.5; self.kd_z = 35.0
        
        # ==========================================
        # 2. P Gains สำหรับ Angle (Outer Cascade Loop)
        # ทำหน้าที่เปลี่ยน Angle Error เป็น Target Rate
        # ==========================================
        self.kp_angle_roll = 0.0
        self.kp_angle_pitch = 0.0
        self.kp_angle_yaw = 0.0
        
        # ==========================================
        # 3. PID Gains สำหรับ Rate (Inner Cascade Loop)
        # ทำหน้าที่ควบคุมความเร็วเชิงมุมให้ได้ตาม Target Rate
        # ==========================================
        self.kp_rate_roll = 0.0; self.ki_rate_roll = 0.0; self.kd_rate_roll = 0.0
        self.kp_rate_pitch = 0.0; self.ki_rate_pitch = 0.0; self.kd_rate_pitch = 0.0
        self.kp_rate_yaw = 0.0; self.ki_rate_yaw = 0.0; self.kd_rate_yaw = 0.0
        
        # ==========================================
        # 4. Limits: Anti-Windup & Motor Saturation
        # ==========================================
        self.max_i_z = 200.0        # ลิมิตค่า I สะสมสำหรับแกน Z
        self.max_i_rate = 100.0     # ลิมิตค่า I สะสมสำหรับ Rate (Roll, Pitch, Yaw)
        self.max_motor_vel = 1500.0 # ลิมิตความเร็วรอบสูงสุดของมอเตอร์ (ปรับตามสเปคของโดรนใน Gazebo)

        # --- States ปัจจุบัน ---
        self.curr_z = 0.0
        self.curr_roll = 0.0; self.curr_pitch = 0.0; self.curr_yaw = 0.0
        self.curr_roll_rate = 0.0; self.curr_pitch_rate = 0.0; self.curr_yaw_rate = 0.0
        self.odom_ready = False
        
        # --- Internal PID States ---
        self.prev_error_z = 0.0; self.integral_z = 0.0
        self.prev_error_roll_rate = 0.0; self.integral_roll_rate = 0.0
        self.prev_error_pitch_rate = 0.0; self.integral_pitch_rate = 0.0
        self.prev_error_yaw_rate = 0.0; self.integral_yaw_rate = 0.0
        
        self.last_time_z = self.get_clock().now()
        self.last_time_att = self.get_clock().now()
        
        # --- Shared Variables ---
        self.base_velocity = 656.0 
        self.total_thrust = self.base_velocity
        
        # --- Timers ---
        self.z_timer = self.create_timer(0.1, self.z_control_loop)        # 10 Hz
        self.att_timer = self.create_timer(0.01, self.attitude_control_loop) # 100 Hz

    def odom_callback(self, msg):
        """ รับค่า Odometry ทั้ง Position, Orientation และ Angular Velocity """
        # อัปเดตตำแหน่ง Z และมุม Euler
        self.curr_z = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        self.curr_roll, self.curr_pitch, self.curr_yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
        
        # ดึงค่าความเร็วเชิงมุม (Angular Velocity) มาเพื่อใช้ใน Rate Loop
        self.curr_roll_rate = msg.twist.twist.angular.x
        self.curr_pitch_rate = msg.twist.twist.angular.y
        self.curr_yaw_rate = msg.twist.twist.angular.z
        
        self.odom_ready = True

        # ส่งค่า Debug 
        self.pub_curr_z.publish(Float64(data=self.curr_z))
        self.pub_tgt_z.publish(Float64(data=self.target_z))
        curr_rpy_msg = Vector3(x=math.degrees(self.curr_roll), y=math.degrees(self.curr_pitch), z=math.degrees(self.curr_yaw))
        self.pub_curr_rpy.publish(curr_rpy_msg)
        tgt_rpy_msg = Vector3(x=math.degrees(self.target_roll), y=math.degrees(self.target_pitch), z=math.degrees(self.target_yaw))
        self.pub_tgt_rpy.publish(tgt_rpy_msg)

    def z_control_loop(self):
        """ SLOW LOOP (10 Hz): คุมตำแหน่งแกน Z """
        if not self.odom_ready:
            return

        curr_time = self.get_clock().now()
        dt = (curr_time - self.last_time_z).nanoseconds / 1e9
        if dt <= 0: return

        # PID สำหรับ Z
        error_z = self.target_z - self.curr_z
        
        # --- Anti-Windup สำหรับแกน Z ---
        self.integral_z += error_z * dt
        self.integral_z = max(-self.max_i_z, min(self.max_i_z, self.integral_z))
        
        deriv_z = (error_z - self.prev_error_z) / dt
        thrust_cmd = (self.kp_z * error_z) + (self.ki_z * self.integral_z) + (self.kd_z * deriv_z)

        self.total_thrust = self.base_velocity + thrust_cmd
        self.prev_error_z = error_z
        self.last_time_z = curr_time

    def attitude_control_loop(self):
        """ FAST LOOP (100 Hz): Cascade Control (Angle -> Rate) และ Motor Mixing """
        if not self.odom_ready:
            return

        curr_time = self.get_clock().now()
        dt = (curr_time - self.last_time_att).nanoseconds / 1e9
        if dt <= 0: return

        # ==========================================
        # STEP 1: Angle Loop (P-Controller)
        # ==========================================
        target_roll_rate = self.kp_angle_roll * (self.target_roll - self.curr_roll)
        target_pitch_rate = self.kp_angle_pitch * (self.target_pitch - self.curr_pitch)
        target_yaw_rate = self.kp_angle_yaw * (self.target_yaw - self.curr_yaw)

        # ==========================================
        # STEP 2: Rate Loop (PID-Controller)
        # ==========================================
        error_roll_rate = target_roll_rate - self.curr_roll_rate
        error_pitch_rate = target_pitch_rate - self.curr_pitch_rate
        error_yaw_rate = target_yaw_rate - self.curr_yaw_rate
        
        # --- Anti-Windup สำหรับ Rate ---
        self.integral_roll_rate += error_roll_rate * dt
        self.integral_roll_rate = max(-self.max_i_rate, min(self.max_i_rate, self.integral_roll_rate))

        self.integral_pitch_rate += error_pitch_rate * dt
        self.integral_pitch_rate = max(-self.max_i_rate, min(self.max_i_rate, self.integral_pitch_rate))

        self.integral_yaw_rate += error_yaw_rate * dt
        self.integral_yaw_rate = max(-self.max_i_rate, min(self.max_i_rate, self.integral_yaw_rate))
        
        # Derivative
        deriv_roll_rate = (error_roll_rate - self.prev_error_roll_rate) / dt
        deriv_pitch_rate = (error_pitch_rate - self.prev_error_pitch_rate) / dt
        deriv_yaw_rate = (error_yaw_rate - self.prev_error_yaw_rate) / dt
        
        # Output Commands (Torque)
        roll_cmd = (self.kp_rate_roll * error_roll_rate) + (self.ki_rate_roll * self.integral_roll_rate) + (self.kd_rate_roll * deriv_roll_rate)
        pitch_cmd = (self.kp_rate_pitch * error_pitch_rate) + (self.ki_rate_pitch * self.integral_pitch_rate) + (self.kd_rate_pitch * deriv_pitch_rate)
        yaw_cmd = (self.kp_rate_yaw * error_yaw_rate) + (self.ki_rate_yaw * self.integral_yaw_rate) + (self.kd_rate_yaw * deriv_yaw_rate)

        # ==========================================
        # STEP 3: Motor Mixing 
        # ==========================================
        m0 = self.total_thrust + roll_cmd + pitch_cmd - yaw_cmd
        m1 = self.total_thrust - roll_cmd - pitch_cmd - yaw_cmd
        m2 = self.total_thrust - roll_cmd + pitch_cmd + yaw_cmd
        m3 = self.total_thrust + roll_cmd - pitch_cmd + yaw_cmd

        # --- Motor Saturation (จำกัดค่าให้ไม่ติดลบ และไม่เกินรอบสูงสุด) ---
        m0 = max(0.0, min(self.max_motor_vel, float(m0)))
        m1 = max(0.0, min(self.max_motor_vel, float(m1)))
        m2 = max(0.0, min(self.max_motor_vel, float(m2)))
        m3 = max(0.0, min(self.max_motor_vel, float(m3)))

        # --- ส่งคำสั่งไปยัง Gazebo ---
        msg_out = Actuators()
        msg_out.header.stamp = self.get_clock().now().to_msg()
        msg_out.velocity = [m0, m1, m2, m3]
        msg_out.position = []
        msg_out.normalized = []
        
        self.motor_pub.publish(msg_out)

        # อัปเดตค่า Previous Error 
        self.prev_error_roll_rate = error_roll_rate
        self.prev_error_pitch_rate = error_pitch_rate
        self.prev_error_yaw_rate = error_yaw_rate
        self.last_time_att = curr_time

def main(args=None):
    rclpy.init(args=args)
    node = HoverController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()