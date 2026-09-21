#!/usr/bin/env python3
"""
Interactive Keyboard Teleoperation Node for Go2 Quadruped.

Provides:
- WASD + QE omnidirectional movement controls
- Stand (X), Sit (C), Balance (B), Jump (J), ESTOP (P/ESC)
- Body height adjustment (R: up, F: down)
- Real-time Terminal Dashboard with live FSM state, velocities, and IMU feedback.
"""

import math
import os
import select
import sys
import termios
import time
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String, UInt8


class KeyboardTeleopNode(Node):
    """Go2 Keyboard Controller and Terminal HUD."""

    def __init__(self):
        super().__init__('keyboard_teleop')

        self.declare_parameter('robot_namespace', 'robot_1')
        self.robot_ns = self.get_parameter('robot_namespace').get_parameter_value().string_value

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.ns_cmd_vel_pub = self.create_publisher(Twist, f'/{self.robot_ns}/cmd_vel', 10)
        self.fsm_cmd_pub = self.create_publisher(String, '/go2/fsm_cmd', 10)
        self.control_mode_pub = self.create_publisher(UInt8, f'/{self.robot_ns}/control/mode', 10)
        self.posture_cmd_pub = self.create_publisher(Twist, '/go2/posture_cmd', 10)

        # Feedback subscribers
        self.fsm_state_sub = self.create_subscription(
            String, '/go2/fsm_state', self.fsm_state_cb, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_cb, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu', self.imu_cb, 10)

        # State variables
        self.current_fsm_state = "STAND"
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_wz = 0.0
        self.target_height = 0.28  # nominal height (m)
        self.target_roll = 0.0
        self.target_pitch = 0.0

        self.actual_speed = 0.0
        self.actual_pos = [0.0, 0.0, 0.0]
        self.actual_rpy = [0.0, 0.0, 0.0]

        # Speed presets
        self.speed_mode = "NORMAL"
        self.vx_step = 0.3
        self.vy_step = 0.2
        self.wz_step = 0.5
        self.turbo = False

        # Terminal settings
        self.settings = termios.tcgetattr(sys.stdin)

        # Timer for publishing cmd_vel at 20 Hz
        self.pub_timer = self.create_timer(0.05, self.publish_velocity)
        # Timer for updating screen dashboard at 10 Hz
        self.ui_timer = self.create_timer(0.1, self.render_dashboard)

        self.get_logger().info("Keyboard Teleop Node Initialized.")

    def fsm_state_cb(self, msg: String):
        self.current_fsm_state = msg.data

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        self.actual_pos = [p.x, p.y, p.z]
        self.actual_speed = math.hypot(v.x, v.y)

    def imu_cb(self, msg: Imu):
        q = msg.orientation
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.actual_rpy = [roll, pitch, yaw]

    def publish_velocity(self):
        cmd = Twist()
        scale = 1.5 if self.turbo else 1.0
        cmd.linear.x = self.target_vx * scale
        cmd.linear.y = self.target_vy * scale
        cmd.angular.z = self.target_wz * scale
        self.cmd_vel_pub.publish(cmd)
        self.ns_cmd_vel_pub.publish(cmd)

    def send_fsm(self, cmd_str: str, quad_mode: int = None):
        msg = String()
        msg.data = cmd_str
        self.fsm_cmd_pub.publish(msg)

        if quad_mode is not None:
            m = UInt8()
            m.data = quad_mode
            self.control_mode_pub.publish(m)

        posture = Twist()
        posture.linear.z = float(self.target_height)
        posture.angular.x = float(self.target_roll)
        posture.angular.y = float(self.target_pitch)
        self.posture_cmd_pub.publish(posture)

    def render_dashboard(self):
        """Render real-time status HUD in terminal."""
        # ANSI clear screen and move cursor home
        sys.stdout.write("\033[H\033[J")
        scale = 1.5 if self.turbo else 1.0

        hud = [
            "===================================================================",
            "                 🐕 Unitree Go2 控制儀表板 (WASD 鍵盤操控)        ",
            "===================================================================",
            f" [FSM 狀態]     : \033[1;32m{self.current_fsm_state:<12}\033[0m | 速度模式: \033[1;33m{self.speed_mode}\033[0m",
            f" [目標速度]     : vx: {self.target_vx*scale:+.2f} m/s | vy: {self.target_vy*scale:+.2f} m/s | wz: {self.target_wz*scale:+.2f} rad/s",
            f" [實測運動]     : 線速度: {self.actual_speed:.2f} m/s | 位置: X:{self.actual_pos[0]:+.2f} Y:{self.actual_pos[1]:+.2f} Z:{self.actual_pos[2]:.2f}m",
            f" [軀幹高度]     : {self.target_height:.2f} m (範圍: 0.15 ~ 0.38m)",
            f" [機載 IMU]     : Roll: {math.degrees(self.actual_rpy[0]):+5.1f}° | Pitch: {math.degrees(self.actual_rpy[1]):+5.1f}° | Yaw: {math.degrees(self.actual_rpy[2]):+5.1f}°",
            "-------------------------------------------------------------------",
            " [操控按鍵說明]",
            "   W / S : 前進 / 後退 (vx)          Q / E : 原地左轉 / 右轉 (wz)",
            "   A / D : 左平移 / 右平移 (vy)      SPACE : 停止移動 (煞停平衡)",
            "   X     : 站立 (STAND_UP)          C     : 坐下折疊 (SIT_DOWN)",
            "   B     : 平衡站立 (BALANCE)       J     : 執行跳躍 (JUMP)",
            "   R / F : 軀幹高度 增 / 減         P/ESC : 緊急停止 (ESTOP)",
            "   1 / 2 : 普通 / Turbo 速度切換    Ctrl+C: 結束程式",
            "===================================================================",
        ]
        sys.stdout.write("\n".join(hud) + "\n")
        sys.stdout.flush()

    def process_key(self, key):
        """Handle key press events."""
        if key in ['w', 'W']:
            self.target_vx = min(0.6, self.target_vx + self.vx_step)
        elif key in ['s', 'S']:
            self.target_vx = max(-0.4, self.target_vx - self.vx_step)
        elif key in ['a', 'A']:
            self.target_vy = min(0.4, self.target_vy + self.vy_step)
        elif key in ['d', 'D']:
            self.target_vy = max(-0.4, self.target_vy - self.vy_step)
        elif key in ['q', 'Q']:
            self.target_wz = min(1.0, self.target_wz + self.wz_step)
        elif key in ['e', 'E']:
            self.target_wz = max(-1.0, self.target_wz - self.wz_step)
        elif key == ' ':
            self.target_vx = 0.0
            self.target_vy = 0.0
            self.target_wz = 0.0
            self.send_fsm("BALANCE")
        elif key in ['x', 'X']:
            self.send_fsm("STAND_UP", quad_mode=1)
        elif key in ['c', 'C']:
            self.target_vx = 0.0
            self.target_vy = 0.0
            self.target_wz = 0.0
            self.send_fsm("SIT_DOWN", quad_mode=0)
        elif key in ['b', 'B']:
            self.target_vx = 0.0
            self.target_vy = 0.0
            self.target_wz = 0.0
            self.send_fsm("BALANCE")
        elif key in ['j', 'J']:
            self.send_fsm("JUMP")
        elif key in ['p', 'P', '\x1b']:  # P or ESC
            self.target_vx = 0.0
            self.target_vy = 0.0
            self.target_wz = 0.0
            self.send_fsm("ESTOP", quad_mode=4)
        elif key in ['r', 'R']:
            self.target_height = min(0.38, round(self.target_height + 0.02, 2))
            self.send_fsm("POSTURE")
        elif key in ['f', 'F']:
            self.target_height = max(0.15, round(self.target_height - 0.02, 2))
            self.send_fsm("POSTURE")
        elif key == '1':
            self.turbo = False
            self.speed_mode = "NORMAL"
        elif key == '2':
            self.turbo = True
            self.speed_mode = "TURBO"

    def restore_terminal(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleopNode()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            key = getKey(node.settings)
            if key == '\x03':  # Ctrl+C
                break
            elif key != '':
                node.process_key(key)
    except Exception as e:
        print(e)
    finally:
        node.restore_terminal()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
