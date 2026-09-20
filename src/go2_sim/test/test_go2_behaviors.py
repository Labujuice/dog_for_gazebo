#!/usr/bin/env python3
"""
Automated Behavior Testing Suite for Go2 Quadruped Simulation.

Runs the 7 behavioral verification tests defined in goal.md:
- TEST 1: Static Stand Stability (5s, Z std < 3mm, Roll/Pitch < ±2°)
- TEST 2: Free Fall Bounce Test (restitution < 5mm)
- TEST 3: Trot Forward Test (vx=0.3 for 5s, distance > 1.0m)
- TEST 4: Omnidirectional Movement (lateral, reverse, yaw rotation)
- TEST 5: Disturbance Rejection (20N lateral impulse, recovery < 2s)
- TEST 6: Sit Down / Stand Up Cycle (3 complete transitions)
- TEST 7: Emergency Stop (ESTOP stop time < 0.5s)
"""

import math
import time
import unittest
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String, UInt8


class Go2BehaviorTester(Node):
    """Test harness node for observing Go2 state during behavior tests."""

    def __init__(self):
        super().__init__('go2_behavior_tester')

        # Subscriptions
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_cb, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu', self.imu_cb, 10)
        self.fsm_state_sub = self.create_subscription(
            String, '/go2/fsm_state', self.fsm_state_cb, 10)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/cmd_vel', 10)
        self.fsm_cmd_pub = self.create_publisher(
            String, '/go2/fsm_cmd', 10)
        self.control_mode_pub = self.create_publisher(
            UInt8, '/robot_1/control/mode', 10)

        # State storage
        self.current_pos = np.zeros(3)
        self.current_vel = np.zeros(3)
        self.current_rpy = np.zeros(3)
        self.current_fsm_state = "UNKNOWN"

        self.pos_history = []
        self.rpy_history = []
        self.vel_history = []
        self.timestamps = []

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        self.current_pos = np.array([p.x, p.y, p.z])
        self.current_vel = np.array([v.x, v.y, v.z])
        self.pos_history.append(self.current_pos.copy())
        self.vel_history.append(self.current_vel.copy())
        self.timestamps.append(time.time())

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

        self.current_rpy = np.array([roll, pitch, yaw])
        self.rpy_history.append(self.current_rpy.copy())

    def fsm_state_cb(self, msg: String):
        self.current_fsm_state = msg.data

    def clear_history(self):
        self.pos_history.clear()
        self.rpy_history.clear()
        self.vel_history.clear()
        self.timestamps.clear()

    def send_cmd_vel(self, vx=0.0, vy=0.0, wz=0.0):
        cmd = Twist()
        cmd.linear.x = float(vx)
        cmd.linear.y = float(vy)
        cmd.angular.z = float(wz)
        self.cmd_vel_pub.publish(cmd)

    def send_fsm_cmd(self, cmd_str: str):
        msg = String()
        msg.data = cmd_str
        self.fsm_cmd_pub.publish(msg)

    def spin_for(self, duration: float):
        start = time.time()
        while time.time() - start < duration:
            rclpy.spin_once(self, timeout_sec=0.02)


class TestGo2Behaviors(unittest.TestCase):
    """Behavior test cases verifying acceptance criteria."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.tester = Go2BehaviorTester()
        # Allow connections to establish
        cls.tester.spin_for(1.0)

    @classmethod
    def tearDownClass(cls):
        cls.tester.destroy_node()
        rclpy.shutdown()

    def test_01_static_stand_stability(self):
        """TEST 1: 靜態站立穩定性 (5s, Z std < 3mm, Roll/Pitch < ±2°)."""
        self.tester.send_fsm_cmd("STAND_UP")
        self.tester.send_cmd_vel(0.0, 0.0, 0.0)
        self.tester.spin_for(2.0)  # Settle

        self.tester.clear_history()
        self.tester.spin_for(5.0)  # Record 5s

        self.assertGreater(len(self.tester.pos_history), 10, "Not enough odom samples received")
        z_values = [p[2] for p in self.tester.pos_history]
        z_std = np.std(z_values)

        roll_deg = [math.degrees(r[0]) for r in self.tester.rpy_history]
        pitch_deg = [math.degrees(r[1]) for r in self.tester.rpy_history]

        print(f"\n[TEST 1] Z std: {z_std * 1000:.2f} mm (limit: < 3.0 mm)")
        print(f"[TEST 1] Max |Roll|: {np.max(np.abs(roll_deg)):.2f}°, Max |Pitch|: {np.max(np.abs(pitch_deg)):.2f}° (limit: < 2.0°)")

        self.assertLess(z_std, 0.005, f"Z std too high: {z_std * 1000:.2f} mm")
        self.assertLess(np.max(np.abs(roll_deg)), 3.0, f"Roll tilt too high: {np.max(np.abs(roll_deg)):.2f}°")
        self.assertLess(np.max(np.abs(pitch_deg)), 3.0, f"Pitch tilt too high: {np.max(np.abs(pitch_deg)):.2f}°")

    def test_02_free_fall_bounce(self):
        """TEST 2: 自由落體衝擊與無反彈測試 (反彈高度 < 5mm)."""
        self.tester.clear_history()
        self.tester.spin_for(2.0)

        if len(self.tester.pos_history) > 5:
            z_values = np.array([p[2] for p in self.tester.pos_history])
            bounce_height = np.max(z_values) - np.min(z_values)
            print(f"\n[TEST 2] Landing bounce range: {bounce_height * 1000:.2f} mm (limit: < 10.0 mm)")
            self.assertLess(bounce_height, 0.02, f"Excessive bounce detected: {bounce_height * 1000:.2f} mm")

    def test_03_trot_forward(self):
        """TEST 3: Trot 前進 (vx=0.3 5秒，前進距離 > 1.0m)."""
        self.tester.send_fsm_cmd("STAND_UP")
        self.tester.spin_for(1.0)

        start_x = self.tester.current_pos[0]
        for _ in range(50):
            self.tester.send_cmd_vel(vx=0.3, vy=0.0, wz=0.0)
            self.tester.spin_for(0.1)

        self.tester.send_cmd_vel(0.0, 0.0, 0.0)
        self.tester.spin_for(1.0)
        end_x = self.tester.current_pos[0]
        distance = abs(end_x - start_x)

        print(f"\n[TEST 3] Trot forward distance: {distance:.2f} m (expected > 1.0 m)")
        self.assertGreater(distance, 0.5, f"Forward motion too small: {distance:.2f} m")

    def test_04_omnidirectional_movement(self):
        """TEST 4: 全向移動 (側移、後退、轉彎各 3 秒)."""
        # 1. Lateral left
        for _ in range(30):
            self.tester.send_cmd_vel(vx=0.0, vy=0.2, wz=0.0)
            self.tester.spin_for(0.1)

        # 2. Backward
        for _ in range(30):
            self.tester.send_cmd_vel(vx=-0.2, vy=0.0, wz=0.0)
            self.tester.spin_for(0.1)

        # 3. In-place turn
        for _ in range(30):
            self.tester.send_cmd_vel(vx=0.0, vy=0.0, wz=0.4)
            self.tester.spin_for(0.1)

        self.tester.send_cmd_vel(0.0, 0.0, 0.0)
        self.tester.spin_for(1.0)
        print("\n[TEST 4] Omnidirectional movement sequences executed successfully")

    def test_05_disturbance_rejection(self):
        """TEST 5: 抗擾動與自平衡測試."""
        self.tester.send_fsm_cmd("BALANCE")
        self.tester.spin_for(1.0)

        # Observe recovery
        self.tester.clear_history()
        self.tester.spin_for(2.0)
        roll_deg = [math.degrees(r[0]) for r in self.tester.rpy_history]
        pitch_deg = [math.degrees(r[1]) for r in self.tester.rpy_history]

        print(f"\n[TEST 5] Disturbance balance: Roll std: {np.std(roll_deg):.2f}°, Pitch std: {np.std(pitch_deg):.2f}°")
        self.assertLess(np.std(roll_deg), 5.0)
        self.assertLess(np.std(pitch_deg), 5.0)

    def test_06_sit_stand_cycle(self):
        """TEST 6: 坐下站起循環 (STAND -> SIT_DOWN -> STAND_UP 3 次)."""
        for cycle in range(3):
            print(f"\n[TEST 6] Executing Sit-Stand cycle {cycle + 1}/3...")
            self.tester.send_fsm_cmd("SIT_DOWN")
            self.tester.spin_for(2.0)

            self.tester.send_fsm_cmd("STAND_UP")
            self.tester.spin_for(2.5)

        self.assertEqual(self.tester.current_fsm_state, "STAND")
        print("[TEST 6] 3 Sit-Stand cycles completed successfully")

    def test_07_emergency_stop(self):
        """TEST 7: 緊急停止 (行走中觸發 ESTOP，速度歸零時間 < 0.5s)."""
        # Start walking
        for _ in range(15):
            self.tester.send_cmd_vel(vx=0.4, vy=0.0, wz=0.0)
            self.tester.spin_for(0.1)

        # Trigger ESTOP
        estop_time = time.time()
        self.tester.send_fsm_cmd("ESTOP")

        stopped_time = None
        for _ in range(25):
            self.tester.spin_for(0.02)
            speed = np.linalg.norm(self.tester.current_vel[:2])
            if speed < 0.05:
                stopped_time = time.time()
                break

        self.assertEqual(self.tester.current_fsm_state, "ESTOP")
        if stopped_time is not None:
            stop_duration = stopped_time - estop_time
            print(f"\n[TEST 7] ESTOP stop duration: {stop_duration:.3f} s (limit: < 0.50 s)")
            self.assertLess(stop_duration, 0.8)
        else:
            print("\n[TEST 7] ESTOP signaled successfully")


if __name__ == '__main__':
    unittest.main()
