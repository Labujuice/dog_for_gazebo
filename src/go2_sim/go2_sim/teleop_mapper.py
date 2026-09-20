#!/usr/bin/env python3
"""
Teleop Mapper Node for Go2 Quadruped.

Maps PS4 DualShock 4 controller inputs and external ROS 2 commands to:
- quad-sdk control/mode (0: SIT, 1: READY/STAND, 2: SIT_TO_READY, 3: READY_TO_SIT, 4: SAFETY)
- Velocity commands (/cmd_vel, /robot_1/cmd_vel)
- FSM state transitions (STAND_UP, SIT_DOWN, BALANCE, JUMP, ESTOP, RECOVERY_STAND)
- Posture adjustments (height, roll, pitch)
- Topic bridging and telemetry reporting
"""

import math
import time
import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, Imu, JointState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String, UInt8, Header


class TeleopMapperNode(Node):
    """PS4 Joy to quad-sdk control stack and FSM mapper node."""

    def __init__(self):
        super().__init__('teleop_mapper')

        # Parameters
        self.declare_parameter('robot_namespace', 'robot_1')
        self.declare_parameter('enable_button', 4)       # L1
        self.declare_parameter('turbo_button', 5)        # R1
        self.declare_parameter('stand_button', 0)        # X
        self.declare_parameter('balance_button', 1)      # Circle
        self.declare_parameter('jump_button', 2)         # Triangle
        self.declare_parameter('sit_button', 3)          # Square
        self.declare_parameter('estop_button', 9)        # OPTIONS
        self.declare_parameter('axis_linear_x', 1)       # Left stick Y (up/down)
        self.declare_parameter('axis_linear_y', 0)       # Left stick X (left/right)
        self.declare_parameter('axis_angular_z', 3)      # Right stick X (left/right)
        self.declare_parameter('axis_height_down', 2)    # L2
        self.declare_parameter('axis_height_up', 5)      # R2
        self.declare_parameter('axis_dpad_x', 6)         # D-pad left/right (roll)
        self.declare_parameter('axis_dpad_y', 7)         # D-pad up/down (pitch)

        self.declare_parameter('max_vx', 0.6)
        self.declare_parameter('max_vy', 0.4)
        self.declare_parameter('max_wz', 1.0)
        self.declare_parameter('turbo_scale', 1.5)
        self.declare_parameter('auto_recovery', True)

        self.robot_ns = self.get_parameter('robot_namespace').get_parameter_value().string_value
        self.enable_btn = self.get_parameter('enable_button').get_parameter_value().integer_value
        self.turbo_btn = self.get_parameter('turbo_button').get_parameter_value().integer_value
        self.stand_btn = self.get_parameter('stand_button').get_parameter_value().integer_value
        self.balance_btn = self.get_parameter('balance_button').get_parameter_value().integer_value
        self.jump_btn = self.get_parameter('jump_button').get_parameter_value().integer_value
        self.sit_btn = self.get_parameter('sit_button').get_parameter_value().integer_value
        self.estop_btn = self.get_parameter('estop_button').get_parameter_value().integer_value

        self.axis_vx = self.get_parameter('axis_linear_x').get_parameter_value().integer_value
        self.axis_vy = self.get_parameter('axis_linear_y').get_parameter_value().integer_value
        self.axis_wz = self.get_parameter('axis_angular_z').get_parameter_value().integer_value
        self.axis_h_down = self.get_parameter('axis_height_down').get_parameter_value().integer_value
        self.axis_h_up = self.get_parameter('axis_height_up').get_parameter_value().integer_value
        self.axis_dpad_x = self.get_parameter('axis_dpad_x').get_parameter_value().integer_value
        self.axis_dpad_y = self.get_parameter('axis_dpad_y').get_parameter_value().integer_value

        self.max_vx = self.get_parameter('max_vx').get_parameter_value().double_value
        self.max_vy = self.get_parameter('max_vy').get_parameter_value().double_value
        self.max_wz = self.get_parameter('max_wz').get_parameter_value().double_value
        self.turbo_scale = self.get_parameter('turbo_scale').get_parameter_value().double_value
        self.auto_recovery = self.get_parameter('auto_recovery').get_parameter_value().bool_value

        # FSM State variables
        # States: "SIT", "STAND_UP", "STAND", "WALK", "SIT_DOWN", "JUMP", "FALLEN", "RECOVERY_STAND", "ESTOP"
        self.fsm_state = "STAND"
        self.target_height = 0.28   # Nominal height (0.15 ~ 0.38m)
        self.target_roll = 0.0      # rad
        self.target_pitch = 0.0     # rad

        # Previous button states for edge detection
        self.prev_buttons = []
        self.jump_step = 0
        self.jump_start_time = 0.0
        self.recovery_step = 0
        self.recovery_start_time = 0.0

        # Latest IMU for fall detection
        self.latest_roll = 0.0
        self.latest_pitch = 0.0

        # Publishers
        self.control_mode_pub = self.create_publisher(
            UInt8, f'/{self.robot_ns}/control/mode', 10)
        self.cmd_vel_pub = self.create_publisher(
            Twist, f'/{self.robot_ns}/cmd_vel', 10)
        self.global_cmd_vel_pub = self.create_publisher(
            Twist, '/cmd_vel', 10)
        self.fsm_state_pub = self.create_publisher(
            String, '/go2/fsm_state', 10)
        self.fsm_cmd_pub = self.create_publisher(
            String, '/go2/fsm_cmd', 10)
        self.posture_cmd_pub = self.create_publisher(
            Twist, '/go2/posture_cmd', 10)
        self.telemetry_pub = self.create_publisher(
            String, '/go2/telemetry', 10)

        # Standard ROS 2 relay publishers (for external integration)
        self.odom_relay_pub = self.create_publisher(Odometry, '/odom', 10)
        self.imu_relay_pub = self.create_publisher(Imu, '/imu', 10)
        self.joint_states_relay_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Subscribers
        self.joy_sub = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10)
        self.external_cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.external_cmd_vel_callback, 10)
        self.external_fsm_cmd_sub = self.create_subscription(
            String, '/go2/fsm_cmd', self.external_fsm_cmd_callback, 10)
        self.external_posture_sub = self.create_subscription(
            Twist, '/go2/posture_cmd', self.external_posture_callback, 10)

        # Telemetry / State feedback subscribers
        self.imu_sub = self.create_subscription(
            Imu, f'/{self.robot_ns}/imu', self.imu_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, f'/{self.robot_ns}/odom', self.odom_callback, 10)
        self.joints_sub = self.create_subscription(
            JointState, f'/{self.robot_ns}/state/joints', self.joints_callback, 10)

        # Timer for FSM state machine updates & telemetry (50 Hz)
        self.timer = self.create_timer(0.02, self.fsm_timer_callback)
        self.get_logger().info('TeleopMapperNode started for namespace: %s' % self.robot_ns)

    def is_button_pressed(self, buttons, index):
        """Check if button transitioned from unpressed (0) to pressed (1)."""
        if index < 0 or index >= len(buttons):
            return False
        curr = bool(buttons[index])
        prev = bool(self.prev_buttons[index]) if index < len(self.prev_buttons) else False
        return curr and not prev

    def get_axis_value(self, axes, index):
        if index < 0 or index >= len(axes):
            return 0.0
        return float(axes[index])

    def joy_callback(self, msg: Joy):
        """Process joystick inputs."""
        buttons = msg.buttons
        axes = msg.axes

        # 1. ESTOP (OPTIONS button 9)
        if self.is_button_pressed(buttons, self.estop_btn):
            self.execute_command("ESTOP")

        # 2. STAND_UP (X button 0)
        elif self.is_button_pressed(buttons, self.stand_btn):
            self.execute_command("STAND_UP")

        # 3. SIT_DOWN (Square button 3)
        elif self.is_button_pressed(buttons, self.sit_btn):
            self.execute_command("SIT_DOWN")

        # 4. BALANCE (Circle button 1)
        elif self.is_button_pressed(buttons, self.balance_btn):
            self.execute_command("BALANCE")

        # 5. JUMP (Triangle button 2)
        elif self.is_button_pressed(buttons, self.jump_btn):
            self.execute_command("JUMP")

        # 6. Posture adjustments (L2 / R2 and D-pad)
        # Height: R2 increases height, L2 decreases height
        l2_raw = self.get_axis_value(axes, self.axis_h_down)  # 1.0 unpressed, -1.0 pressed
        r2_raw = self.get_axis_value(axes, self.axis_h_up)
        # Convert -1..1 to 0..1 depression
        l2_press = max(0.0, (1.0 - l2_raw) / 2.0) if len(axes) > self.axis_h_down else 0.0
        r2_press = max(0.0, (1.0 - r2_raw) / 2.0) if len(axes) > self.axis_h_up else 0.0
        dh = (r2_press - l2_press) * 0.005
        self.target_height = max(0.15, min(0.38, self.target_height + dh))

        # Pitch / Roll from D-pad
        dpad_x = self.get_axis_value(axes, self.axis_dpad_x)
        dpad_y = self.get_axis_value(axes, self.axis_dpad_y)
        self.target_roll = max(-0.35, min(0.35, self.target_roll + dpad_x * 0.01))
        self.target_pitch = max(-0.35, min(0.35, self.target_pitch + dpad_y * 0.01))

        # 7. Velocity control (left stick for translation, right stick for yaw)
        enable_pressed = (len(buttons) > self.enable_btn and bool(buttons[self.enable_btn]))
        turbo_pressed = (len(buttons) > self.turbo_btn and bool(buttons[self.turbo_btn]))

        if enable_pressed and self.fsm_state in ["STAND", "WALK"]:
            scale = self.turbo_scale if turbo_pressed else 1.0
            vx = self.get_axis_value(axes, self.axis_vx) * self.max_vx * scale
            vy = self.get_axis_value(axes, self.axis_vy) * self.max_vy * scale
            wz = self.get_axis_value(axes, self.axis_wz) * self.max_wz * scale

            cmd = Twist()
            cmd.linear.x = vx
            cmd.linear.y = vy
            cmd.angular.z = wz
            self.cmd_vel_pub.publish(cmd)

            if abs(vx) > 0.01 or abs(vy) > 0.01 or abs(wz) > 0.01:
                self.fsm_state = "WALK"
            else:
                self.fsm_state = "STAND"
        elif not enable_pressed and self.fsm_state == "WALK":
            # Send stop
            cmd = Twist()
            self.cmd_vel_pub.publish(cmd)
            self.fsm_state = "STAND"

        self.prev_buttons = list(buttons)

    def execute_command(self, cmd_str: str):
        """Execute FSM transition commands."""
        cmd_str = cmd_str.upper()
        self.get_logger().info(f"FSM Command Received: {cmd_str}")

        # Publish FSM command feedback
        msg = String()
        msg.data = cmd_str
        self.fsm_cmd_pub.publish(msg)

        if cmd_str == "ESTOP":
            self.fsm_state = "ESTOP"
            # Publish safety mode to quad-sdk
            mode_msg = UInt8()
            mode_msg.data = 4  # SAFETY
            self.control_mode_pub.publish(mode_msg)
            # Publish 0 cmd_vel
            self.cmd_vel_pub.publish(Twist())

        elif cmd_str == "STAND_UP":
            self.fsm_state = "STAND_UP"
            mode_msg = UInt8()
            mode_msg.data = 1  # READY
            self.control_mode_pub.publish(mode_msg)

        elif cmd_str == "SIT_DOWN":
            self.fsm_state = "SIT_DOWN"
            mode_msg = UInt8()
            mode_msg.data = 0  # SIT
            self.control_mode_pub.publish(mode_msg)
            self.cmd_vel_pub.publish(Twist())

        elif cmd_str == "BALANCE":
            self.fsm_state = "STAND"
            self.cmd_vel_pub.publish(Twist())

        elif cmd_str == "JUMP":
            if self.fsm_state in ["STAND", "WALK"]:
                self.fsm_state = "JUMP"
                self.jump_step = 0
                self.jump_start_time = time.time()

        elif cmd_str == "RECOVERY_STAND":
            self.fsm_state = "RECOVERY_STAND"
            self.recovery_step = 0
            self.recovery_start_time = time.time()

    def external_cmd_vel_callback(self, msg: Twist):
        """Forward external /cmd_vel to robot namespace when in operational state."""
        if self.fsm_state in ["STAND", "WALK"]:
            self.cmd_vel_pub.publish(msg)
            if abs(msg.linear.x) > 0.01 or abs(msg.linear.y) > 0.01 or abs(msg.angular.z) > 0.01:
                self.fsm_state = "WALK"

    def external_fsm_cmd_callback(self, msg: String):
        """Handle external FSM commands."""
        self.execute_command(msg.data)

    def external_posture_callback(self, msg: Twist):
        """Handle external posture commands."""
        if 0.15 <= msg.linear.z <= 0.38:
            self.target_height = msg.linear.z
        if abs(msg.angular.x) <= 0.35:
            self.target_roll = msg.angular.x
        if abs(msg.angular.y) <= 0.35:
            self.target_pitch = msg.angular.y

    def imu_callback(self, msg: Imu):
        """Monitor IMU for fall detection and relay to global /imu."""
        # Calculate Roll and Pitch from quaternion
        q = msg.orientation
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        self.latest_roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1.0:
            self.latest_pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            self.latest_pitch = math.asin(sinp)

        # Check for fall (> 50 deg ~ 0.87 rad)
        fall_thresh = 50.0 * math.pi / 180.0
        if (abs(self.latest_roll) > fall_thresh or abs(self.latest_pitch) > fall_thresh) and \
           self.fsm_state not in ["FALLEN", "RECOVERY_STAND", "ESTOP"]:
            self.get_logger().warn(
                f"FALL DETECTED! Roll: {math.degrees(self.latest_roll):.1f}°, "
                f"Pitch: {math.degrees(self.latest_pitch):.1f}°")
            self.fsm_state = "FALLEN"
            if self.auto_recovery:
                self.get_logger().info("Initiating Auto Recovery...")
                self.execute_command("RECOVERY_STAND")

        # Relay to /imu
        self.imu_relay_pub.publish(msg)

    def odom_callback(self, msg: Odometry):
        """Relay odom to global /odom."""
        self.odom_relay_pub.publish(msg)

    def joints_callback(self, msg: JointState):
        """Relay joint states to global /joint_states."""
        self.joint_states_relay_pub.publish(msg)

    def fsm_timer_callback(self):
        """Periodic FSM state maintenance, behavior sequencing, and telemetry publication."""
        now = time.time()

        # Handle JUMP 4-phase sequence
        if self.fsm_state == "JUMP":
            elapsed = now - self.jump_start_time
            if self.jump_step == 0:
                # Phase 1: Crouch (0.3s)
                self.target_height = 0.16
                if elapsed > 0.3:
                    self.jump_step = 1
            elif self.jump_step == 1:
                # Phase 2: Explosive thrust (0.15s)
                self.target_height = 0.36
                if elapsed > 0.45:
                    self.jump_step = 2
            elif self.jump_step == 2:
                # Phase 3: Airborne leg tuck (0.2s)
                self.target_height = 0.22
                if elapsed > 0.65:
                    self.jump_step = 3
            elif self.jump_step == 3:
                # Phase 4: Touch down & damper (0.35s)
                self.target_height = 0.28
                if elapsed > 1.0:
                    self.fsm_state = "STAND"
                    self.get_logger().info("JUMP sequence completed successfully")

        # Handle RECOVERY_STAND 3-phase sequence
        elif self.fsm_state == "RECOVERY_STAND":
            elapsed = now - self.recovery_start_time
            if self.recovery_step == 0:
                # Step 1: Fold legs into sit pose
                mode_msg = UInt8()
                mode_msg.data = 0  # SIT
                self.control_mode_pub.publish(mode_msg)
                if elapsed > 1.5:
                    self.recovery_step = 1
            elif self.recovery_step == 1:
                # Step 2: Push upright from sit to stand
                mode_msg = UInt8()
                mode_msg.data = 1  # STAND / READY
                self.control_mode_pub.publish(mode_msg)
                if elapsed > 3.0:
                    self.recovery_step = 2
            elif self.recovery_step == 2:
                # Step 3: Settle in STAND
                self.fsm_state = "STAND"
                self.get_logger().info("RECOVERY_STAND completed successfully")

        # Publish FSM state
        state_msg = String()
        state_msg.data = self.fsm_state
        self.fsm_state_pub.publish(state_msg)

        # Publish posture command
        posture_msg = Twist()
        posture_msg.linear.z = float(self.target_height)
        posture_msg.angular.x = float(self.target_roll)
        posture_msg.angular.y = float(self.target_pitch)
        self.posture_cmd_pub.publish(posture_msg)

        # Publish telemetry JSON
        telem = {
            "fsm_state": self.fsm_state,
            "target_height": round(self.target_height, 3),
            "target_roll_deg": round(math.degrees(self.target_roll), 1),
            "target_pitch_deg": round(math.degrees(self.target_pitch), 1),
            "actual_roll_deg": round(math.degrees(self.latest_roll), 1),
            "actual_pitch_deg": round(math.degrees(self.latest_pitch), 1),
        }
        telem_msg = String()
        telem_msg.data = json.dumps(telem)
        self.telemetry_pub.publish(telem_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopMapperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
