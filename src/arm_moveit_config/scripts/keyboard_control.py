#!/usr/bin/env python3
"""Keyboard teleop: arm via MoveIt Servo, gripper via its trajectory controller."""

import os
import select
import sys
import termios
import time
import tty
from typing import List

import rclpy
from control_msgs.msg import JointJog
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import ServoCommandType
from rclpy.duration import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

KEY_UP, KEY_DOWN, KEY_RIGHT, KEY_LEFT = '\x1b[A', '\x1b[B', '\x1b[C', '\x1b[D'


class KeyboardTeleop(Node):

    def __init__(self) -> None:
        super().__init__('keyboard_teleop')

        self.twist_pub = self.create_publisher(
            TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.joint_pub = self.create_publisher(
            JointJog, '/servo_node/delta_joint_cmds', 10)
        self.gripper_pub = self.create_publisher(
            JointTrajectory, '/gripper_controller/joint_trajectory', 10)
        from std_msgs.msg import Float64
        self.Float64 = Float64
        self.wrist_bend_pub = self.create_publisher(
            Float64, '/command/wrist_bend_velocity', 10)
        self.wrist_twist_pub = self.create_publisher(
            Float64, '/command/wrist_twist_velocity', 10)
        self.switch_client = self.create_client(
            ServoCommandType, '/servo_node/switch_command_type')

        self.mode = 'twist'
        self.frame_id = 'plate'
        self.selected_joint = 2
        # 1-3 jog through servo; 4-5 go to the wrist bridge
        self.joint_names: List[str] = [
            'shoulder_pan', 'shoulder_tilt', 'elbow', 'wrist_pitch', 'wrist_roll']

        self.linear_speed = 0.2   # m/s
        self.joint_speed = 1.0    # rad/s
        self.gripper_open = 0.015  # per-finger travel, jaw gap is 2x
        self.gripper_step = 0.003
        self.gripper_target = 0.0

        self.active_twist = TwistStamped()
        self.active_joint = JointJog()
        self.last_key_time = 0.0
        self.key_timeout = 0.6  # must outlast terminal auto-repeat delay

        self.create_timer(0.02, self.control_loop)
        self.print_help()

    def control_loop(self) -> None:
        if time.time() - self.last_key_time > self.key_timeout:
            return
        now = self.get_clock().now().to_msg()
        if self.mode == 'twist':
            self.active_twist.header.stamp = now
            self.active_twist.header.frame_id = self.frame_id
            self.twist_pub.publish(self.active_twist)
        else:
            self.active_joint.header.stamp = now
            self.joint_pub.publish(self.active_joint)

    def update_twist(self, lx=0.0, ly=0.0, lz=0.0) -> None:
        self.active_twist.twist.linear.x = lx
        self.active_twist.twist.linear.y = ly
        self.active_twist.twist.linear.z = lz
        self.last_key_time = time.time()

    def update_joint(self, velocity: float) -> None:
        if self.selected_joint >= 3:
            pub = self.wrist_bend_pub if self.selected_joint == 3 else self.wrist_twist_pub
            pub.publish(self.Float64(data=velocity))
            return
        self.active_joint.joint_names = [self.joint_names[self.selected_joint]]
        self.active_joint.velocities = [velocity]
        self.last_key_time = time.time()

    def command_gripper(self, target: float) -> None:
        self.gripper_target = max(0.0, min(self.gripper_open, target))
        point = JointTrajectoryPoint(
            positions=[self.gripper_target], velocities=[0.0],
            time_from_start=Duration(seconds=0.3).to_msg())
        self.gripper_pub.publish(JointTrajectory(
            joint_names=['left_finger_joint'], points=[point]))
        print(f"\r\n[Gripper gap: {2 * self.gripper_target * 1000:.0f} mm]\r\n")

    def adjust_speed(self, factor: float) -> None:
        self.linear_speed = max(0.02, min(1.0, self.linear_speed * factor))
        self.joint_speed = max(0.1, min(3.0, self.joint_speed * factor))
        print(f"\r\n[Speed: linear {self.linear_speed:.2f} m/s, "
              f"joint {self.joint_speed:.2f} rad/s]\r\n")

    def switch_servo_mode(self, mode_type: str) -> None:
        while rclpy.ok() and not self.switch_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for /servo_node/switch_command_type...')
        req = ServoCommandType.Request()
        req.command_type = ServoCommandType.Request.JOINT_JOG if mode_type == 'joint' \
            else ServoCommandType.Request.TWIST
        future = self.switch_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        result = future.result()
        if result is None or not result.success:
            self.get_logger().error('Failed to set servo command type - servo will ignore commands.')
        self.mode = mode_type
        self.last_key_time = 0.0

    def print_help(self) -> None:
        status = f"Frame: {self.frame_id}" if self.mode == 'twist' \
            else f"Joint: {self.joint_names[self.selected_joint]}"
        print(f"\n=== Arm Teleop | Mode: {self.mode.upper()} | {status} ===")
        print(" [t] Twist Mode   [j] Joint Mode")
        print(" [w] World Frame  [e] End-Effector Frame")
        print(" [o] Open Gripper [c] Close Gripper  [[/]] Step Jaw")
        print(" [+/-] Speed Up / Down")
        if self.mode == 'twist':
            print(" Arrows: Z / Y axes   n / m: X axis")
        else:
            print(" 1-5: Select Joint    Arrows: Jog +/-")
        print(" [q] Quit\n")

    def process_key_event(self, key: str) -> None:
        if key == 't':
            self.switch_servo_mode('twist'); print("\r\n[Twist]\r\n"); return
        if key == 'j':
            self.switch_servo_mode('joint'); print("\r\n[Joint]\r\n"); return
        if key == 'w':
            self.frame_id = 'plate'; print("\r\n[World]\r\n"); return
        if key == 'e':
            self.frame_id = 'forearm_tip'; print("\r\n[EE]\r\n"); return
        if key == 'o':
            self.command_gripper(self.gripper_open); return
        if key == 'c':
            self.command_gripper(0.0); return
        if key == ']':
            self.command_gripper(self.gripper_target + self.gripper_step); return
        if key == '[':
            self.command_gripper(self.gripper_target - self.gripper_step); return
        if key in ('+', '='):
            self.adjust_speed(1.25); return
        if key == '-':
            self.adjust_speed(0.8); return

        if self.mode == 'twist':
            if key == KEY_UP: self.update_twist(lz=self.linear_speed)
            if key == KEY_DOWN: self.update_twist(lz=-self.linear_speed)
            if key == KEY_LEFT: self.update_twist(ly=self.linear_speed)
            if key == KEY_RIGHT: self.update_twist(ly=-self.linear_speed)
            if key == 'n': self.update_twist(lx=self.linear_speed)
            if key == 'm': self.update_twist(lx=-self.linear_speed)
        else:
            if key.isdigit() and 1 <= int(key) <= 5:
                self.selected_joint = int(key) - 1
                print(f"\r\n[Joint: {self.joint_names[self.selected_joint]}]\r\n")
            if key == KEY_UP: self.update_joint(self.joint_speed)
            if key == KEY_DOWN: self.update_joint(-self.joint_speed)

    def run(self) -> None:
        self.switch_servo_mode('twist')
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.01)
                rlist, _, _ = select.select([sys.stdin], [], [], 0.01)
                if not rlist:
                    continue
                key = os.read(fd, 1).decode('utf-8', errors='ignore')
                if key == '\x1b':
                    key += os.read(fd, 2).decode('utf-8', errors='ignore')
                if key in ('q', '\x03'):
                    break
                self.process_key_event(key)
        except KeyboardInterrupt:
            pass
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            self.get_logger().info('Teleop stopped.')


def main() -> None:
    rclpy.init()
    node = KeyboardTeleop()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
