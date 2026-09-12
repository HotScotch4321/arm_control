#!/usr/bin/env python3
"""Gamepad teleop: arm via MoveIt Servo, wrist and gripper via their own
trajectory controllers. Self-contained - no velocity bridge nodes required.

Axis and button indices come from a controller profile (config/joystick_*.yaml),
so re-binding to a different pad needs no code change. Defaults are the Xbox
(xpad) layout.

Starts LOCKED. Press the enable button with the sticks centred and the triggers
released to unlock; it re-locks on a dropped or malformed /joy.
"""

import math
from typing import Dict, List, Optional

import rclpy
from control_msgs.msg import JointJog
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import ServoCommandType
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState, Joy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

WRIST_JOINTS = ['wrist_pitch', 'wrist_roll']
WRIST_LIMITS = {'wrist_pitch': (-1.57, 1.57), 'wrist_roll': (-6.28318, 6.28318)}
GRIPPER_JOINT = 'left_finger_joint'

# Profile keys and their Xbox (xpad) defaults.
AXIS_DEFAULTS = {
    'left_x': 0, 'left_y': 1, 'left_trigger': 2,
    'right_x': 3, 'right_y': 4, 'right_trigger': 5,
}
BUTTON_DEFAULTS = {
    'enable_toggle': 3,   # Y
    'mode_toggle': 2,     # X
    'left_bumper': 4,     # LB
    'right_bumper': 5,    # RB
}


class JoystickTeleop(Node):

    def __init__(self) -> None:
        super().__init__('joystick_teleop')

        for name, default in AXIS_DEFAULTS.items():
            self.declare_parameter(f'axes.{name}', default)
        for name, default in BUTTON_DEFAULTS.items():
            self.declare_parameter(f'buttons.{name}', default)

        self.declare_parameter('max_linear_speed', 0.20)     # m/s
        self.declare_parameter('max_joint_speed', 0.75)      # rad/s
        self.declare_parameter('max_gripper_speed', 0.02)    # m/s per finger
        self.declare_parameter('gripper_open', 0.015)        # m, closed is 0.0
        self.declare_parameter('deadzone', 0.08)
        self.declare_parameter('joy_timeout', 0.25)          # s
        self.declare_parameter('mode_switch_settle_time', 0.1)  # s
        self.declare_parameter('command_frame', 'plate')

        self.axis = {n: self.get_parameter(f'axes.{n}').value for n in AXIS_DEFAULTS}
        self.button = {n: self.get_parameter(f'buttons.{n}').value
                       for n in BUTTON_DEFAULTS}

        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_joint = self.get_parameter('max_joint_speed').value
        self.max_gripper = self.get_parameter('max_gripper_speed').value
        self.gripper_open = self.get_parameter('gripper_open').value
        self.deadzone = self.get_parameter('deadzone').value
        self.joy_timeout = self.get_parameter('joy_timeout').value
        self.settle_time = self.get_parameter('mode_switch_settle_time').value
        self.frame_id = self.get_parameter('command_frame').value

        # A profile that indexes past the end of the Joy message is a
        # configuration error, so size the guard from the profile itself.
        self.min_axes = max(self.axis.values()) + 1
        self.min_buttons = max(self.button.values()) + 1

        self.twist_pub = self.create_publisher(
            TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.joint_pub = self.create_publisher(
            JointJog, '/servo_node/delta_joint_cmds', 10)
        self.wrist_pub = self.create_publisher(
            JointTrajectory, '/wrist_controller/joint_trajectory', 10)
        self.gripper_pub = self.create_publisher(
            JointTrajectory, '/gripper_controller/joint_trajectory', 10)
        self.create_subscription(Joy, '/joy', self.on_joy, 10)
        self.create_subscription(JointState, '/joint_states', self.on_joint_states, 10)
        self.switch_client = self.create_client(
            ServoCommandType, '/servo_node/switch_command_type')

        # Commanded velocities, refreshed on every Joy message.
        self.command: List[float] = [0.0] * 3
        self.shoulder_velocity = 0.0
        self.wrist_velocity: Dict[str, float] = dict.fromkeys(WRIST_JOINTS, 0.0)
        self.gripper_velocity = 0.0

        # Integrated position targets, seeded from /joint_states.
        self.wrist_target: Optional[Dict[str, float]] = None
        self.gripper_target: Optional[float] = None
        self.warned_no_state = False

        self.requested_mode = 'cartesian'
        self.active_mode: Optional[str] = None
        self.mode_request = None
        self.mode_request_after_ns: Optional[int] = None
        self.mode_ready_after_ns: Optional[int] = None
        self.last_service_warning = None
        self.last_mapping_warning = None

        self.enabled = False
        self.enable_was_pressed = False
        self.mode_was_pressed = False
        self.last_joy_time = None

        self.period = 0.02
        self.create_timer(self.period, self.control_loop)
        self.create_timer(0.25, self.ensure_servo_mode)

        self.get_logger().info(
            'Joystick teleop started: LOCKED, Cartesian mode requested')

    # --- helpers ---------------------------------------------------------

    def apply_deadzone(self, value: float) -> float:
        """Remove stick drift and rescale the remaining range."""
        if abs(value) <= self.deadzone:
            return 0.0
        scaled = (abs(value) - self.deadzone) / (1.0 - self.deadzone)
        return math.copysign(scaled, value)

    @staticmethod
    def trigger_amount(value: float) -> float:
        """Convert trigger range from [+1, -1] to [0, 1]."""
        return (1.0 - max(-1.0, min(1.0, value))) / 2.0

    def stop_motion(self) -> None:
        self.command = [0.0] * 3
        self.shoulder_velocity = 0.0
        self.wrist_velocity = dict.fromkeys(WRIST_JOINTS, 0.0)
        self.gripper_velocity = 0.0

    def on_joint_states(self, msg: JointState) -> None:
        if self.wrist_target is None:
            try:
                self.wrist_target = {
                    j: msg.position[msg.name.index(j)] for j in WRIST_JOINTS}
            except ValueError:
                pass
        if self.gripper_target is None:
            try:
                self.gripper_target = msg.position[msg.name.index(GRIPPER_JOINT)]
            except ValueError:
                pass

    # --- servo command mode ----------------------------------------------

    def ensure_servo_mode(self) -> None:
        """Servo ignores commands whose type it was not switched to."""
        if self.active_mode == self.requested_mode:
            return
        now = self.get_clock().now()
        if (self.mode_request_after_ns is not None
                and now.nanoseconds < self.mode_request_after_ns):
            return
        if self.mode_request is not None and not self.mode_request.done():
            return
        if not self.switch_client.service_is_ready():
            if (self.last_service_warning is None
                    or (now - self.last_service_warning).nanoseconds > 5_000_000_000):
                self.get_logger().warning(
                    'Waiting for MoveIt Servo command-mode service')
                self.last_service_warning = now
            return

        requested = self.requested_mode
        request = ServoCommandType.Request()
        request.command_type = (ServoCommandType.Request.TWIST
                                if requested == 'cartesian'
                                else ServoCommandType.Request.JOINT_JOG)
        self.mode_request_after_ns = None
        self.mode_request = self.switch_client.call_async(request)
        self.mode_request.add_done_callback(
            lambda future: self.on_servo_mode_response(future, requested))

    def on_servo_mode_response(self, future, requested: str) -> None:
        try:
            response = future.result()
        except Exception as error:  # ROS service failure
            self.get_logger().error(f'Failed to switch Servo mode: {error}')
            return
        if not response.success:
            self.get_logger().error('MoveIt Servo rejected the mode switch')
            return

        self.active_mode = requested
        self.mode_ready_after_ns = (self.get_clock().now().nanoseconds
                                    + int(self.settle_time * 1e9))
        self.stop_motion()
        self.get_logger().info(f"Control mode: {requested.replace('_', ' ').title()}")

    def toggle_control_mode(self) -> None:
        self.requested_mode = ('shoulder_pivot'
                               if self.requested_mode == 'cartesian' else 'cartesian')
        self.active_mode = None
        self.mode_request_after_ns = (self.get_clock().now().nanoseconds
                                      + int(self.settle_time * 1e9))
        self.mode_ready_after_ns = None
        self.stop_motion()
        self.get_logger().info(
            f"Requesting control mode: {self.requested_mode.replace('_', ' ').title()}")
        self.ensure_servo_mode()

    # --- joystick input ---------------------------------------------------

    def on_joy(self, msg: Joy) -> None:
        if len(msg.axes) < self.min_axes or len(msg.buttons) < self.min_buttons:
            # Throttled: a mismatched pad publishes at tens of hertz.
            now = self.get_clock().now()
            if (self.last_mapping_warning is None
                    or (now - self.last_mapping_warning).nanoseconds > 5_000_000_000):
                self.get_logger().error(
                    f'Unexpected /joy mapping: got {len(msg.axes)} axes and '
                    f'{len(msg.buttons)} buttons, profile needs at least '
                    f'{self.min_axes} and {self.min_buttons}')
                self.last_mapping_warning = now
            self.lock('malformed /joy')
            return

        self.last_joy_time = self.get_clock().now()

        enable_pressed = bool(msg.buttons[self.button['enable_toggle']])
        mode_pressed = bool(msg.buttons[self.button['mode_toggle']])

        if mode_pressed and not self.mode_was_pressed:
            self.toggle_control_mode()
        self.mode_was_pressed = mode_pressed

        if enable_pressed and not self.enable_was_pressed:
            if self.enabled:
                self.lock('button')
            else:
                self.try_unlock(msg)
        self.enable_was_pressed = enable_pressed

        if not self.enabled:
            self.stop_motion()
            return

        left_bumper = bool(msg.buttons[self.button['left_bumper']])
        right_bumper = bool(msg.buttons[self.button['right_bumper']])

        left_x = self.apply_deadzone(msg.axes[self.axis['left_x']])
        left_y = self.apply_deadzone(msg.axes[self.axis['left_y']])
        right_x = self.apply_deadzone(msg.axes[self.axis['right_x']])
        right_y = self.apply_deadzone(msg.axes[self.axis['right_y']])

        # Bumpers divert their stick axis to independent wrist jogging.
        self.wrist_velocity = {
            'wrist_roll': left_x * self.max_joint if left_bumper else 0.0,
            'wrist_pitch': -right_y * self.max_joint if right_bumper else 0.0,
        }

        left_lateral = 0.0 if left_bumper else left_x
        right_lateral = 0.0 if right_bumper else right_x
        lateral = (left_lateral if abs(left_lateral) >= abs(right_lateral)
                   else right_lateral)

        self.command = [0.0] * 3
        self.shoulder_velocity = 0.0
        if self.requested_mode == 'cartesian':
            self.command = [
                0.0 if right_bumper else right_y * self.max_linear,
                lateral * self.max_linear,
                0.0 if left_bumper else left_y * self.max_linear,
            ]
        else:
            self.shoulder_velocity = lateral * self.max_joint

        close = self.trigger_amount(msg.axes[self.axis['left_trigger']])
        open_ = self.trigger_amount(msg.axes[self.axis['right_trigger']])
        # Positive opens; negative closes.
        self.gripper_velocity = (open_ - close) * self.max_gripper

    def try_unlock(self, msg: Joy) -> None:
        sticks_neutral = all(
            abs(msg.axes[self.axis[n]]) <= self.deadzone
            for n in ('left_x', 'left_y', 'right_x', 'right_y'))
        # Read the analog triggers rather than digital trigger buttons, which
        # Xbox pads do not report.
        triggers_released = all(
            self.trigger_amount(msg.axes[self.axis[n]]) <= 0.1
            for n in ('left_trigger', 'right_trigger'))

        if sticks_neutral and triggers_released:
            self.enabled = True
            self.get_logger().info('Teleoperation UNLOCKED')
        else:
            self.get_logger().warning('Cannot unlock: release sticks and triggers')

    def lock(self, reason: str) -> None:
        if self.enabled:
            self.enabled = False
            self.get_logger().info(f'Teleoperation LOCKED ({reason})')
        self.stop_motion()

    # --- output -----------------------------------------------------------

    def control_loop(self) -> None:
        now = self.get_clock().now()

        timed_out = (self.last_joy_time is None
                     or (now - self.last_joy_time).nanoseconds / 1e9 > self.joy_timeout)
        if timed_out:
            self.lock('joy timeout')

        self.publish_wrist()
        self.publish_gripper()

        # Servo's incoming_command_timeout is 0.1 s, so keep republishing rather
        # than only sending on a Joy callback. Zeros while locked actively halt.
        mode_ready = (self.active_mode == self.requested_mode
                      and self.mode_ready_after_ns is not None
                      and now.nanoseconds >= self.mode_ready_after_ns)
        if not mode_ready:
            return

        if self.active_mode == 'cartesian':
            twist = TwistStamped()
            twist.header.stamp = now.to_msg()
            twist.header.frame_id = self.frame_id
            twist.twist.linear.x = self.command[0]
            twist.twist.linear.y = self.command[1]
            twist.twist.linear.z = self.command[2]
            self.twist_pub.publish(twist)
        else:
            jog = JointJog()
            jog.header.stamp = now.to_msg()
            jog.header.frame_id = self.frame_id
            jog.joint_names = ['shoulder_pan']
            jog.velocities = [self.shoulder_velocity]
            self.joint_pub.publish(jog)

    def publish_wrist(self) -> None:
        """Integrate wrist velocity into a position target.

        wrist_pitch and wrist_roll are outside Servo's planning group, so they
        go straight to their trajectory controller.
        """
        if not any(self.wrist_velocity.values()):
            return
        if self.wrist_target is None:
            self.warn_no_state()
            return
        for joint, velocity in self.wrist_velocity.items():
            low, high = WRIST_LIMITS[joint]
            moved = self.wrist_target[joint] + velocity * self.period
            self.wrist_target[joint] = max(low, min(high, moved))
        point = JointTrajectoryPoint(
            positions=[self.wrist_target[j] for j in WRIST_JOINTS],
            velocities=[0.0] * len(WRIST_JOINTS),
            time_from_start=Duration(seconds=2 * self.period).to_msg())
        self.wrist_pub.publish(
            JointTrajectory(joint_names=WRIST_JOINTS, points=[point]))

    def publish_gripper(self) -> None:
        if self.gripper_velocity == 0.0:
            return
        if self.gripper_target is None:
            self.warn_no_state()
            return
        moved = self.gripper_target + self.gripper_velocity * self.period
        self.gripper_target = max(0.0, min(self.gripper_open, moved))
        point = JointTrajectoryPoint(
            positions=[self.gripper_target], velocities=[0.0],
            time_from_start=Duration(seconds=2 * self.period).to_msg())
        self.gripper_pub.publish(
            JointTrajectory(joint_names=[GRIPPER_JOINT], points=[point]))

    def warn_no_state(self) -> None:
        if not self.warned_no_state:
            self.get_logger().warning('Waiting for /joint_states before jogging')
            self.warned_no_state = True


def main() -> None:
    rclpy.init()
    node = JoystickTeleop()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
