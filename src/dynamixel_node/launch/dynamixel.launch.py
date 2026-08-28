"""Standalone bring-up for the Dynamixel bus. Nothing else is launched."""

from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('baud_rate', default_value='57600'),
        # Empty means: broadcast ping and use whatever answers.
        DeclareLaunchArgument('ids', default_value='[]'),
        DeclareLaunchArgument('publish_rate', default_value='10.0'),
        DeclareLaunchArgument('enable_torque_on_start', default_value='false'),
        DeclareLaunchArgument('disable_torque_on_shutdown', default_value='false'),
    ]

    dynamixel_node = Node(
        package='dynamixel_node',
        executable='dynamixel_node',
        name='dynamixel_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baud_rate': ParameterValue(
                LaunchConfiguration('baud_rate'), value_type=int),
            'ids': ParameterValue(
                LaunchConfiguration('ids'), value_type=List[int]),
            'publish_rate': ParameterValue(
                LaunchConfiguration('publish_rate'), value_type=float),
            'enable_torque_on_start': ParameterValue(
                LaunchConfiguration('enable_torque_on_start'), value_type=bool),
            'disable_torque_on_shutdown': ParameterValue(
                LaunchConfiguration('disable_torque_on_shutdown'), value_type=bool),
        }],
    )

    return LaunchDescription(arguments + [dynamixel_node])
