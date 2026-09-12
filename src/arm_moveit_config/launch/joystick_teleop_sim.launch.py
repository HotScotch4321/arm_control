"""Gamepad teleop on top of the mock-hardware simulation stack.

Brings up servo_sim.launch.py, then the joy driver and the joystick teleop node.
Defaults to an Xbox pad; pass controller_profile:=joystick_dualsense.yaml for a
PlayStation pad.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start RViz2",
        ),
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="true",
            description="Start robot with mock hardware mirroring command to its states.",
        ),
        DeclareLaunchArgument(
            "joy_topic",
            default_value="/joy",
            description="Topic the joy driver publishes on",
        ),
        DeclareLaunchArgument(
            "joy_device_id",
            default_value="0",
            description="Index of the joystick device to open",
        ),
        DeclareLaunchArgument(
            "controller_profile",
            default_value="joystick_xbox.yaml",
            description=(
                "Controller profile in this package's config/ directory: "
                "joystick_xbox.yaml or joystick_dualsense.yaml"
            ),
        ),
    ]

    use_rviz = LaunchConfiguration("use_rviz")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    joy_topic = LaunchConfiguration("joy_topic")
    joy_device_id = LaunchConfiguration("joy_device_id")
    controller_profile = LaunchConfiguration("controller_profile")

    profile_path = PathJoinSubstitution(
        [FindPackageShare("arm_moveit_config"), "config", controller_profile]
    )

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("arm_moveit_config"),
                    "launch",
                    "servo_sim.launch.py",
                ]
            )
        ),
        launch_arguments={
            "use_rviz": use_rviz,
            "use_mock_hardware": use_mock_hardware,
        }.items(),
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        parameters=[
            {
                "device_id": joy_device_id,
                # Deadzone is applied in the teleop node so that one profile
                # governs it; let the driver pass the raw axes through.
                "deadzone": 0.0,
                # Servo times commands out after 0.1 s, so the driver must keep
                # publishing while a stick is held still.
                "autorepeat_rate": 20.0,
            }
        ],
        remappings=[("joy", joy_topic)],
    )

    joystick_teleop_node = Node(
        package="arm_moveit_config",
        executable="joystick_control",
        name="joystick_teleop",
        output="screen",
        parameters=[profile_path],
        remappings=[("/joy", joy_topic)],
    )

    # The teleop node tolerates Servo not being up yet -- it warns and retries
    # the mode-switch service -- so it does not need the spawner sequencing that
    # servo_sim.launch.py applies internally.
    return LaunchDescription(
        declared_arguments + [simulation, joy_node, joystick_teleop_node]
    )
