# arm_moveit_config

Standalone MoveIt configuration + keyboard teleop for the arm, extracted from
`perseus_payloads` in `perseus-v2`. Deliberately contains **no hardware
drivers** — no CAN, no RSBL/RMD, no `PerseusArmHardware`. The point is an
isolated sim environment for iterating on servos that aren't the Perseus ones.

## Layout

| Path | What |
| --- | --- |
| `config/arm.urdf.xacro` | Top-level description; pulls in the URDF and the ros2_control block |
| `config/arm.ros2_control.xacro` | Joint/interface definitions; mock hardware by default |
| `config/arm.srdf` | Planning groups, EE, collision matrix |
| `config/ros2_controllers.yaml` | `joint_state_broadcaster`, `arm_controller`, `servo_controller` |
| `config/servo.yaml` | MoveIt Servo tuning |
| `config/{kinematics,joint_limits,ompl_planning,moveit_controllers,pilz_cartesian_limits}.yaml` | MoveIt config |
| `config/moveit.rviz` | RViz layout |
| `src/moveit/arm.urdf` | The arm geometry itself |
| `launch/servo_sim.launch.py` | Pure simulation stack (mock hardware) |
| `launch/servo.launch.py` | Same stack, but able to load your own hardware plugin |
| `scripts/keyboard_control.py` | Keyboard teleop over `/servo_node/delta_*_cmds` |

## Build & run

```bash
cd /home/maximus/proj/arm_control   # direnv loads the ROS 2 Jazzy flake shell
colcon build --packages-select arm_moveit_config
source install/setup.bash

# Simulation
ros2 launch arm_moveit_config servo_sim.launch.py

# Teleop, in a second shell
ros2 run arm_moveit_config keyboard_control
```

## Wiring in your own servos

Nothing here talks to hardware. When you have a `hardware_interface` plugin for
your servos, build it as its own package in this workspace and point this one at
it — no edits to the xacro needed:

```bash
ros2 launch arm_moveit_config servo.launch.py \
    use_mock_hardware:=false \
    hardware_plugin:=my_servos/MyServoHardware
```

The joint names the plugin must expose are in `config/arm.ros2_control.xacro`:
`shoulder_pan`, `shoulder_tilt`, `elbow`, `wrist_pitch`, `wrist_roll`,
`fake_dof` — each with `position`/`velocity` command and state interfaces.
