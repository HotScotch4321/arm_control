# arm_moveit_config

Standalone MoveIt configuration + keyboard teleop for the arm, extracted from
`perseus_payloads` in `perseus-v2`. 
## Layout

| Path | What |
| --- | --- |
| `config/arm.urdf.xacro` | Top-level description; pulls in the URDF and the ros2_control block |
| `config/arm.ros2_control.xacro` | Joint/interface definitions; mock hardware by default |
| `config/arm.srdf` | Planning groups, EE, collision matrix |
| `config/ros2_controllers.yaml` | `joint_state_broadcaster`, `servo_controller`, `gripper_controller` |
| `config/servo.yaml` | MoveIt Servo tuning |
| `config/{kinematics,joint_limits,ompl_planning,moveit_controllers,pilz_cartesian_limits}.yaml` | MoveIt config |
| `config/moveit.rviz` | RViz layout |
| `src/moveit/arm.urdf` | The arm geometry and the parallel-jaw gripper |
| `scripts/generate_ikfast_plugin.sh` | Builds `arm_ikfast_plugin` via OpenRAVE in Docker |
| `launch/servo_sim.launch.py` | Pure simulation stack (mock hardware) |
| `launch/servo.launch.py` | Same stack, but able to load your own hardware plugin |
| `scripts/keyboard_control.py` | Keyboard teleop over `/servo_node/delta_*_cmds` |

## Fresh machine

With nix (any Linux distro, including a Raspberry Pi):

```bash
git clone <repo> && cd arm_control
nix develop        # pinned env incl. the patched moveit_servo; first run may build for a while
```

Without nix, on stock ROS 2 Jazzy (Ubuntu 24.04 / Pi):

```bash
git clone <repo> && cd arm_control
./src/arm_moveit_config/scripts/patch_moveit_servo.sh   # apt's moveit_servo halts arms with <6 joints
rosdep install --from-paths src --ignore-src -y
colcon build
```

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
`left_finger_joint` — each with `position`/`velocity` command and state
interfaces. `right_finger_joint` is a URDF `mimic` of `left_finger_joint`, so
ros2_control drives it and the plugin must not expose it.


