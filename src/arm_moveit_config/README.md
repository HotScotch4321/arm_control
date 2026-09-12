# arm_moveit_config

Standalone MoveIt configuration + keyboard teleop for the arm, extracted from
`perseus_payloads` in `perseus-v2`. 
## Layout

| Path | What |
| --- | --- |
| `config/arm.urdf.xacro` | Top-level description; pulls in the URDF and the ros2_control block |
| `config/arm.ros2_control.xacro` | Joint/interface definitions; mock hardware by default |
| `config/arm.srdf` | Planning groups, EE, collision matrix |
| `config/ros2_controllers.yaml` | `joint_state_broadcaster`, `servo_controller`, `wrist_controller`, `gripper_controller`; `update_rate` 100 Hz |
| `config/servo.yaml` | MoveIt Servo tuning |
| `config/initial_positions.yaml` | Startup pose, also seeds mock hardware |
| `config/{kinematics,joint_limits,ompl_planning,moveit_controllers,pilz_cartesian_limits}.yaml` | MoveIt config |
| `config/moveit.rviz` | RViz layout |
| `src/moveit/arm.urdf` | The arm geometry and the parallel-jaw gripper |
| `launch/servo_sim.launch.py` | Full stack; mock by default, switches to a real plugin |
| `launch/servo.launch.py` | Same stack, defaulting to real hardware |
| `scripts/keyboard_control.py` | Keyboard teleop over `/servo_node/delta_*_cmds` |
| `scripts/patch_moveit_servo.sh` | Vendors patched `moveit_servo` for non-nix builds |
| `scripts/generate_ikfast_plugin.sh` | Regenerates `arm_ikfast_plugin` via OpenRAVE in Docker |
| `scripts/urdf_to_openrave.py` | Helper used by `generate_ikfast_plugin.sh` |

Nothing under `scripts/` is needed to build or run the repo.
`generate_ikfast_plugin.sh` rewrites one committed file
(`src/arm_ikfast_plugin/src/arm_arm_ikfast_solver.cpp`) and is only re-run when
the arm geometry in `arm.urdf` changes — the solver is baked against the link
offsets and joint axes, and is silently wrong if they drift.

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

That `moveit_servo` patch is the only one this workspace needs. The bug it fixes:
moveit_servo 2.12.4 indexes the SVD by the 6-D twist dimension in its
singularity check, so with the 3-DOF `arm` group it reads out of bounds and
emergency-stops every command (moveit2 #3411).

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

Nothing in this package talks to hardware. The driver lives in a sibling
package — `dynamixel_node`, which exports `dynamixel_node/DynamixelServos` and
is the default plugin. Point either launch file at a different one without
editing the xacro:

```bash
ros2 launch arm_moveit_config servo_sim.launch.py \
    use_mock_hardware:=false \
    hardware_plugin:=my_servos/MyServoHardware
```

`servo_sim.launch.py` defaults to mock; the two constants at the top of the file
flip it. `servo.launch.py` is the same stack defaulting to real hardware.

The joints the plugin must expose are in `config/arm.ros2_control.xacro`:
`shoulder_pan`, `shoulder_tilt`, `elbow`, `wrist_pitch`, `wrist_roll`,
`left_finger_joint` — each with a `position` **command** interface and
`position` + `velocity` **state** interfaces. Each carries a
`<param name="servo_id">`; the four `<transmission>` blocks carry the
mechanical reduction. `right_finger_joint` is a URDF `mimic` of
`left_finger_joint`, so ros2_control drives it and the plugin must not expose
it.

Serial port and baud rate are deliberately not in the xacro — they belong to
whichever plugin is loaded.


