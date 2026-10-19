# Quick start

## Environment and build

The target environment is Ubuntu 22.04, system Python 3, and ROS 2 Humble. PyBullet and ROS Python nodes must use the same Python environment with access to `rclpy`. From the repository root, run:

```bash
./scripts/setup.sh
```

The script checks dependencies and builds the project. Install any missing system dependencies listed in its output, then rerun it. Use a Python environment that can load the system ROS libraries.

On a new machine, install ROS 2 Humble first, then the required ROS control components and PyBullet:

```bash
sudo apt-get install ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-robot-state-publisher python3-colcon-common-extensions \
  python3-numpy python3-pip python3-pytest
/usr/bin/python3 -m pip install --user -r requirements.txt
./scripts/setup.sh
```

## First run

Run the joint demo, wait for successful completion, then run the Cartesian demo:

```bash
./scripts/run_demo.sh joint
./scripts/run_demo.sh cartesian
```

For a graphical window, use a terminal with an available desktop display:

```bash
./scripts/run_demo.sh joint gui:=true
```

The `joint` demo sends seven joint position targets, waits for controller completion, and checks physical feedback. The `cartesian` demo generates a small path from the current end-effector pose, solves IK at each sample, and submits it to the same trajectory action. The joint demo checks the input-to-servo chain; the Cartesian demo also checks kinematics and the end-effector path.

The script saves `artifacts/joint_report.json`, `artifacts/cartesian_report.json`, and corresponding `*_launch.log` files. Each demo stops the launch process it started. If the system is already running, use the separate-terminal procedure below to avoid duplicate instances.

## Run and observe in separate terminals

In terminal A, open the repository root and start the system:

```bash
./scripts/run_demo.sh bringup gui:=true
```

In terminal B, open the same repository root and set up the environment:

```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=87
ros2 control list_controllers -c /lbr/controller_manager
ros2 action list -t
```

Both `joint_state_broadcaster` and `joint_trajectory_controller` should be `active`. The action list should contain `/lbr/joint_trajectory_controller/follow_joint_trajectory`. Run the demos one at a time:

```bash
ros2 run lbr_pybullet_tutorial demo --mode joint --timeout 60 --report /tmp/lbr_joint_report.json
ros2 run lbr_pybullet_tutorial demo --mode cartesian --timeout 60 --report /tmp/lbr_cartesian_report.json
```

`--report` writes the current result to a JSON file for comparison. See [validation](validation.md) for success criteria and measurement interpretation.

Use another terminal with the same sourced environment and domain to observe:

```bash
ros2 topic echo /lbr_tutorial/status
ros2 topic echo /iiwa7/joint_states
ros2 topic echo /lbr/joint_trajectory_controller/state
```

Physical feedback comes from `/iiwa7/joint_states`. Controller state includes desired, actual, and error values. Publishing a target alone does not establish successful tracking.

## Send a ROS 2 joint goal

With the system running, send a goal from the command line:

```bash
ros2 topic pub --once /lbr_tutorial/joint_goal sensor_msgs/msg/JointState \
  "{name: [lbr_A1, lbr_A2, lbr_A3, lbr_A4, lbr_A5, lbr_A6, lbr_A7], position: [0.1, 0.4, 0.0, -1.1, 0.0, 0.9, 0.0]}"
```

Positions are in radians. The motion node validates the complete goal and creates a trajectory for the controller. Start with the default demo, then change values and observe measured feedback. Joint names must be complete and unique, and targets must remain within model limits.

End-effector inputs use `/lbr_tutorial/pose_goal` or `/lbr_tutorial/cartesian_path`. Obtain values from the current robot's FK or a validated target. Arbitrary XYZ coordinates and quaternions may be unreachable. Use `demo --mode cartesian` for the first experiment; see [interfaces](interfaces.md) for fields and frame conventions.

The default backend publishes `/lbr/ee_pose`. Inspect the measured pose:

```bash
ros2 topic echo --once /lbr/ee_pose
```

For the default tutorial model, copy the pose, set `frame_id: lbr_link_0`, and set the timestamp to zero to construct a hold-position goal. Then try about 0.01 m along one axis. Do not reuse the feedback acquisition timestamp as the goal timestamp. The `existing` backend does not provide this topic; calculate FK from `/lbr/joint_states` with the same model. For configured workbench tools, follow the [TCP conventions](workbench.md).

Request cancellation with:

```bash
ros2 service call /lbr_tutorial/cancel std_srvs/srv/Trigger '{}'
```

Observe the final status and wait until idle before sending another goal.

## Run the original LBR Stack client

With `bringup` running, use a second terminal with the same environment and domain:

```bash
/usr/bin/python3 scripts/upstream_joint_demo.py --ros-args -r __ns:=/lbr
```

The wrapper loads the unmodified `third_party/lbr_stack/joint_trajectory_client.py` and calls its `main()`. The upstream example moves odd-numbered joints to 1 rad, then returns all joints to zero. Each segment takes about 15 s, approximately 30 s total. Namespace remapping resolves its relative action to `/lbr/joint_trajectory_controller/follow_joint_trajectory`.

This checks the upstream client, the same JTC, the custom hardware plugin, and PyBullet. It bypasses the tutorial goal receiver; submit no other goals during this run. The wrapper adds an entry point without changing the upstream algorithm.

## Use an existing PyBullet node

The default backend is included in this repository. To test an existing `ros2_pybullet_ws` implementation, source its built overlay before this repository's overlay. Replace `/path/to/ros2_pybullet_ws` with its actual location:

```bash
source /opt/ros/humble/setup.bash
source /path/to/ros2_pybullet_ws/install/local_setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=87
ros2 launch lbr_pybullet_tutorial tutorial.launch.py sim_backend:=existing gui:=false
```

This launches the existing simulator and requires its ROS package to be installed. It uses joint position control (`initial_control_mode:=1`), disables legacy `goto` control (`use_goto:=false`), and resets to the same initial positions as the tutorial. Thus `/iiwa7/joint_desired` is the sole trajectory input, avoiding two interpolators overwriting commands.

The existing node is `pybullet_ros2_sim/iiwa_pybullet_sim_node`. Launch parameters include `reset_to_init_on_start:=true` and initial positions `[0.0, 0.35, 0.0, -0.9, 0.0, 0.65, 0.0]` rad. The wrapper also supports:

```bash
export LBR_TUTORIAL_LEGACY_WS=/path/to/ros2_pybullet_ws
./scripts/run_demo.sh joint sim_backend:=existing
./scripts/run_demo.sh cartesian sim_backend:=existing
```

Use the actual workspace path. Run only one simulator for a given `iiwa7` instance. If its namespace, message types, joint count, or model differ, update and validate the adapter configuration first. Matching topic names alone do not establish compatibility.

For manual runs, source both overlays in the second terminal and run demos sequentially. Use `Ctrl+C` to stop the demo and launch when finished. Record `existing` results separately from default-backend results.

## Troubleshooting

| Symptom | Check |
| --- | --- |
| No `ros2` or Python cannot import `rclpy` | Use system Python; source `/opt/ros/humble/setup.bash`. |
| Tutorial packages unavailable | Complete `setup.sh`, then source `install/local_setup.bash`. |
| Demo waits for an action or state | Match `ROS_DOMAIN_ID` across terminals; check launch and controllers. |
| Controller waits for hardware state | Check advancing `/iiwa7/joint_states`, duplicate simulators, and namespaces. |
| Action exists but system is not ready | Verify fresh measured joints and JTC state; an action server alone is insufficient. |
| Goal rejected as busy | Wait for completion or request cancellation; new messages do not preempt. |
| Pose rejected | Check frame, zero timestamp, finite values, quaternion, IK, and joint limits. |
| `feedback/action timeout` | The fault is latched. Restore feedback, then restart tutorial bringup. |
| GUI cannot open | Check with `gui:=false`; graphical mode requires a working desktop display. |
| ROS shared-memory or discovery errors | Check the independent domain; leave unrelated workspace nodes running. |

Check in order: advancing simulator state, active hardware, active controllers, action availability, demo acceptance and final status, then physical feedback and error reports.
