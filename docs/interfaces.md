# Interfaces frames and interpolation

## Inputs and outputs

This table describes the standalone tutorial's default robot. For configured workbench projects, pose and path goals refer to the configured TCP; see the [workbench guide](workbench.md).

| Interface | Type | Meaning |
| --- | --- | --- |
| `/lbr_tutorial/joint_goal` | `sensor_msgs/msg/JointState` | Complete seven-joint position goal, rad |
| `/lbr_tutorial/pose_goal` | `geometry_msgs/msg/PoseStamped` | Single link7 pose for the default model, m |
| `/lbr_tutorial/cartesian_path` | `nav_msgs/msg/Path` | Ordered end-effector poses |
| `/lbr_tutorial/status` | `std_msgs/msg/String` | JSON events with node-assigned `request_id` |
| `/lbr_tutorial/cancel` | `std_srvs/srv/Trigger` | Request cancellation of the active task |
| `/lbr/joint_trajectory_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | Standard trajectory execution action |
| `/lbr/ee_pose` | `geometry_msgs/msg/PoseStamped` | Measured link7 pose from the default simulator; unavailable in `existing` |
| `/lbr/joint_states` | `sensor_msgs/msg/JointState` | LBR joint state broadcast from measured hardware state |
| `/lbr/joint_trajectory_controller/state` | `control_msgs/msg/JointTrajectoryControllerState` | Humble JTC desired, actual, and error state |

`accepted` means the tutorial node generated a trajectory and started submitting the action. The controller may still reject it and produce `failed`. Wait for the final event with the same ID. A successful cancellation service response means cancellation was requested; final action status is reported separately. With no active task, it returns `success: false`.

The receiver executes one task at a time. While busy, it rejects new input without preempting the active goal. Events are `ready`, `accepted`, `succeeded`, `rejected`, and `failed`. Each request has its own `request_id`. A busy rejection belongs to the new request, not the active task.

Readiness requires fresh measured joint state, advancing raw PyBullet timestamps, fresh JTC state, and an available action server. A broadcaster can republish its last sample; the arrival rate of `/lbr/joint_states` alone does not establish fresh physical feedback. The node independently monitors source timestamps on `/iiwa7/joint_states`. State or action timeouts during execution report `failed` and latch a fault. Further goals are rejected until tutorial bringup is restarted. Motion does not resume automatically after interrupted feedback.

`JointState` is a position-goal container here, rather than a timed trajectory protocol. `name` must cover `lbr_A1` through `lbr_A7` without duplicates, with matching `position` values. Do not encode extra commands in `velocity` or `effort`. For explicit multi-point timing, use `FollowJointTrajectory` directly and avoid simultaneous tutorial-topic goals.

The hardware command `/iiwa7/joint_desired` is `std_msgs/msg/Float64MultiArray`, with seven ordered joint positions. Feedback `/iiwa7/joint_states` is `sensor_msgs/msg/JointState`, named `iiwa_joint_1` through `iiwa_joint_7`, with complete positions and velocities. The bridge reads feedback by name, rather than assuming incoming array order.

## Frames and time

- The default end effector is the seventh-link origin in `pybullet_data/kuka_iiwa/model.urdf`, PyBullet link index 6. The original name is `lbr_iiwa_link_7`; the generated ROS name is `lbr_link_7`. This is the link frame, with no added gripper TCP, and not the centre-of-mass frame.
- `PoseStamped.header.frame_id` accepts only `world` or `lbr_link_0`, which coincide at the fixed base in this tutorial.
- `Path.header.frame_id` follows the same convention. An empty frame in a contained pose inherits the path frame; an explicit frame must be consistent.
- Quaternions use ROS order `x, y, z, w`. Values must be finite, with norm within 0.001 of 1. Do not put Euler angles in quaternion fields.
- Goal timestamps must be zero. Planning starts on receipt; nonzero timestamps are rejected. The interface does not schedule future movement or replay historical poses.

Transform camera-frame points or grasp poses with application-level tf2 before submission. This interface does not query camera TF or provide calibration; `camera_*` frames are rejected.

Replace the placeholders below with FK, trajectory-generator, or calibrated planning values before publishing:

```yaml
header:
  frame_id: lbr_link_0
  stamp: {sec: 0, nanosec: 0}
pose:
  position: {x: x, y: y, z: z}
  orientation: {x: qx, y: qy, z: qz, w: qw}
```

A Path uses the same header convention and an ordered `poses` array. The tutorial generator assigns timing; Path timestamps do not specify arrival times at points.

## From joint goals to physical feedback

1. The tutorial node reads measured joint positions, validates input, and generates a `JointTrajectory`.
2. The action submits the full trajectory to the running official `joint_trajectory_controller`.
3. Each controller update calculates desired positions and writes the `ros2_control` position command interfaces.
4. C++ `SystemInterface.write()` publishes commands to `/iiwa7/joint_desired`.
5. PyBullet drives joint position servos while continuously stepping physics.
6. `/iiwa7/joint_states` carries measured positions and velocities; `SystemInterface.read()` supplies these to the controller.

The broadcaster publishes simulator state read through the hardware interface. During debugging, observe controller error and raw `/iiwa7/joint_states` as well as desired commands.

## Cartesian trajectory execution

Cartesian input is sampled in position with quaternion SLERP for orientation. Numerical IK solves each sample, seeded by the previous joint solution to reduce branch jumps. The whole joint trajectory is submitted only after all points convert successfully and pass limits.

JTC interpolates the resulting timed joint waypoints during control updates. Humble documents linear interpolation for positions only, cubic for positions and velocities, and quintic for positions, velocities, and accelerations. See the [official interpolation documentation](https://control.ros.org/humble/doc/ros2_controllers/joint_trajectory_controller/doc/trajectory.html).

The joint demo supplies endpoint positions and zero endpoint velocities, yielding cubic interpolation. Cartesian samples supply positions only, yielding linear interpolation between points. This implementation does not establish continuous Cartesian velocity or acceleration.

| Current default | Value |
| --- | --- |
| Controller update | 100 Hz |
| Default PyBullet physics step | 240 Hz |
| Controller state publication | 50 Hz |
| Minimum joint goal duration | 4 s, extended for joint speed requirements |
| Joint speed reference | 0.35 rad/s |
| Maximum Cartesian sampling interval | Translation 0.004 m; rotation 0.03 rad |
| Cartesian speed reference | Translation 0.04 m/s; rotation 0.20 rad/s |
| Maximum joint change between adjacent IK solutions | 0.25 rad |
| Independent FK check of IK | Position 0.001 m; orientation 0.005 rad |
| Path limits | 1–100 poses; at most 2000 samples; at most 120 s |
| Cartesian planning budget | 5 s, checked in the sampling loop; timeout rejects the entire path |

Any position coordinate with absolute value above 2 m is rejected as a coarse invalid-input guard. Targets inside this envelope still need IK; it is not the reachable workspace.

Speed references allocate time and do not independently certify complete continuous velocity or acceleration constraints. Actual timing depends on machine load; 240 Hz is a default timer setting. This implementation publishes no `/clock` and uses the system ROS clock.

Cartesian tracking includes geometric approximation from sampling and IK, and physical tracking error. Denser sampling can reduce some geometric error. Speed planning, singularity handling, collision planning, and the full path between samples require additional validation.

The tutorial provides no MoveIt planning scene, obstacle avoidance, or gripper contact strategy. Joint limits and IK validate motion inputs; grasp feasibility requires further checks.

## Joint name mapping

| Control interface | PyBullet revolute joint order |
| --- | --- |
| `lbr_A1` | 1 |
| `lbr_A2` | 2 |
| `lbr_A3` | 3 |
| `lbr_A4` | 4 |
| `lbr_A5` | 5 |
| `lbr_A6` | 6 |
| `lbr_A7` | 7 |

The mapping uses seven movable model joints; physical feedback must match. After changing the URDF, base transform, or tool, update simulation, kinematics, and frame documentation together, then rerun both demos.

Activation waits for measured feedback and initializes commands to received positions. After 0.5 s without fresh feedback, the hardware interface reports an error and stops position commands. The default simulator has a separate 0.5 s command timeout that holds measured positions. Planning FK/IK uses a separate PyBullet DIRECT client; resetting that model does not teleport the physical simulation.

## Relationship to LBR Stack

The upstream Python example sends `lbr_A1` through `lbr_A7` goals using `FollowJointTrajectory`. This tutorial retains the same standard action entry point and provenance. See the [upstream client](https://github.com/lbr-stack/lbr_fri_ros2_stack/blob/humble/lbr_demos/lbr_demos_py/lbr_demos_py/joint_trajectory_client.py).

LBR Stack provides real-robot FRI and simulation integration. This project connects its ROS trajectory-control approach to PyBullet. Real LBR migration requires a replacement hardware layer, matching robot description and controller configuration, and renewed frame, TCP, and motion-limit validation. Passing PyBullet tests does not establish real-robot performance.
