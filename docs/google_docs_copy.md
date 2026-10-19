PyBullet Motion Control and Student Implementation Guide

1. Project objective

Build, debug, and visualize a self-contained ROS 2 motion-control project. Other nodes send joint positions, end-effector poses, or Cartesian paths; this module converts and executes them and reports results. It provides the motion layer for llm_ros, sam3_ros, and later medical instrument sorting tasks.

Repository: https://github.com/jackysqing-max/robot-visual-simulation-platform
The ROS packages remain lbr_pybullet_control and lbr_pybullet_tutorial, with /lbr_tutorial interfaces retained for tutorial compatibility. The standalone motion core can run without the desktop workbench.

2. Control chain

ROS 2 goal -> trajectory generation / numerical IK -> FollowJointTrajectory -> joint_trajectory_controller (JTC) -> ros2_control hardware plugin -> PyBullet position servos and physics stepping -> measured joint feedback.

The project reuses LBR Stack controller configuration, standard actions, and the original joint trajectory client. JTC is the official ros2_controllers implementation. Cartesian sampling, quaternion SLERP, and numerical IK are implemented here. Joint demos supply zero endpoint velocities for cubic interpolation; Cartesian joint samples contain positions only and use linear JTC interpolation. Physical feedback supports controller state and error monitoring.

The default model is PyBullet's seven-axis iiwa. Its end effector is link7, with no additional tool TCP, and world coincides with lbr_link_0. Arm-motion simulation is implemented. Gripper actuation, obstacle avoidance, grasp-result verification, and real-robot migration are later work. Configured workbench projects have explicit tools and TCPs; follow docs/workbench.md for their conventions.

3. Components and installation

src/lbr_pybullet_control: C++ SystemInterface between JTC commands and measured PyBullet state.
src/lbr_pybullet_tutorial: simulation, FK/IK, goal receiver, demos, launches, configuration, and workbench.
scripts: setup, build, launch, automated checks, and tracking plots.
docs: interfaces, quick start, student plan, and validation evidence.
third_party/lbr_stack: pinned upstream configuration/client, provenance, and license.

Use Ubuntu 22.04, ROS 2 Humble, and system Python 3. Install ROS 2 Humble first. From the repository root:

```bash
sudo apt-get install ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-robot-state-publisher python3-colcon-common-extensions \
  python3-numpy python3-pip python3-pytest
/usr/bin/python3 -m pip install --user -r requirements.txt
./scripts/setup.sh
```

The default backend is self-contained and requires no legacy workspace or full LBR Stack/FRI SDK. See README.md for desktop UI dependencies.

4. Standalone demos and GUI debugging

Run these sequentially. Each starts its system, executes checks, saves a report, and stops its own launch:

```bash
./scripts/run_demo.sh joint gui:=true
./scripts/run_demo.sh cartesian gui:=true
```

Use gui:=false for headless operation. GUI mode requires a desktop display and shows robot motion with a green measured end-effector trail. Adjust the view with the mouse. The Cartesian demo generates a closed 3 cm square from FK of the current measured joints and holds orientation. Reports are saved as artifacts/*_report.json.

For continuous observation and manual goals, start terminal A:

```bash
./scripts/run_demo.sh bringup gui:=true
```

In terminal B, open the repository root and apply the following setup. Use the same setup for subsequent ros2 commands:

```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=87
ros2 control list_controllers -c /lbr/controller_manager
ros2 topic echo /lbr_tutorial/status
```

Both controllers should be active. Another configured terminal can run ros2 run lbr_pybullet_tutorial demo --mode joint or --mode cartesian, while observing /iiwa7/joint_states and /lbr/joint_trajectory_controller/state.

5. ROS 2 interfaces and examples

/lbr_tutorial/joint_goal: sensor_msgs/msg/JointState; complete seven-joint positions, rad.
/lbr_tutorial/pose_goal: geometry_msgs/msg/PoseStamped; one link7 pose for the default model, m.
/lbr_tutorial/cartesian_path: nav_msgs/msg/Path; ordered end-effector poses.
/lbr_tutorial/status: std_msgs/msg/String; JSON events and request_id.
/lbr_tutorial/cancel: std_srvs/srv/Trigger; cancel the active task.
/lbr/joint_trajectory_controller/follow_joint_trajectory: standard FollowJointTrajectory action for explicit multi-point timing.
/lbr/joint_states and /lbr/joint_trajectory_controller/state: measured joints and desired/actual/error.
/lbr/ee_pose: measured link7 pose from the default simulator.

After ready, send a joint goal:

```bash
ros2 topic pub --once /lbr_tutorial/joint_goal sensor_msgs/msg/JointState \
  "{name: [lbr_A1, lbr_A2, lbr_A3, lbr_A4, lbr_A5, lbr_A6, lbr_A7], position: [0.1, 0.4, 0.0, -1.1, 0.0, 0.9, 0.0]}"
```

End-effector targets must come from model FK or validated planning. This runnable default-model example reads /lbr/ee_pose and moves 1 cm along y. Set mode to path for a move-and-return path. Wait for the preceding task to finish before running it:

```bash
/usr/bin/python3 - <<'PY'
import copy
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
rclpy.init()
n = rclpy.create_node('cartesian_input_example')
received = []
sub = n.create_subscription(PoseStamped, '/lbr/ee_pose', received.append, 10)
while not received:
    rclpy.spin_once(n, timeout_sec=0.2)
start = copy.deepcopy(received[-1])
start.header.frame_id = 'lbr_link_0'
start.header.stamp.sec = start.header.stamp.nanosec = 0
target = copy.deepcopy(start)
target.pose.position.y += 0.01
mode = 'pose'  # Change to 'path' for the alternative input.
msg = target if mode == 'pose' else Path(header=start.header, poses=[target, start])
topic = '/lbr_tutorial/pose_goal' if mode == 'pose' else '/lbr_tutorial/cartesian_path'
pub = n.create_publisher(type(msg), topic, 10)
while pub.get_subscription_count() == 0:
    rclpy.spin_once(n, timeout_sec=0.2)
pub.publish(msg)
for _ in range(5):
    rclpy.spin_once(n, timeout_sec=0.1)
n.destroy_node()
rclpy.shutdown()
PY
```

Pose frames must be world or lbr_link_0. Goal timestamps must be zero; quaternions must be normalized in x/y/z/w order. Transform camera-frame targets and tool TCPs at the application layer as required by the selected model. Wait for succeeded, failed, or rejected with the matching request_id; accepted means submission has started. Busy inputs are rejected. Request cancellation with:

```bash
ros2 service call /lbr_tutorial/cancel std_srvs/srv/Trigger '{}'
```

6. Compose with other nodes

Other modules in the same ROS_DOMAIN_ID can publish goals and subscribe to status without importing this project's Python source. Submit goals serially, correlate motion request IDs, and advance only after the matching final result. Direct action clients can use action goal UUIDs for correlation.

Simulation and control can run separately. In a configured terminal A:

```bash
ros2 launch lbr_pybullet_tutorial sim.launch.py gui:=true
```

In terminal B, start controllers and goal reception against the running compatible simulator:

```bash
./scripts/run_demo.sh bringup sim_backend:=external
```

The simulator must receive /iiwa7/joint_desired (Float64MultiArray, seven joints in rad) and continuously publish /iiwa7/joint_states (JointState, iiwa_joint_1 through iiwa_joint_7, complete position/velocity, advancing timestamps). Use the same model and mapping, one simulator per robot, and one command-writing chain.

For the legacy PyBullet workspace, specify its actual path:

```bash
export LBR_TUTORIAL_LEGACY_WS=/path/to/ros2_pybullet_ws
./scripts/run_demo.sh joint sim_backend:=existing
```

7. Validation and acceptance

The standalone core was built and passed 54 unit checks in the recorded retest. In an isolated environment sourcing only system ROS and this repository, GUI joint and Cartesian demos passed with real OpenGL captures, logs, and error reports. External mode passed eight checks: both demos, one pose, duplicate-name rejection, unreachable-pose rejection, invalid-frame rejection, busy rejection, and cancellation.

The source ZIP was rebuilt and tested end to end in a fresh directory. Evidence is retained under docs/evidence/standalone_gui, standalone_external, and portability, with standalone_unit_tests.txt. An initial GUI tracking-error failure and later passing runs are both preserved. See docs/validation.md and the JSON reports for exact measurements and definitions. Legacy-backend, feedback-interruption, and upstream-client records remain identified as historical snapshots.

Run ./scripts/test.sh to create new reports. Demo acceptance requires action success, at least five tracking samples, final position error below 3 mm, final joint error below 0.02 rad, final orientation error below 0.01 rad, and sampled maximum position tracking error below 15 mm. Cartesian acceptance additionally requires sampled geometric path deviation below 3 mm. A feedback timeout latches a fault and requires bringup to be restarted.

8. Student implementation sequence

T0 build and activate controllers; T1 joint targets and feedback; T2 FK/IK and Cartesian paths; T3 topics, status, and cancellation; T4 separate simulation/control and compare the existing backend; T5 implement the application adapter after validated coordinate and grasp outputs exist.

Application milestones follow in order. M1 connects llm_ros and sam3_ros and validates instructions and masks. M2 aligns RGB-D, reconstructs 3D points, and calibrates camera-to-base coordinates. M3 generates grasp candidates and checks TCP, IK, joint, collision, and gripper constraints. M4 serially approaches, grasps, transfers, releases, and verifies physical results. M5 freezes evaluation scenes and records success, error, latency, and failure causes. Retain inputs, configuration, logs, and results for each stage, and pass its review before connecting the next.

9. English desktop workbench

Start Robot Workbench with ./scripts/run_workbench.sh --start. Import a robot URDF and an instrument URDF/STL/OBJ, select the mounting link, and configure translation, rotation, scale, and TCP. Apply & Reload updates the simulator owned by the window. The centre shows live simulation; joint/TCP controls and Robot Monitor show motion and feedback. Save and reopen JSON projects.

View > Legacy Robot Monitor selects the original monitor and workspace and embeds the actual Qt6 window on X11. Compatible seven-axis models support trajectory control; other supported joint counts display Preview Only. Instruments are rigid attachments, and the fixed gripper example has no finger actuator. The recorded workbench release includes 82 passing unit tests; see docs/validation.md for UI evidence.
