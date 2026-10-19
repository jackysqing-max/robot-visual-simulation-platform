# Robot Visual Simulation Platform

**Robot Workbench** is an English desktop application for robot model import, interchangeable instruments, TCP configuration, ROS 2 motion control, and live PyBullet visualization.

[中文说明](README.zh-CN.md) · [Workbench guide](docs/workbench.md) · [ROS interfaces](docs/interfaces.md) · [Validation reports](docs/validation.md)

![Robot Workbench with a probe and live Robot Monitor](docs/evidence/workbench/probe.png)

## Features

- Import robot URDFs and rigid end effectors from URDF, STL, or OBJ files.
- Set the flange, instrument scale, mounting transform, and tool centre point; save and reopen JSON projects.
- View the actual simulated robot through an embedded camera with orbit and zoom controls.
- Execute joint goals and Cartesian TCP trajectories through the LBR-configured ROS 2 joint trajectory controller.
- Observe measured positions, velocities, efforts, and tracking error in a dockable Robot Monitor.
- Optionally embed the original C++ Qt6 Robot Monitor as a native child window on X11.
- Run simulation and control as independent ROS nodes and connect other application modules through standard ROS interfaces.

Trajectory control currently supports compatible seven-axis robots with bounded revolute joints in one serial chain. Other supported models are available in **Preview only** mode. Instruments are rigid attachments; the gripper example has no finger actuator. Hardware execution and collision-aware motion planning are outside the current implementation.

## Install

The tested baseline is Ubuntu 22.04, ROS 2 Humble, and system Python 3.10. Install ROS 2 Humble first, then:

```bash
git clone https://github.com/jackysqing-max/robot-visual-simulation-platform.git
cd robot-visual-simulation-platform

sudo apt-get update
sudo apt-get install -y build-essential python3-colcon-common-extensions \
  python3-numpy python3-pip python3-pyqt5 python3-pytest \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-robot-state-publisher
/usr/bin/python3 -m pip install --user -r requirements.txt
./scripts/setup.sh
```

Use a ROS-compatible system Python environment. The standalone simulator and built-in monitor require no external project checkout, LLM service, or FRI SDK. The original Qt6 monitor is an optional integration with its own installed workspace.

## Open the desktop platform

```bash
./scripts/run_workbench.sh --start
```

Open a supplied project:

```bash
./scripts/run_workbench.sh --project examples/probe_project.json --start
# Alternatives: examples/gripper_project.json or examples/scalpel_project.json
```

Run one platform instance per ROS domain. The default domain is 87; set another domain when needed:

```bash
ROS_DOMAIN_ID=88 ./scripts/run_workbench.sh --start
```

Import the robot and instrument in the left panel, configure the mount and TCP, and select **Apply & Reload**. Use the joint or Cartesian TCP controls and observe the monitor. Project edits are staged until applied. See the [English workbench guide](docs/workbench.md) for units, compatibility rules, and the original-monitor integration.

## Run without the desktop platform

```bash
./scripts/run_demo.sh joint
./scripts/run_demo.sh cartesian
```

Add `gui:=true` to open PyBullet's own visualizer. To separate the simulator and controller, run these in two terminals using the same ROS domain:

```bash
# Terminal A
./scripts/run_demo.sh sim gui:=true

# Terminal B
./scripts/run_demo.sh bringup sim_backend:=external
```

## ROS integration

| Interface | Type | Purpose |
| --- | --- | --- |
| `/lbr_tutorial/joint_goal` | `sensor_msgs/msg/JointState` | Seven joint position targets, radians |
| `/lbr_tutorial/pose_goal` | `geometry_msgs/msg/PoseStamped` | Configured TCP target for workbench projects |
| `/lbr_tutorial/cartesian_path` | `nav_msgs/msg/Path` | Cartesian TCP path |
| `/lbr_tutorial/status` | `std_msgs/msg/String` | JSON task events and request IDs |
| `/lbr_tutorial/cancel` | `std_srvs/srv/Trigger` | Cancel the active task |
| `/lbr/joint_states` | `sensor_msgs/msg/JointState` | Measured joint feedback |
| `/lbr/ee_pose` | `geometry_msgs/msg/PoseStamped` | Measured flange pose |
| `/lbr/tcp_pose` | `geometry_msgs/msg/PoseStamped` | Measured configured TCP pose |
| `/lbr/sim/config` | `std_msgs/msg/String` | Applied model and instrument configuration |
| `/lbr/sim/camera/image` | `sensor_msgs/msg/Image` | Live simulation camera |

Source `install/local_setup.bash` in participating ROS terminals and use the same `ROS_DOMAIN_ID`. The retained package names are `lbr_pybullet_control` and `lbr_pybullet_tutorial`. See [interface details](docs/interfaces.md) and the [workbench guide](docs/workbench.md) for frame conventions and task sequencing.

## Architecture

```text
Robot / instrument project
  → assembled URDF and explicit TCP
  → PyBullet physics + measured feedback + camera

ROS goals / desktop controls
  → joint trajectory or Cartesian sampling + numerical IK
  → FollowJointTrajectory → official joint_trajectory_controller
  → ros2_control SystemInterface → PyBullet position servo
```

The project reuses the LBR Stack controller configuration and original trajectory client at commit `52f816bd59cbd2390efc890cca0ad61389e4ccb3`. The controller implementation comes from ROS `ros2_controllers`; the TCP sampling and numerical IK are implemented here. Imported models keep their original geometry and use an explicit joint-name mapping.

## Validation

```bash
./scripts/test.sh
source install/local_setup.bash
/usr/bin/python3 scripts/workbench_smoke_test.py --output artifacts/workbench_check
```

The recorded release includes **82 passing unit tests**, a real desktop test covering joint motion, probe replacement, TCP motion, project save/open and six-axis preview, and actual X11 embedding of the original Robot Monitor. Reports, screenshots, and failure records are retained in [validation documentation](docs/validation.md). These results describe the tested simulation configurations.

## Repository layout

```text
src/lbr_pybullet_control/    C++ ros2_control hardware adapter
src/lbr_pybullet_tutorial/   Simulator, FK/IK, motion node, desktop UI and launches
examples/                   Probe, gripper and OBJ instrument projects
scripts/                    Build, run, test and export tools
docs/                       Guides and frozen validation evidence
third_party/lbr_stack/      Pinned upstream client, license and provenance
```

## License

Original implementation: [Apache-2.0](LICENSE). See [NOTICE](NOTICE) and [upstream provenance](third_party/lbr_stack/provenance.json) for retained LBR Stack files. The PyBullet robot model is loaded from the installed dependency; its BSD notice is preserved in generated descriptions.
