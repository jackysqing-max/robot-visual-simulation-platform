# lbr_pybullet_ros

可独立构建、调试和可视化运行的 ROS 2 Humble 机械臂运控仓库。接收关节目标、末端位姿或笛卡尔路径，使用 LBR Stack 配置的官方 `joint_trajectory_controller`（JTC）执行轨迹，通过 `ros2_control` 驱动 PyBullet 七轴 iiwa，并反馈实际物理状态。

默认只依赖本仓库、系统 ROS 2 和 Python 依赖。`llm_ros`、`sam3_ros` 及应用规划器通过 ROS 节点通信接入。ROS 包名保留 `lbr_pybullet_control`、`lbr_pybullet_tutorial`，原有 topic 保持兼容。

[Google 文档可粘贴正文](docs/google_docs_copy.txt) · [DOCX 版本](docs/google_docs_copy.docx) · [快速开始](docs/quickstart.md) · [接口](docs/interfaces.md) · [学生计划](docs/student_plan.md) · [验证记录](docs/validation.md)

## 安装与可视化

在已安装 ROS 2 Humble 的 Ubuntu 22.04 上：

```bash
sudo apt-get install ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-robot-state-publisher python3-colcon-common-extensions \
  python3-numpy python3-pip python3-pytest
/usr/bin/python3 -m pip install --user -r requirements.txt
./scripts/setup.sh
```

依次运行两个可视化 demo；每次自动启动系统，执行并保存报告后关闭：

```bash
./scripts/run_demo.sh joint gui:=true
./scripts/run_demo.sh cartesian gui:=true
```

`gui:=false` 用于无窗口测试。GUI 显示机器人与绿色实测末端轨迹，可调整视角观察。报告和日志在 `artifacts/`。

持续运行完整系统，允许其他节点发送目标：

```bash
./scripts/run_demo.sh bringup gui:=true
```

第二终端进入仓库后设置环境，运行 demo 或自己的应用节点：

```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=87
ros2 run lbr_pybullet_tutorial demo --mode joint
```

所有通信节点使用相同 domain；脚本默认为 87。

## 拆分节点调试

终端 A 只运行 PyBullet 物理和可视化节点：

```bash
./scripts/run_demo.sh sim gui:=true
```

终端 B 启动控制器与任务接收节点，连接已经运行的仿真：

```bash
./scripts/run_demo.sh bringup sim_backend:=external
```

两组进程可以分别调试和重启。仿真持续发布 `/iiwa7/joint_states`，接收 `/iiwa7/joint_desired`。每台仿真只启动一个实例，指令由一条运控链路写入。上层目标 topic 由任务接收节点提供。

兼容原来的 `pybullet_ros2_sim` 时显式选择旧后端：

```bash
export LBR_TUTORIAL_LEGACY_WS=/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws
./scripts/run_demo.sh joint sim_backend:=existing
```

`existing` 启动旧仿真；`external` 连接已运行的兼容仿真。默认模式不会自动 source 其他工作区。

## ROS 接口

| 接口 | 类型 | 用途 |
|---|---|---|
| `/lbr_tutorial/joint_goal` | `sensor_msgs/msg/JointState` | 七轴位置目标，rad |
| `/lbr_tutorial/pose_goal` | `geometry_msgs/msg/PoseStamped` | 单个末端位姿，m |
| `/lbr_tutorial/cartesian_path` | `nav_msgs/msg/Path` | 末端路径 |
| `/lbr_tutorial/status` | `std_msgs/msg/String` | JSON 任务状态与 request_id |
| `/lbr_tutorial/cancel` | `std_srvs/srv/Trigger` | 取消当前任务 |
| `/lbr/joint_states` | `sensor_msgs/msg/JointState` | 实测关节反馈 |
| `/lbr/ee_pose` | `geometry_msgs/msg/PoseStamped` | 默认仿真实测末端位姿 |
| `/lbr/joint_trajectory_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | 精确指定多点时间的标准 action |

一次执行一个目标。应用等待最终结果后再提交下一目标。末端 frame 为 `world` 或 `lbr_link_0`，目标 stamp 为零，四元数为 xyzw。相机坐标与工具 TCP 由应用层转换。完整消息示例见 [Google 文档正文](docs/google_docs_copy.md)。

## 实现与目录

```text
ROS 2 目标 → 关节轨迹 / 笛卡尔采样与数值 IK
  → FollowJointTrajectory → 官方 JTC → 自定义 SystemInterface
  → PyBullet 电机伺服与物理步进 → 实测关节反馈

src/lbr_pybullet_control/     C++ ros2_control 硬件插件
src/lbr_pybullet_tutorial/    仿真、运动学、目标接收、demo 和 launch
scripts/                     构建、启动、验证与打包
third_party/lbr_stack/       固定版本的上游客户端、许可与来源
```

上游 LBR Stack 固定为 `52f816bd59cbd2390efc890cca0ad61389e4ccb3`。复用其控制器配置和原始客户端；JTC 来自 ROS 官方 `ros2_controllers`，数值 IK 由本项目实现。默认模型为 `pybullet_data/kuka_iiwa/model.urdf`，末端为 link7，没有附加夹爪 TCP，也未连接真实机器人。

## 验证与源码导出

```bash
./scripts/test.sh
source install/local_setup.bash
/usr/bin/python3 scripts/feedback_fault_test.py
/usr/bin/python3 scripts/gui_smoke_test.py
/usr/bin/python3 scripts/package_source.py
```

测试范围、真实 GUI 截图与独立节点联调结果见 [验证记录](docs/validation.md)。源码 ZIP 排除本机的 build/install/log 和 Git 内部文件，包含文档及冻结证据。

抓取、避障、夹爪与真机迁移按 [学生计划](docs/student_plan.md) 后续接入。许可见 [LICENSE](LICENSE)，上游来源见 [NOTICE](NOTICE)。
