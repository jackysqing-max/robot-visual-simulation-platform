LBR Stack × PyBullet 独立运控项目与学生实施说明

1. 项目目标

建立可独立构建、调试和可视化运行的 ROS 2 运控仓库 lbr_pybullet_ros。其他节点发送关节位置、末端位姿或笛卡尔路径，本模块负责转换、执行并反馈结果，作为 llm_ros、sam3_ros 与后续器械分拣任务之间的运动执行层。

本地仓库：/home/siqin/ros2_workspaces/lbr_pybullet_ros。仓库名称独立，ROS 包名保留 lbr_pybullet_control、lbr_pybullet_tutorial，接口保留 /lbr_tutorial，方便与原教程兼容。GitHub 独立仓库：https://github.com/jackysqing-max/robot-visual-simulation-platform 。本地源码仓库及压缩包继续保留。

2. 实际控制链路

ROS 2 目标 → 轨迹生成/数值 IK → FollowJointTrajectory → joint_trajectory_controller（JTC）→ ros2_control 硬件插件 → PyBullet 位置伺服与物理步进 → 实测关节反馈。

本项目复用 LBR Stack 的控制器配置、标准 action 接口及原始关节轨迹客户端；JTC 来自 ROS 官方 ros2_controllers。末端轨迹的采样、四元数 SLERP 和数值 IK 由本项目实现。关节 demo 提供零端点速度，JTC 采用三次插值；笛卡尔路径转换为仅含位置的关节采样点，JTC 在点间线性插值。物理反馈参与控制器的状态与误差监视。

默认模型为 PyBullet 内置七轴 iiwa。末端定义为 link7，没有附加工具 TCP；world 与 lbr_link_0 重合。当前完成机械臂仿真运控，夹爪、避障、抓取结果验证与真机迁移属于后续阶段。

3. 工程组成与安装

src/lbr_pybullet_control：C++ SystemInterface，连接 JTC 命令与 PyBullet 实测状态。
src/lbr_pybullet_tutorial：仿真、FK/IK、目标接收、demo、launch 与配置。
scripts：安装构建、启动、自动检查及轨迹绘图。
docs：接口、快速开始、学生计划和验证证据。
third_party/lbr_stack：固定版本的上游配置、客户端、来源及许可记录。

环境为 Ubuntu 22.04、ROS 2 Humble 与系统 Python 3。新机器先安装 ROS 2 Humble，再执行：

```bash
cd /home/siqin/ros2_workspaces/lbr_pybullet_ros
sudo apt-get install ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-robot-state-publisher python3-colcon-common-extensions \
  python3-numpy python3-pip python3-pytest
/usr/bin/python3 -m pip install --user -r requirements.txt
./scripts/setup.sh
```

默认后端全部源码在本仓库，无需旧工作区或完整 LBR Stack/FRI SDK。

4. 独立运行与 GUI 调试

以下两个 demo 依次运行，每次自动启动系统、执行检查、保存报告并关闭本次启动的系统：

```bash
./scripts/run_demo.sh joint gui:=true
./scripts/run_demo.sh cartesian gui:=true
```

gui:=false 使用无窗口模式。GUI 需要可用桌面显示；窗口显示关节运动与绿色实测末端轨迹，可调整视角观察。Cartesian demo 依据当前实测关节的 FK 生成边长 3 cm 的闭合方形，保持姿态。报告保存到 artifacts/*_report.json。

持续观察和手动发目标时，终端 A 启动：

```bash
./scripts/run_demo.sh bringup gui:=true
```

终端 B 进入仓库，执行以下环境设置；后续所有 ros2 命令均使用相同设置：

```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=87
ros2 control list_controllers -c /lbr/controller_manager
ros2 topic echo /lbr_tutorial/status
```

两项控制器应为 active。另一个已设置环境的终端可运行 ros2 run lbr_pybullet_tutorial demo --mode joint 或 --mode cartesian，并观察 /iiwa7/joint_states 与 /lbr/joint_trajectory_controller/state。

5. 对外 ROS 2 接口与消息示例

/lbr_tutorial/joint_goal：sensor_msgs/msg/JointState，完整七轴位置，单位 rad。
/lbr_tutorial/pose_goal：geometry_msgs/msg/PoseStamped，单个 link7 位姿，单位 m。
/lbr_tutorial/cartesian_path：nav_msgs/msg/Path，按顺序经过的末端位姿。
/lbr_tutorial/status：std_msgs/msg/String，JSON 任务事件与 request_id。
/lbr_tutorial/cancel：std_srvs/srv/Trigger，取消当前任务。
/lbr/joint_trajectory_controller/follow_joint_trajectory：标准 FollowJointTrajectory action，适合精确指定多点时间。
/lbr/joint_states、/lbr/joint_trajectory_controller/state：关节反馈及期望/实际/误差。
/lbr/ee_pose：默认仿真发布的 link7 实测位姿。

系统 ready 后发送关节目标：

```bash
ros2 topic pub --once /lbr_tutorial/joint_goal sensor_msgs/msg/JointState \
  "{name: [lbr_A1, lbr_A2, lbr_A3, lbr_A4, lbr_A5, lbr_A6, lbr_A7], position: [0.1, 0.4, 0.0, -1.1, 0.0, 0.9, 0.0]}"
```

末端目标必须来自当前模型的 FK 或经验证的规划结果。以下可运行示例读取 /lbr/ee_pose，在当前位姿上沿 y 移动 1 cm；将 mode 改为 path 可发送“移动后返回”的路径。运行前等待上一任务结束：

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
mode = 'pose'  # 改为 'path'，运行另一种输入示例
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

位姿 frame 只接受 world 或 lbr_link_0；目标 stamp 必须为零；姿态为归一化 x/y/z/w 四元数。相机坐标与工具 TCP 需要应用层先转换。观察同一 request_id 的最终 succeeded、failed 或 rejected，accepted 仅表示开始提交。忙碌时新目标被拒绝。取消命令：

```bash
ros2 service call /lbr_tutorial/cancel std_srvs/srv/Trigger '{}'
```

6. 与其他节点组合

其他模块在同一 ROS_DOMAIN_ID 中直接发布上述目标并订阅状态，无需导入本项目的 Python 源码。应用应串行提交目标、关联运控 request_id、等待最终结果后推进任务；直接使用底层 action 时，可使用 action goal UUID 关联执行结果。

也可拆分仿真与控制。先在终端 A 完成相同环境设置并启动仿真：

```bash
ros2 launch lbr_pybullet_tutorial sim.launch.py gui:=true
```

终端 B 启动控制器和目标接收节点，连接已经运行的兼容仿真：

```bash
./scripts/run_demo.sh bringup sim_backend:=external
```

仿真侧须接收 /iiwa7/joint_desired（Float64MultiArray，七轴 rad），持续发布 /iiwa7/joint_states（JointState，iiwa_joint_1～7，完整 position/velocity 和递增时间戳），使用相同模型与关节映射。每台仿真只运行一个实例，命令仅由一套执行链路写入。

兼容原有 PyBullet 工作区时显式设置路径：

```bash
export LBR_TUTORIAL_LEGACY_WS=/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws
./scripts/run_demo.sh joint sim_backend:=existing
```

7. 验证与验收

独立仓库已完成构建与 54 项单元检查；仅加载系统 ROS 和本仓库安装目录的环境中，PyBullet GUI 的关节与笛卡尔 demo 均通过，并保存真实 OpenGL 窗口截图、控制器日志与误差报告。分离运行仿真和控制节点的 external 模式也通过 8 项检查：两个 demo、单个位姿、重复关节名、不可达位姿、错误坐标系、busy 和取消。

源码 ZIP 已在全新目录重新构建并完成端到端检查。新证据保存在 docs/evidence/standalone_gui、standalone_external 与 portability，单元记录为 standalone_unit_tests.txt。GUI 曾有一次跟踪误差超限，失败记录与后续通过记录一并保存。精确指标及测量定义见 docs/validation.md 和对应 JSON。原教程的旧后端、反馈中断及上游客户端记录另行保留，标记为拆分前快照。

运行 ./scripts/test.sh 保存新报告。demo 验收包含 action 成功、终点位置误差 <3 mm、终点关节误差 <0.02 rad、终点姿态误差 <0.01 rad、采样最大位置跟踪误差 <15 mm；Cartesian demo 另要求采样路径最大几何偏差 <3 mm。具体测量定义见验证文档。反馈超时会锁定故障，需要重新启动 bringup。

8. 学生执行顺序

T0 构建与控制器激活 → T1 关节目标及反馈 → T2 FK/IK 与笛卡尔轨迹 → T3 topic、状态、取消 → T4 仿真与控制拆分、旧后端接入 → T5 编写应用适配器。

随后按 M1～M5 推进：M1 接入 llm_ros 和 sam3_ros，校验指令与分割结果；M2 对齐 RGB-D、恢复三维并标定相机到基座；M3 生成抓取候选，检查 TCP、IK、关节和碰撞约束；M4 用状态机串行执行接近、夹持、搬运与释放，并验证物理结果；M5 固定场景评估成功率、误差、延迟和失败原因。每阶段保存输入、配置、日志及结果，上一阶段验收后再接入下一阶段。

9. 全英文桌面 UI

使用 ./scripts/run_workbench.sh --start 启动 Robot Workbench。左侧导入机器人 URDF 和末端 URDF/STL/OBJ，选择安装连杆，配置安装位置、角度、缩放及 TCP；Apply & Reload 更新自己启动的仿真。中心显示真实仿真画面，下方提供关节/TCP 控制和 Robot Monitor。可以保存及重新打开 JSON 项目。View → Legacy Robot Monitor 可选择原有监控器与工作区，把实际 Qt6 窗口嵌入子模块。七轴兼容模型提供轨迹控制，其他轴数明确显示 Preview Only。末端以刚性器械装配，固定夹爪示例不含手指驱动。
