# 快速开始

## 环境与构建

目标环境为 Ubuntu 22.04、系统 Python 3、ROS 2 Humble。PyBullet 和 ROS Python 节点需要运行在能够导入 `rclpy` 的同一个 Python 环境。首次执行：

```bash
cd /home/siqin/ros2_workspaces/lbr_pybullet_ros
./scripts/setup.sh
```

脚本负责检查依赖并构建本工程。缺少系统依赖时，按脚本输出安装，再重新运行。不要在缺少系统 ROS 库的任意 Conda 环境中启动节点。

新环境可先安装所需 ROS 控制组件，再安装 PyBullet：

```bash
sudo apt-get install ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-robot-state-publisher python3-colcon-common-extensions \
  python3-numpy python3-pip python3-pytest
/usr/bin/python3 -m pip install --user -r requirements.txt
./scripts/setup.sh
```

## 最短运行路径

先运行关节 demo，成功退出后再运行笛卡尔 demo：

```bash
./scripts/run_demo.sh joint
./scripts/run_demo.sh cartesian
```

需要图形窗口时，在有桌面显示的终端运行：

```bash
./scripts/run_demo.sh joint gui:=true
```

`joint` demo 发送七个关节的目标位置，等待控制器完成并检查物理反馈；`cartesian` demo 根据当前末端位姿生成小范围轨迹，经逐点 IK 后发送给同一个轨迹 action。关节 demo 验证输入到伺服通路，笛卡尔 demo 额外验证运动学和末端轨迹。

脚本保存 `artifacts/joint_report.json`、`artifacts/cartesian_report.json` 以及对应的 `*_launch.log`。demo 完成后脚本会停止自己启动的 launch。若已经有系统在运行，使用下面的“分终端运行”方法，避免重复启动。

## 分终端运行与观察

先只启动系统：

```bash
cd /home/siqin/ros2_workspaces/lbr_pybullet_ros
./scripts/run_demo.sh bringup gui:=true
```

另一个终端设置相同环境：

```bash
cd /home/siqin/ros2_workspaces/lbr_pybullet_ros
source /opt/ros/humble/setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=87
ros2 control list_controllers -c /lbr/controller_manager
ros2 action list -t
```

应看到处于 `active` 状态的 `joint_state_broadcaster` 与 `joint_trajectory_controller`，以及 `/lbr/joint_trajectory_controller/follow_joint_trajectory` action。之后运行：

```bash
ros2 run lbr_pybullet_tutorial demo --mode joint --timeout 60 --report /tmp/lbr_joint_report.json
ros2 run lbr_pybullet_tutorial demo --mode cartesian --timeout 60 --report /tmp/lbr_cartesian_report.json
```

这些命令仍需依次完成。`--report` 将本次结果写入指定 JSON 文件，便于保存与比较；成功条件和可解释范围见 [验证文档](validation.md)。

另开一个完成相同 source 和 domain 设置的终端观察：

```bash
ros2 topic echo /lbr_tutorial/status
ros2 topic echo /iiwa7/joint_states
ros2 topic echo /lbr/joint_trajectory_controller/state
```

物理反馈来自 `/iiwa7/joint_states`；控制器状态同时包含期望、实际和误差。只看到目标被发布不足以说明跟踪成功。

## 发送 ROS 2 关节目标

系统已启动时，可直接从命令行发消息：

```bash
ros2 topic pub --once /lbr_tutorial/joint_goal sensor_msgs/msg/JointState \
  "{name: [lbr_A1, lbr_A2, lbr_A3, lbr_A4, lbr_A5, lbr_A6, lbr_A7], position: [0.1, 0.4, 0.0, -1.1, 0.0, 0.9, 0.0]}"
```

单位为弧度。节点收到完整目标后组装轨迹，由控制器执行。学生应先使用默认 demo，随后再更改数值并观察反馈。输入关节名称必须完整且唯一，目标须位于模型关节限制内。

末端输入使用 `/lbr_tutorial/pose_goal` 或 `/lbr_tutorial/cartesian_path`。实际数值应来自当前机器人 FK 或经过验证的目标，不应猜测一个 XYZ 和四元数就直接解释为可达目标。首个笛卡尔实验使用 `demo --mode cartesian`；消息字段和坐标约定见 [接口文档](interfaces.md)。

默认后端会发布 `/lbr/ee_pose`，可先查看实际末端位姿：

```bash
ros2 topic echo --once /lbr/ee_pose
```

复制其中的 pose、使用 `frame_id: lbr_link_0` 并把 `stamp` 清零，即可构造保持当前位置的 `pose_goal`；之后只在一个方向增加约 0.01 m 做小位移实验。反馈中的采集时间戳不能原样用作目标时间戳。`existing` 后端没有该输出 topic，需从 `/lbr/joint_states` 使用相同模型计算 FK。

需要请求取消时：

```bash
ros2 service call /lbr_tutorial/cancel std_srvs/srv/Trigger '{}'
```

取消后观察状态返回，待系统空闲再提交下一个目标。

## 直接运行 LBR Stack 原始客户端

在 `bringup` 已运行、环境和 domain 已设置好的第二终端执行：

```bash
/usr/bin/python3 scripts/upstream_joint_demo.py --ros-args -r __ns:=/lbr
```

包装脚本直接加载保留原样的 `third_party/lbr_stack/joint_trajectory_client.py` 并调用其 `main()`。上游范例会先把奇数轴转到 1 rad，再回到全零，每段约 15 s，总计约 30 s。命名空间重映射使其相对 action 名落到 `/lbr/joint_trajectory_controller/follow_joint_trajectory`。

这条路径直接验证上游客户端 → 同一个 JTC → 自定义硬件插件 → PyBullet；它不经过 `/lbr_tutorial/joint_goal` 的任务接收节点，因此运行期间不要同时提交其他目标。包装脚本补充运行入口，没有改写上游客户端算法。

## 使用已有 PyBullet 节点

默认后端是本教程自己的 PyBullet 节点。要测试当前 `ros2_pybullet_ws` 中的仿真实现，先 source 其已构建的 overlay，再 source 本教程：

```bash
cd /home/siqin/ros2_workspaces/lbr_pybullet_ros
source /opt/ros/humble/setup.bash
source /home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/install/local_setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=87
ros2 launch lbr_pybullet_tutorial tutorial.launch.py sim_backend:=existing gui:=false
```

该模式由 launch 启动已有仿真节点，要求已有 ROS 包已经安装。启动配置使用关节位置控制 `initial_control_mode:=1`、禁用旧 `goto` 控制 `use_goto:=false`，并使用与本教程相同的复位初始关节位置。这样 `/iiwa7/joint_desired` 是唯一的轨迹执行输入，不会同时运行两套相互覆盖的插值器。

已有节点为 `pybullet_ros2_sim/iiwa_pybullet_sim_node`。启动参数名为 `reset_to_init_on_start:=true`，初始关节值为 `[0.0, 0.35, 0.0, -0.9, 0.0, 0.65, 0.0]` rad。脚本也支持直接运行：

```bash
./scripts/run_demo.sh joint sim_backend:=existing
./scripts/run_demo.sh cartesian sim_backend:=existing
```

若已有工作区放在其他位置，设置 `LBR_TUTORIAL_LEGACY_WS=/实际路径` 后再调用脚本。

不要另外再启动同一台 `iiwa7` 的仿真节点。若已有仿真以其他命名空间、消息类型、关节数量或模型启动，需要先修改适配配置并检查对应关系，不能仅凭 topic 名称相同认定兼容。

在第二个终端同样 source 两个 overlay，然后运行两个 demo。完成后分别用 `Ctrl+C` 停止 demo 与 launch。`existing` 的通过结果需要单独记录，它不能用默认后端的结果替代。

## 常见问题

| 现象 | 检查方法 |
|---|---|
| 找不到 `ros2` 或 Python 无法导入 `rclpy` | 使用系统 Python，source `/opt/ros/humble/setup.bash` |
| 找不到本教程的包 | 先成功执行 `setup.sh`，再 source `install/local_setup.bash` |
| demo 等不到 action 或状态 | 确认所有终端 `ROS_DOMAIN_ID` 一致，检查 launch 进程与控制器状态 |
| 控制器等待硬件状态 | 检查 `/iiwa7/joint_states` 是否持续更新，确认没有重复仿真或错误命名空间 |
| action 存在但系统不 ready | 同时确认实测关节状态与 `/lbr/joint_trajectory_controller/state` 新鲜，action 存在本身不足以表示控制器正在更新 |
| 目标返回 busy | 等待原目标完成或调用取消服务；新消息不会抢占正在执行的任务 |
| 位姿被拒绝 | 检查 frame、零时间戳、有限数值与有效四元数，再检查 IK、关节限制 |
| 提示 `feedback/action timeout` | 系统已锁定该故障，恢复反馈后仍需停止并重新启动 tutorial bringup |
| GUI 无法打开 | 先用 `gui:=false` 验证链路；GUI 需要可用的桌面显示环境 |
| ROS 报共享内存或发现异常 | 先确认项目使用独立 domain；不要停止本工作区中其他任务的节点 |

依次排查：仿真状态持续更新 → 硬件接口已激活 → 控制器 active → action 存在 → demo 的 `accepted` 与最终状态 → 物理反馈和误差报告。
