# 接口、坐标与插值

## 对外输入与输出

| 接口 | 类型 | 含义 |
|---|---|---|
| `/lbr_tutorial/joint_goal` | `sensor_msgs/msg/JointState` | 完整七轴关节位置目标，单位 rad |
| `/lbr_tutorial/pose_goal` | `geometry_msgs/msg/PoseStamped` | 第七连杆的单个末端位姿，位置单位 m |
| `/lbr_tutorial/cartesian_path` | `nav_msgs/msg/Path` | 依次经过的末端位姿 |
| `/lbr_tutorial/status` | `std_msgs/msg/String` | JSON 状态事件，包含节点分配的 `request_id` |
| `/lbr_tutorial/cancel` | `std_srvs/srv/Trigger` | 请求取消当前任务 |
| `/lbr/joint_trajectory_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | 底层标准轨迹执行接口 |
| `/lbr/ee_pose` | `geometry_msgs/msg/PoseStamped` | 默认仿真后端的实测 link7 位姿；existing 后端不提供 |
| `/lbr/joint_states` | `sensor_msgs/msg/JointState` | 从实测硬件状态广播的 LBR 关节状态 |
| `/lbr/joint_trajectory_controller/state` | `control_msgs/msg/JointTrajectoryControllerState` | Humble JTC 的期望、实际与误差状态 |

`accepted` 表示教学节点已生成轨迹并开始提交 action；随后控制器仍可能拒绝并返回 `failed`。应用需要等待同一 ID 的最终事件。取消服务的成功响应表示已请求取消，最终 action 状态另行报告；没有活动任务时返回 `success: false`。

目标接收节点一次执行一个任务。busy 时拒绝新输入，不抢占当前目标。状态事件为 `ready`、`accepted`、`succeeded`、`rejected`、`failed`；每个请求用自己的 `request_id` 关联。busy 拒绝对应新请求的 ID，不应被应用层当成原任务的失败结果。

启动就绪同时要求：最新实测关节状态、持续前进的原始 PyBullet 状态时间戳、新鲜 JTC 状态和可用的 action server。状态广播器可能重复发布最后一次读到的状态，因此只看 `/lbr/joint_states` 的到达频率不足以证明物理反馈仍在更新。节点还独立监视 `/iiwa7/joint_states` 的源时间戳。任务执行期间若状态或 action 超时，节点报告 `failed` 并锁定故障；后续目标会被拒绝，需重新启动 tutorial bringup 恢复。它不会在状态中断后自动恢复运动。

`JointState` 在这里用作位置目标的输入容器；它不是带时间的轨迹协议。`name` 必须覆盖 `lbr_A1`–`lbr_A7` 且不能重复，`position` 与名称一一对应；不要用 `velocity` 或 `effort` 传附加运控指令。需要精确指定多点轨迹时间时，直接使用标准 `FollowJointTrajectory` action，并避免同时从教学目标 topic 提交任务。

硬件侧 `/iiwa7/joint_desired` 是 `std_msgs/msg/Float64MultiArray`，数组为按序排列的七轴位置；`/iiwa7/joint_states` 是 `sensor_msgs/msg/JointState`，名称为 `iiwa_joint_1`–`iiwa_joint_7`，必须含完整位置与速度。桥接器按关节名称读取反馈，而不是依赖 JointState 消息的数组顺序。

## 坐标与时间

- 末端是 `pybullet_data/kuka_iiwa/model.urdf` 的第七连杆原点，PyBullet link index 为 6，原模型名为 `lbr_iiwa_link_7`，生成的 ROS 描述名为 `lbr_link_7`。使用 link frame，未额外叠加夹爪 TCP，也不使用连杆质心 frame。
- `PoseStamped.header.frame_id` 只接受 `world` 或 `lbr_link_0`，两者在本教程内是同一静止基座坐标系。
- `Path.header.frame_id` 使用相同约定。各 `PoseStamped` 的 frame 可以为空以继承 Path 的 frame；显式填写时必须与路径坐标约定一致。
- 四元数采用 ROS 的 `x, y, z, w` 顺序，数值须有限且范数与 1 的差不超过 0.001；避免把欧拉角直接填入四元数字段。
- 目标 header 的时间戳必须为零。节点按接收时刻立即规划，非零时间戳会被拒绝；接口不支持按消息时间调度未来运动或重放历史位姿。

相机坐标下的点或抓取姿态必须先经过应用层 tf2 转换，才能发送到这里。当前接口不会查询相机 TF，也不提供相机标定。传入 `camera_*` frame 会被拒绝。

位姿消息结构如下；其中 `x, y, z, qx, qy, qz, qw` 应由 FK、轨迹生成器或经过标定的规划结果填入，不能把这些占位符直接发布：

```yaml
header:
  frame_id: lbr_link_0
  stamp: {sec: 0, nanosec: 0}
pose:
  position: {x: x, y: y, z: z}
  orientation: {x: qx, y: qy, z: qz, w: qw}
```

Path 使用同一 `header`，并在 `poses` 数组里按顺序给出多个上述位姿。时间参数由教学轨迹生成器分配；Path 的 stamp 不表示每个点的到达时刻。

## 从关节目标到物理反馈

1. 教学节点读取最新实测关节位置，校验输入并生成 `JointTrajectory`。
2. action 将完整轨迹交给实际运行的官方 `joint_trajectory_controller`。
3. 控制器在自己的更新周期内计算当前期望关节位置，向 `ros2_control` position command interface 写入值。
4. C++ `SystemInterface.write()` 把命令转换到 `/iiwa7/joint_desired`。
5. PyBullet 用关节位置伺服驱动模型，并持续运行物理步进。
6. `/iiwa7/joint_states` 携带实测位置和速度，`SystemInterface.read()` 将其提供给控制器。

这是一条以物理状态反馈的通路。`joint_state_broadcaster` 的状态来自硬件接口读到的仿真状态，而不是把命令赋值给当前位置。调试时同时观察控制器误差和 `/iiwa7/joint_states`，不能只读 `/iiwa7/joint_desired`。

## 笛卡尔轨迹如何执行

笛卡尔输入先在位置空间采样、在姿态空间使用四元数 SLERP，再为每个采样位姿求数值 IK。后续 IK 使用前一关节解作为参考，减少解支跳变。所有点转换成功且通过限制后，才会发送整段关节轨迹。

这一步生成的是一组带时间的关节路径点。控制周期内的插值仍由 JTC 执行。Humble 官方说明：仅提供位置时使用线性插值，提供位置和速度时使用三次插值，位置、速度和加速度都提供时使用五次插值。[官方插值说明](https://control.ros.org/humble/doc/ros2_controllers/joint_trajectory_controller/doc/trajectory.html)

当前关节 demo 提供起终点位置与零端点速度，因此采用三次插值；笛卡尔采样后的关节轨迹只提供位置，因此点间采用线性插值。当前实现没有宣称笛卡尔轨迹的速度或加速度连续。

| 当前默认设置 | 数值 |
|---|---|
| 控制器更新 | 100 Hz |
| 默认 PyBullet 物理步进 | 240 Hz |
| 控制器状态发布 | 50 Hz |
| 关节目标最短时长 | 4 s，按关节速度要求延长 |
| 关节轨迹速度基准 | 0.35 rad/s |
| 笛卡尔采样间隔上限 | 平移 0.004 m，旋转 0.03 rad |
| 笛卡尔速度基准 | 平移 0.04 m/s，旋转 0.20 rad/s |
| 相邻 IK 解最大关节变化 | 0.25 rad |
| IK 独立 FK 校验 | 位置 0.001 m，姿态 0.005 rad |
| 路径输入限制 | 1–100 个位姿，最多 2000 个采样点，总时长不超过 120 s |
| 笛卡尔规划计算预算 | 5 s，采样循环中检查，超时整段拒绝 |

目标位置任一轴绝对值超过 2 m 时先做输入拒绝；该范围只是防止异常输入的外包络，范围内仍需通过 IK，并不代表机械臂真实可达区域。

速度基准用于时间分配，不是独立认证的完整连续轨迹速度/加速度约束。实时频率取决于运行机器负载，240 Hz 是默认定时器设置；本实现未发布 `/clock`，时间使用系统 ROS 时钟。

因此“笛卡尔轨迹跟踪”包含两层误差：采样和 IK 导致的几何近似误差，以及物理执行导致的关节跟踪误差。增密采样可以减少部分几何误差，但不能代替速度规划、奇异点处理或碰撞规划；采样间的完整连续路径也需要单独验证。

本教程不提供 MoveIt 规划场景、障碍物避让或夹爪接触策略。关节限位与 IK 可达性是运控输入校验的一部分，不能解释为完整抓取可行性检查。

## 关节名称映射

| 控制接口 | PyBullet 转动关节顺序 |
|---|---|
| `lbr_A1` | 1 |
| `lbr_A2` | 2 |
| `lbr_A3` | 3 |
| `lbr_A4` | 4 |
| `lbr_A5` | 5 |
| `lbr_A6` | 6 |
| `lbr_A7` | 7 |

映射使用模型中七个可动关节，物理反馈必须对应同一顺序。若更换机器人 URDF、设置非零基座变换或增加工具，需同时更新仿真模型、运动学模型和坐标文档，再重新运行两个 demo。

硬件激活时等待真实反馈，并把初始命令设置为收到的关节位置。反馈超过 0.5 s 未更新时，硬件接口报错并停止发布位置命令。默认仿真端另有 0.5 s 命令超时，触发时保持当前测量关节位置。规划用 FK/IK 位于独立的 PyBullet DIRECT client，重置该运动学模型的关节不会瞬移物理仿真机械臂。

## 与 LBR Stack 的关系

LBR Stack 的上游 Python 范例通过 `FollowJointTrajectory` 发送 `lbr_A1`–`lbr_A7` 的目标，本教程沿用这一标准 action 入口，并保留上游来源记录。[上游客户端](https://github.com/lbr-stack/lbr_fri_ros2_stack/blob/humble/lbr_demos/lbr_demos_py/lbr_demos_py/joint_trajectory_client.py)

LBR Stack 提供真机 FRI 接口和仿真集成；本项目的工作是把其采用的 ROS 轨迹控制方式接到当前 PyBullet。后续迁移到真实 LBR 时，应替换硬件层、使用与真机一致的 LBR 描述和控制配置，并重新验证坐标、TCP 与运动限制。当前 PyBullet 工程的通过结果不能用作真机运行证据。
