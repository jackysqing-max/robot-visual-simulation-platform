# 验证与证据

本文件记录 2026-10-19 独立仓库的复测，并保留拆分前教程的验证快照。独立仓库已验证真实 OpenGL GUI 中的关节与笛卡尔 demo，以及仿真/控制分离运行的 `external` 模式。以下数值来自 PyBullet 仿真；抓取接触与真机不在本次验证范围内。

## 独立仓库复测

从 `/home/siqin/ros2_workspaces/lbr_pybullet_ros` 构建运行。隔离的启动环境仅加载 `/opt/ros/humble` 和本仓库 `install/local_setup.bash`，未加载原 `humble` 工作区 overlay。默认后端所需源码和配置均来自当前仓库。

| 项目 | 状态 | 当前独立仓库证据 |
|---|---|---|
| 独立构建 | 两包通过 | [构建日志](evidence/standalone_build.log)、[环境](evidence/standalone_environment.json) |
| 单元/接口检查 | 54 项通过 | [独立单元输出](evidence/standalone_unit_tests.txt) |
| 默认无窗口端到端 | 8 项通过 | [当前摘要](evidence/standalone_headless/summary.json) |
| 源码 ZIP 迁移构建与运行 | 通过 | [独立目录复测](evidence/portability/summary.json) |
| GUI 关节与笛卡尔 demo | 两项通过 | [GUI 摘要](evidence/standalone_gui/summary.json)、[关节报告](evidence/standalone_gui/joint.json)、[笛卡尔报告](evidence/standalone_gui/cartesian.json) |
| GUI 真窗口截图 | 已保存 | [关节窗口](evidence/standalone_gui/joint.png)、[笛卡尔窗口](evidence/standalone_gui/cartesian.png) |
| `external` 分离仿真/控制节点 | 8 项通过 | [external 摘要](evidence/standalone_external/summary.json) |

GUI 测试实际创建 Bullet 的 OpenGL 窗口并捕获窗口图像，同时执行完整 demo 的反馈与误差验收；截图和 JSON 分别证明可视化窗口与运动执行。GUI 精确样本数、终点误差、最大跟踪误差和路径几何偏差以本次保存的 JSON 为准。

`external` 测试先单独运行 `sim.launch.py`，再启动 `tutorial.launch.py sim_backend:=external`。后者只启动控制器、硬件桥、状态发布与运动接收节点。该模式下，关节 demo、笛卡尔 demo、单个位姿、重复关节名拒绝、不可达位姿拒绝、错误坐标系拒绝、busy 拒绝与取消均通过。Cartesian 终点位置误差为 0.0413 mm，实测采样路径最大几何偏差为 0.0741 mm；这是本次仿真观测值。

源码包还被解压到一个全新的临时目录，在该目录重新构建两个包并运行完整无窗口检查，全部通过；该检查没有使用原目录的 build/install。

可按 [快速开始](quickstart.md) 复现 GUI 与分离运行方式。运行时报告位于 `artifacts/gui/` 和 `artifacts/external/`，本次归档副本位于上表的 `docs/evidence/standalone_*` 路径。

## 自动检查入口

```bash
cd /home/siqin/ros2_workspaces/lbr_pybullet_ros
./scripts/test.sh
```

检查范围以脚本当前输出为准。测试应覆盖运动学和接口校验，并分别验证关节控制与笛卡尔控制的完整运行路径；构建成功不能替代端到端测试。

默认运行单元检查和默认后端的端到端检查。可选运行已有后端与约 30 s 的上游原始客户端：

```bash
./scripts/test.sh --existing --upstream
```

`--upstream` 为默认后端追加原样 LBR Stack 客户端检查；`--existing` 再启动已有 PyBullet 后端检查。脚本使用独立测试 domain，结果位于 `artifacts/smoke_tutorial/` 与 `artifacts/smoke_existing/`。

独立反馈中断测试使用自己的 domain，并只暂停它自己启动的仿真进程：

```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash
/usr/bin/python3 scripts/feedback_fault_test.py --domain 92 --output artifacts/feedback_fault
```

若手动复现，按 [快速开始](quickstart.md) 启动系统，在另一终端分别执行：

```bash
ros2 run lbr_pybullet_tutorial demo --mode joint --timeout 60 --report /tmp/lbr_joint_report.json
ros2 run lbr_pybullet_tutorial demo --mode cartesian --timeout 60 --report /tmp/lbr_cartesian_report.json
```

保留命令、退出码、控制器日志和 JSON 报告；报告保存目录可以改成自己的实验记录目录。

## 必须证明的行为

| 检查 | 需要观察的证据 |
|---|---|
| 物理状态链路 | `/iiwa7/joint_states` 持续变化，JTC actual 与其对应 |
| 控制器执行 | 控制器 active，action 接收和结果可关联，desired/actual/error 可观察 |
| 关节 demo | 完整七轴目标执行后与实测位置对比，保存最大关节误差 |
| 笛卡尔 demo | 目标采样、IK 轨迹、由实测关节求得的末端位置/姿态对比 |
| 有效输入校验 | 拒绝非法关节名、越界或非有限值、错误 frame、无效四元数 |
| 不可执行路径 | IK 失败时整段拒绝，不能只执行前半段 |
| 并发与取消 | busy 拒绝不影响原目标，取消后状态可解释 |
| 后端可替换 | 默认后端与 `existing` 模式各自留证 |

这些检查证明本模型上的输入、运动学、控制器与物理反馈链路。它们不证明夹持稳定性、器械分拣成功率、碰撞规避或真实机械臂精度。

## 误差如何解释

报告的关节终点误差为各轴 `abs(q_actual - q_target)` 的最大值，单位 rad。笛卡尔位置终点误差使用目标与实测 FK 位置的欧氏距离；姿态终点误差使用四元数对应的最小旋转角，单位 rad。

过程跟踪误差使用同一条 JTC 状态消息里的 actual 与 desired：分别求 FK，再计算位置和姿态误差。状态配置为 50 Hz，报告记录实际收到的样本数量与单调时钟时间。

笛卡尔报告另含 `max_actual_path_deviation_m` 与 `max_desired_path_deviation_m`：把每个 actual/desired FK 位置投影到输入方形折线的所有线段，取最近距离，再报告所有采样的最大值。它检验路径几何，不是按时间对齐的 Cartesian 参考误差，也不能说明采样间的最大偏差。

当前 demo 的 `passed` 要求：action 成功、至少 5 个过程样本、终点位置误差 <3 mm、终点关节误差 <0.02 rad、终点姿态误差 <0.01 rad、最大位置跟踪误差 <15 mm。Cartesian demo 额外要求实测采样路径最大几何偏差 <3 mm。最大过程关节/姿态误差也记录在报告中，未设单独的 demo 阈值。

action 的 `SUCCESSFUL` 是控制器执行结果，物理状态对比是另一条证据；验收时同时查看二者。IK 预测路径和物理执行路径应分别记录，以便区分几何近似与伺服误差。

## 拆分前教程的验证快照

本节及其 `evidence/tutorial`、`evidence/existing`、`evidence/upstream`、`evidence/fault` 记录来自拆分前教程，随独立仓库归档以说明原链路的验证历史。它们与上面的独立仓库复测分开列示；不能据此声称当前仓库已重新运行旧后端、上游客户端或反馈中断测试。

| 项目 | 状态 | 证据 |
|---|---|---|
| 依赖与构建 | 通过 | [构建日志](evidence/build.log)、[环境版本](evidence/environment.json) |
| 单元/接口检查 | 54 项通过 | [pytest 输出](evidence/unit_tests.txt)、[测试源码](../tests/test_planning.py) |
| 默认后端 joint / cartesian 与异常输入 | 通过 | [基础检查摘要](evidence/tutorial/summary.json) |
| 上游原样客户端与独立运控复核 | 通过 | [上游客户端检查摘要](evidence/upstream/summary.json) |
| existing 后端 joint / cartesian 与异常输入 | 通过 | [现有后端摘要](evidence/existing/summary.json) |
| 反馈中断 | 通过 | [反馈中断摘要](evidence/fault/summary.json) |
| GUI | 原快照未覆盖 | 当前独立仓库 GUI 结果见上节 |

默认和 existing 两种后端均完成：关节目标、3 cm 方形笛卡尔路径、单个 PoseStamped 小位移、重复关节名称拒绝、不可达位姿拒绝、错误 frame 拒绝、busy 拒绝与取消。取消检查期望最终 `action_status=5`（CANCELED），教学状态映射为 `failed`；这项结果表示取消行为符合预期。

最终默认后端测试使用 domain 89，existing 使用 domain 90；上游原样客户端单独使用 domain 91。下面的长度统一换算为 mm，原始报告使用 m：

| 后端与模式 | 样本数 | 终点位置误差 mm | 最大过程跟踪误差 mm | 实测路径最大几何偏差 mm | 终点最大关节误差 rad |
|---|---:|---:|---:|---:|---:|
| 默认 joint | 157 | 0.002097 | 0.979217 | 不适用 | 0.000004272 |
| 默认 cartesian | 119 | 0.040695 | 0.777888 | 0.089640 | 0.000017190 |
| existing joint | 155 | 0.003547 | 1.075465 | 不适用 | 0.000006516 |
| existing cartesian | 117 | 0.083585 | 0.989691 | 0.183928 | 0.000188344 |

默认与 existing 的 Cartesian desired 路径几何偏差分别为 0.030547 mm、0.028045 mm。表格是拆分前该次运行的观测值，不能解释为设备精度保证。

上游原始客户端两段轨迹均被控制器接受，日志未出现执行失败，最终七轴相对零位的最大绝对位置为 `2.072043e-6` rad；记录在 [上游客户端检查摘要](evidence/upstream/summary.json) 中的 `unmodified_lbr_stack_client` 项。

反馈中断测试先确认 JTC 正在执行实际轨迹，再暂停该测试的物理进程。本次应用在中断后 0.9405 s 报告超时失败；最后一条位置命令出现在中断后 0.4990 s，在中断后 1.0–1.8 s 的观察窗内收到 0 条新命令。硬件日志同时记录原始反馈过期。测试结束会恢复其暂停的进程并关闭 launch。

表格与图片采用最终默认后端和现有后端摘要中的测量值。重新运行测试会更新 `artifacts/`，本次交付证据单独保存在 `docs/evidence/`。

## 查看与导出轨迹图

下图显示一次默认后端 Cartesian 运行的七轴 desired/actual、位置跟踪误差和输入方形路径对比：

![笛卡尔轨迹与实测反馈](evidence/cartesian_tracking.png)

[关节 demo 轨迹图](evidence/joint_tracking.png) 另行保存。要从自己的报告生成图片，额外安装绘图依赖 `matplotlib`；它不是运控链路的运行依赖：

```bash
/usr/bin/python3 -m pip install --user matplotlib
/usr/bin/python3 scripts/plot_tracking.py artifacts/cartesian_report.json /tmp/cartesian_tracking.png
```

## 后续应用验收

M1–M5 的语言理解、分割、标定、抓取与最终任务评估在 [学生计划](student_plan.md) 中列出。当前工程的关节或笛卡尔 demo 通过后，只能把运控预备环节标为完成；应用成果需要自己的逐场景证据。

图形模式复测中，首次启动曾出现一次 19.74 mm 的采样跟踪误差，超过 15 mm 门限；该次动作成功但整体验收失败。同一代码与门限的后续运行通过，原始失败报告保存在 `evidence/standalone_gui/attempt1/`。GUI 渲染与系统负载会影响定时器调度，验收应保留全部运行记录。窗口可用鼠标调整视角，绿色实测轨迹可能被机械臂局部遮挡。
