# MoveIt → OSC 轨迹工具

这些工具位于源码目录，未注册为 `ros2 run` 命令。返回[MoveIt 配置说明](../README.md)；已有样例见[轨迹数据](../../../moveit_trajectory_records/README.md)，Gazebo 数据见[OSC 录包](../../../osc_records/README.md#moveit-文件轨迹录包)。

## 文件流程

| 工具 | 输入 | 输出与行为 |
| --- | --- | --- |
| [`capture_moveit_trajectory.py`](capture_moveit_trajectory.py) | `/display_planned_path` 的 `moveit_msgs/msg/DisplayTrajectory` | 保存首条合格消息中第一段关节轨迹；固定写到 `~/ros2_ws/moveit_trajectory_records/` 的时间戳 NPZ |
| [`convert_moveit_to_osc.py`](convert_moveit_to_osc.py) | 原始轨迹 NPZ | 保留关节数据，计算 `x/dx/ddx`、`J_dot dq`、最小奇异值，写出 `<输入名>_osc.npz` |
| [`interpolate_moveit_trajectory.py`](interpolate_moveit_trajectory.py) | 转换后的 NPZ | 用端点位置/速度/加速度作五次 Hermite 插值，按 `0.01 s` 采样并追加终点，写出 `<输入名>_interp_100hz.npz` |
| [`analyze_moveit_osc_bag.py`](analyze_moveit_osc_bag.py) | MCAP 录包目录及参考 NPZ | 读取 `/osc_tracking`，写出 `<录包名>_analysis/` 内的 NPZ 和三张图 |

转换和插值不会改写原始输入，但会覆盖同名派生输出；录包分析也会覆盖同名结果。仓库已经保留样例，浏览数据不必重跑这些命令。

精选轨迹 NPZ、分析 NPZ 和图片继续跟踪；实验目录中的新增数据默认仅本地保存。原始 MCAP 不再纳入 Git，新克隆无法直接运行依赖历史录包的分析命令，需要先取得完整录包。数据保留范围见[首页规则](../../../README.md#实验数据管理)，本次未修改任何工具的读写路径。

## 采集新规划

在 ROS 2 环境中具备 `rclpy`、`moveit_msgs` 和 NumPy 后，从工作空间根目录启动：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 src/three_link_arm_moveit_config/tools/capture_moveit_trajectory.py
```

脚本使用可靠、VOLATILE QoS，最多等待 `120 s`；应在订阅已启动后让 MoveIt 产生新规划。此步骤在 RViz 点击 **Plan**，不需要点击 Execute，也不会自动执行轨迹。消息必须包含位置、速度、加速度且数值有限、时间严格递增；只保存第一条合格消息的 `trajectory[0]`，不是全部候选或多段轨迹。

## 转换与插值

下列命令以仓库中的现有样例为输入，运行会覆盖其已有转换/插值结果；若只查看样例，请直接浏览数据目录。

```bash
python3 src/three_link_arm_moveit_config/tools/convert_moveit_to_osc.py \
  moveit_trajectory_records/moveit_trajectory_20261013_085205.npz
python3 src/three_link_arm_moveit_config/tools/interpolate_moveit_trajectory.py \
  moveit_trajectory_records/moveit_trajectory_20261013_085205_osc.npz
```

两者使用 NumPy，要求三关节数组及有效、递增时间。转换工具要求名称顺序为 `joint1, joint2, joint3`，输出任务坐标系为 `arm_base`，不是世界坐标系；按 `x=f(q)`、`dx=J dq`、`ddx=J ddq + J_dot dq` 计算，并打印 `J_dot dq` 的有限差分核对结果。

插值工具在原节点核对位置/速度/加速度，最大重建误差超过 `1e-7` 时拒绝继续；终点显式追加，因此最后一个采样间隔可能不足 `0.01 s`。这个 `100 Hz` 是参考采样间隔，不是 Gazebo 实测控制频率。在线节点对密集 `q/dq/ddq` 再分别线性采样，见[节点说明](../../three_link_arm_kinematics/README.md#moveit-文件轨迹-osc)。

## 手动离线实验

以下脚本位于 [`manual_tests/`](../manual_tests/)，不是由默认包测试自动证明通过的完整验收：

| 脚本 | 输入与验证范围 | 写文件 |
| --- | --- | --- |
| [`test_moveit_osc_reference.py`](../manual_tests/test_moveit_osc_reference.py) | 不接收文件参数，使用源码内单个固定参考点核对任务参考与 `J_dot dq` | 否，打印结果 |
| [`test_moveit_osc_torque_consistency.py`](../manual_tests/test_moveit_osc_torque_consistency.py) | 接收转换或插值 NPZ；在理想跟踪状态比较 OSC 加零空间前馈与完整逆动力学，并检查参考一致性 | 否，打印结果与力矩范围 |
| [`test_moveit_osc_closed_loop.py`](../manual_tests/test_moveit_osc_closed_loop.py) | 密集 NPZ；A/B/C 控制方案、可选命令延迟；`10 ms` 控制、`1 ms` 积分、约 `1 s` 终点保持、逐关节 `±10 N·m` 限幅 | 新目录内每方案一个 NPZ、汇总 CSV 和六张对比图 |
| [`test_moveit_osc_singularity_profile.py`](../manual_tests/test_moveit_osc_singularity_profile.py) | 参考 NPZ 与 Gazebo 分析 NPZ；比较 Jacobian 奇异值、条件数与操作空间惯性最大特征值 | 否，打印统计 |
| [`test_osc_regularization.py`](../manual_tests/test_osc_regularization.py) | 同上；`--case all/force/projector/split` 比较任务正则化、零空间泄漏和分离式投影 | 否，打印结果 |
| [`test_osc_singular_direction.py`](../manual_tests/test_osc_singular_direction.py) | 同上；默认 `--scale 1 3`，比较最敏感姿态和完全伸直姿态下的方向降级 | 否，打印结果 |
| [`test_osc_qp_task.py`](../manual_tests/test_osc_qp_task.py) | 固定姿态、偏置已补偿的增量模型；比较截断、软/硬任务和不可行目标 | 否，检查求解状态、力矩与任务等式 |
| [`test_osc_qp_dynamics.py`](../manual_tests/test_osc_qp_dynamics.py) | 固定姿态的静止/非零速度状态，包含重力、速度偏置及 `Jdot dq`；运动状态另加入零空间姿态力矩 | 否，检查完整动力学任务残差、力矩余量和姿态项泄漏 |

这些实验需要可导入工作空间的 `three_link_arm_kinematics`，QP 实验还需要 SciPy 和 OSQP。构建并加载 `install/setup.bash` 后，从根目录运行，例如：

```bash
python3 src/three_link_arm_moveit_config/manual_tests/test_moveit_osc_torque_consistency.py \
  moveit_trajectory_records/moveit_trajectory_20261013_085205_osc_interp_100hz.npz
```

实验复用四个同目录模块：[`osc_analysis_common.py`](../manual_tests/osc_analysis_common.py) 负责轨迹读取及矩阵分析；[`osc_direction_common.py`](../manual_tests/osc_direction_common.py) 负责方向权重实验；[`osc_qp_common.py`](../manual_tests/osc_qp_common.py) 负责可行性和 QP 求解；[`osc_simulation_common.py`](../manual_tests/osc_simulation_common.py) 负责闭环动力学、延迟队列、结果汇总及 `calculate_posture_torque` 姿态力矩辅助函数。总力矩 QP 入口从该仿真模块导入姿态辅助函数，因此还需要可导入 Matplotlib。运行入口脚本时会从同目录加载这些模块。

### A/B/C 闭环与命令延迟

```bash
python3 -B src/three_link_arm_moveit_config/manual_tests/test_moveit_osc_closed_loop.py \
  moveit_trajectory_records/moveit_trajectory_20261013_085205_osc_interp_100hz.npz \
  --modes A_strict B_all_damped C_split --delay-ms 20
```

- `A_strict` 沿用严格操作空间动力学和动态一致零空间；`B_all_damped` 同时正则化任务逆惯性和零空间矩阵；`C_split` 只正则化主任务，零空间用 SVD 基构造精确投影。B/C 保留完整关节动力学偏置补偿。
- 默认运行三种模式且 `--delay-ms 0`；延迟必须为非负 `10 ms` 整数倍。延迟实验将首条力矩预置为初始保持值，后续命令通过队列延迟生效，不模拟启动阶段的自由下落。
- 默认输出目录为 `<输入名>_closed_loop_delay_<三位毫秒数>ms_<时间戳>/`，也可通过 `--output` 指定一个尚不存在的目录；已存在的目录会被拒绝，不会覆盖旧结果。
- 每个方案输出同名 NPZ，如 `A_strict.npz`；另有 `summary.csv`、末端 x/y 误差、正则化参数、最小奇异值、姿态加速度泄漏及 joint1 计算/施加力矩共六张对比图。
- NPZ 中 `tau_computed` 是本轮计算并限幅的指令，`tau` 是实际施加的力矩；`command_age_ms` 在控制采样时刻记录所用指令年龄。`--baseline` 可读取另一份该格式汇总 CSV，按方案打印 RMS 差值，但不会自动判断两份实验的参数是否一致。
- 某方案触发安全退出时，保存已有记录并在汇总中标记 `completed=False`；判断是否完成应查看汇总，不能只看进程退出码。只有完整完成的方案才报告全程运动 RMS。

本次使用仓库现有密集轨迹，在内存中检查三种方案各自的 `0/20 ms` 延迟，共六组均完成 1,500 个控制采样，输出力矩均在 `±10 N·m` 内；队列力矩与指定延迟的计算指令对应。另已运行上表新增的三个分析入口和两个 QP 入口，总力矩实验新增的姿态项检查也已通过。在该固定工况下，加入姿态目标前后的 QP 力矩基本相同；这只验证主任务和力矩约束未被破坏，不证明姿态已收敛。没有覆盖既有 NPZ 或图片。这些是同一标称模型上的离线检查，不替代 Gazebo、真实反馈或硬件验收，也不能证明三种方案对所有轨迹均适用。

## Gazebo 录包分析

先取得完整 MCAP 录包并放回下面指定目录，再在 ROS 环境具备 `rosbag2_py`、MCAP 插件、`std_msgs`、NumPy 和 Matplotlib 后运行，例如：

```bash
python3 src/three_link_arm_moveit_config/tools/analyze_moveit_osc_bag.py \
  osc_records/moveit_osc_effort_20261013_103450 \
  moveit_trajectory_records/moveit_trajectory_20261013_085205_osc_interp_100hz.npz
```

脚本只读消息，不回放控制命令；但会写入分析结果。它筛除不是 16 字段或含非有限数的消息，要求有启动后数据，且最大轨迹时间至少达到参考终点前 `0.1 s`。运动指标只使用 `0 < elapsed <= T` 的样本，并要求至少十条；终点后的保持数据另行报告。这是时间范围检查，不是逐时刻无缺口证明。

工具从参考 NPZ 线性还原 `q_d`，计算关节误差、任务误差、最大输出力矩和实际姿态最小奇异值；RMS 按样本平均，不按时间加权。饱和附近计数的阈值固定为 `9.999 N·m`，不是从录包中读取的动态参数。

输出为 `gazebo_tracking.npz`、`gazebo_task_error.png`、`gazebo_joint_error.png`、`gazebo_torque.png`，使用无窗口的 `Agg` 后端。同名文件会覆盖；本次文档维护未运行上述生成命令。
