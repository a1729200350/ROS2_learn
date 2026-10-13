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
| [`test_moveit_osc_closed_loop.py`](../manual_tests/test_moveit_osc_closed_loop.py) | 接收密集 NPZ；控制周期 `10 ms`、半隐式欧拉积分 `1 ms`、结束后继续保持约 `1 s`、逐关节 `±10 N·m` 限幅 | 写 `<输入名>_closed_loop/` 内 NPZ 和三张图 |

后两项需要可导入工作空间的 `three_link_arm_kinematics`；构建并加载 `install/setup.bash` 后，从根目录运行，例如：

```bash
python3 src/three_link_arm_moveit_config/manual_tests/test_moveit_osc_torque_consistency.py \
  moveit_trajectory_records/moveit_trajectory_20261013_085205_osc_interp_100hz.npz
```

离线闭环使用同一标称动力学模型及移动姿态零空间目标，不运行 Gazebo；重复运行会覆盖 `tracking.npz`、`task_error.png`、`joint_error.png`、`torque.png`。这些结果不能替代真实反馈闭环验证。

## Gazebo 录包分析

在 ROS 环境具备 `rosbag2_py`、MCAP 插件、`std_msgs`、NumPy 和 Matplotlib 后，例如：

```bash
python3 src/three_link_arm_moveit_config/tools/analyze_moveit_osc_bag.py \
  osc_records/moveit_osc_effort_20261013_103450 \
  moveit_trajectory_records/moveit_trajectory_20261013_085205_osc_interp_100hz.npz
```

脚本只读消息，不回放控制命令；但会写入分析结果。它筛除不是 16 字段或含非有限数的消息，要求有启动后数据，且最大轨迹时间至少达到参考终点前 `0.1 s`。运动指标只使用 `0 < elapsed <= T` 的样本，并要求至少十条；终点后的保持数据另行报告。这是时间范围检查，不是逐时刻无缺口证明。

工具从参考 NPZ 线性还原 `q_d`，计算关节误差、任务误差、最大输出力矩和实际姿态最小奇异值；RMS 按样本平均，不按时间加权。饱和附近计数的阈值固定为 `9.999 N·m`，不是从录包中读取的动态参数。

输出为 `gazebo_tracking.npz`、`gazebo_task_error.png`、`gazebo_joint_error.png`、`gazebo_torque.png`，使用无窗口的 `Agg` 后端。同名文件会覆盖；本次文档维护未运行上述生成命令。
