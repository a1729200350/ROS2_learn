# 辅助脚本

这些脚本位于工作空间根目录，未注册为 `ros2 run` 入口。完整运行环境见[首页](../README.md)，实验文件见[OSC 记录](../osc_records/README.md)。

MoveIt 轨迹采集、转换、插值及文件轨迹 OSC 的录包分析位于另一个目录，见[MoveIt 工具说明](../src/three_link_arm_moveit_config/tools/README.md)，不要与本目录的 JTC 状态分析或基线绘图混淆。

原始 MCAP 已停止 Git 跟踪，新克隆不含完整录包。下列绘图和 JTC 分析命令都需要先将完整录包放回指定路径；已有曲线和摘要可直接查看。脚本及其读写路径未因这次数据管理调整而修改，详见[数据管理规则](../README.md#实验数据管理)。

## 控制器与 OSC 输出监视

[`osc_controller_state_watch.py`](osc_controller_state_watch.py) 由原 OSC 的 `three_link_arm_gazebo.launch.py` 或文件驱动的 `three_link_arm_moveit_osc.launch.py` 自动启动，不必另开一份；独立 JTC launch 不启动它。它不是纯日志查看器：触发故障后会主动请求暂停仿真。

| 监视条件 | 当前行为 |
| --- | --- |
| 尚未观察到 `effort_controller` 为 `active` | 等待，不按未激活判故障 |
| 曾经激活，后来状态变为非 `active` 或控制器缺失 | 锁定故障 |
| 已激活，但从未收到 `/osc_tracking` | 激活超过 `2 s` 后锁定故障 |
| 已激活且收到过 `/osc_tracking` | 最近消息距今超过 `0.5 s` 时锁定故障 |

控制器状态来自 `/controller_manager/activity`（`ControllerManagerActivity`），采用可靠、瞬态本地、深度为 1 的 QoS；跟踪消息使用 `Float64MultiArray`。每 `0.05 s` 检查一次，消息年龄由单调墙钟计算。脚本只记录跟踪消息的到达时间，不检查其数值是否正确或是否饱和。

故障只触发一次，后台线程先请求 `/world/empty/control` 的 `pause: true`，成功后读取 `/world/empty/stats` 确认 `paused: true`。“接受暂停请求”与“确认已暂停”是不同日志；失败或超时会报告错误。脚本不发布零力矩、不自动重启控制器，也不自动恢复世界。

路径与环境限制：

- launch 固定用 `/usr/bin/python3` 执行 `~/ros2_ws/scripts/osc_controller_state_watch.py`，不会改用虚拟环境的 Python。
- 需要已加载的 ROS 2 环境、`rclpy`、`std_msgs`、`controller_manager_msgs` 和可用的 `gz` 命令。
- 服务与状态话题固定针对 `empty` 世界；更换世界名后不能直接假定监视器仍有效。

## 基线录包绘图

[`plot_osc_error.py`](plot_osc_error.py) 使用 `rosbag2_py` 读取 MCAP，不回放消息。它固定读取 `~/ros2_ws/osc_records/baseline_01`，目前不支持命令行选择其他录包。

在具备 NumPy、Matplotlib 和 MCAP 存储插件的 ROS 2 环境中，从工作空间根目录运行：

```bash
source /opt/ros/jazzy/setup.bash
python3 scripts/plot_osc_error.py
```

无图形环境时可用 `MPLBACKEND=Agg python3 scripts/plot_osc_error.py`。脚本读取 `/osc_tracking`，跳过长度少于 16 的消息；找不到有效消息或轨迹时间大于零的记录时会报错。

横轴以第一条 `t > 0` 的跟踪消息对应的录包时间为零，绘制前 `0.5 s` 至后 `5 s` 的窗口；这是根据采样消息定位的起点，不是精确的服务调用时间。保存结果如下，已有同名图片会被覆盖：

| 输出文件（位于 `osc_records/`） | 内容 |
| --- | --- |
| `baseline_01_error.png` | x/y 末端位置误差，原始 m 转换为 mm |
| `baseline_01_torque.png` | 三关节发布的力矩指令，N·m |
| `baseline_01_velocity.png` | 三关节速度，rad/s |

当前脚本在顶层直接执行读取和绘图，导入它也会产生上述文件写入；它不是无副作用的库模块。仓库已附带三张图片，浏览结果不需要重新运行脚本。

## 周期往返录包绘图

[`plot_osc_periodic.py`](plot_osc_periodic.py) 固定读取 `~/ros2_ws/osc_records/periodic_01`，依赖与基线脚本相同。它不回放消息，也没有命令行录包选择参数；不会自动切换到 `periodic_60s_01`。

```bash
source /opt/ros/jazzy/setup.bash
python3 scripts/plot_osc_periodic.py
```

无图形环境时可在命令前加 `MPLBACKEND=Agg`。脚本跳过不足 16 字段或轨迹时间 `t <= 0` 的消息，以控制器发布的累计轨迹时间为横轴，绘制全部有效记录，不采用基线脚本的前后截取窗口。若找不到启动后的数据则报错。

终端输出有效样本数、轨迹时间范围、x/y 全程 RMS 和最大绝对误差（mm），以及逐关节最大绝对力矩（N·m）。RMS 按消息样本计算，未按相邻时间间隔加权。输出位置为 `osc_records/`：

| 文件 | 内容 |
| --- | --- |
| `periodic_01_tracking.png` | x/y 期望与实际末端位置，m |
| `periodic_01_error.png` | x/y 位置误差，mm |
| `periodic_01_torque.png` | 三关节限幅后的力矩指令，N·m |
| `periodic_01_velocity.png` | 三关节速度，rad/s |

运行或导入此脚本都会执行顶层绘图并覆盖同名图片。仓库已经保存了四张图，可直接在[实验记录页](../osc_records/README.md#已保存的周期曲线)查看；本目录说明不将文件名或当前参数当成采集时的参数证明。

## MoveIt/JTC 录包分析

[`analyze_moveit_jtc_bag.py`](analyze_moveit_jtc_bag.py) 从命令行接收一个录包目录，读取 `/joint_trajectory_controller/controller_state`（`control_msgs/msg/JointTrajectoryControllerState`）。它使用 JTC 同一条消息中的参考与反馈，不依赖分别对齐 `/joint_states` 和外部轨迹，也不回放录包。

加载 ROS 2 环境并具备 `control_msgs`、`rosbag2_py`、MCAP 插件、NumPy 和 Matplotlib 后，从工作空间根目录运行，例如：

```bash
source /opt/ros/jazzy/setup.bash
python3 scripts/analyze_moveit_jtc_bag.py \
  jtc_records/moveit_jtc_gain020_speed020_20261012_155117
```

脚本会创建或使用录包旁的 `<录包名>_analysis/` 目录，并覆盖其中的 `joint_positions.png`、`joint_errors.png`；已在源码中选择 `Agg` 后端，不需要图形窗口。数值结果打印到终端，当前不会自动保存分析文本。

分析规则：

- 用控制器状态的 `header.stamp` 建立时间轴，要求时间严格递增、关节顺序不变，且至少有两条有效状态；参考/反馈位置长度不匹配的记录跳过。
- 误差为 `reference.positions - feedback.positions`。期望速度数组缺失或长度不匹配时按零补齐，仍可通过参考位置变化识别运动。
- 任一关节的期望速度绝对值超过 `1e-4`，或相邻参考位置变化超过 `1e-7` 时视为运动；选取第一至最后运动样本，并按中位采样间隔在前后各保留约 `0.2 s`。多个运动段之间的停留也会包含在这个窗口内。
- 输出该窗口内的逐关节 RMS、最大绝对误差（rad）及 RMS（deg），并绘制参考/反馈位置和误差；RMS 按样本计算，不是时间加权积分。
- 在速度足够大的样本上拟合 `误差 ≈ 等效滞后 × 参考速度`，报告拟合滞后与残差 RMS；不足十个有效样本时不拟合。该值是等效跟踪滞后，不是独立测量的通信或调度延迟。

若录包没有有效控制器状态、时间不递增或未检测到期望运动，脚本会报错。已有录包的元数据、图片和已保存文本见[JTC 实验记录](../jtc_records/README.md)，查看这些文件不需要重新运行分析脚本；原始 MCAP 需另外取得。
