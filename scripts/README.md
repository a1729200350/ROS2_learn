# 辅助脚本

这些脚本位于工作空间根目录，未注册为 `ros2 run` 入口。完整运行环境见[首页](../README.md)，实验文件见[OSC 记录](../osc_records/README.md)。

## 控制器与 OSC 输出监视

[`osc_controller_state_watch.py`](osc_controller_state_watch.py) 由 Gazebo launch 自动启动，不必另开一份。它不是纯日志查看器：触发故障后会主动请求暂停仿真。

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
