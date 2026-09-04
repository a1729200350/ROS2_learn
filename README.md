# ROS2_learn

一个面向 ROS 2 入门与机械臂运动学实验的学习工作空间，包含双 `turtlesim`、三连杆机械臂 URDF/RViz、约束逆运动学，以及基于 ros2_control 模拟硬件的轨迹生成、执行与跟踪误差监控。

## 功能包

| 功能包 | 类型 | 内容 |
| --- | --- | --- |
| `my_turtle_launch` | `ament_python` | 同时启动两个带命名空间的 `turtlesim` 节点，分别使用红色和蓝色背景 |
| `two_link_arm_description` | `ament_cmake` | 三连杆 URDF、RViz 启动文件及 ros2_control 模拟硬件、关节状态广播器和轨迹控制器配置 |
| `two_link_arm_kinematics` | `ament_python` | 提供 `kinematics_monitor`、`trajectory_generator`、`trajectory_monitor` 三个入口，分别用于约束逆运动学分析、轨迹生成与发布、轨迹跟踪误差监控 |

三连杆模型的连杆长度为 `L1 = 0.5 m`、`L2 = 0.4 m`、`L3 = 0.4 m`，与 URDF 和运动学节点中的参数一致。

## 环境

- Ubuntu 24.04 或 WSL2 Ubuntu
- ROS 2 Jazzy
- Python 3、NumPy、SciPy、OSQP
- RViz2、`joint_state_publisher_gui`、`robot_state_publisher` 和 `turtlesim`
- ros2_control、ros2_controllers、`controller_manager`、`trajectory_msgs`

## 获取代码与安装依赖

```bash
git clone git@github.com:a1729200350/ROS2_learn.git
cd ROS2_learn
```

首次使用 `rosdep` 时，需要先完成系统级初始化：

```bash
sudo rosdep init
rosdep update
```

随后在工作空间根目录安装清单中声明的依赖：

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

当前运动学节点还会直接导入 `scipy` 和 `osqp`。可使用保留 ROS 2 系统包访问能力的本地虚拟环境安装 Python 依赖：

```bash
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install numpy scipy osqp
```

`.venv/` 仅用于本机运行环境，不需要提交到仓库。

新增节点使用 `trajectory_msgs`；运动学包的依赖清单目前尚未显式列出该消息包及 SciPy、OSQP，因此仅执行 `rosdep` 不保证这些依赖全部齐备。请确认运行节点的 Python/ROS 环境可以导入它们。

## 构建

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

每次打开新终端后，都需要重新加载 ROS 2 和当前工作空间环境：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

## 运行示例

### 1. 启动两个 turtlesim

```bash
ros2 launch my_turtle_launch turtlesim.launch.py
```

两个实例分别位于 `/robot1` 和 `/robot2` 命名空间。可在其他终端发送速度指令：

```bash
ros2 topic pub --rate 2 /robot1/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 1.0}, angular: {z: 0.5}}"
```

将话题改为 `/robot2/cmd_vel` 即可控制第二只海龟。

### 2. 显示三连杆机械臂

```bash
ros2 launch two_link_arm_description display.launch.py
```

当前 `display.launch.py` 只启动 `robot_state_publisher` 和 RViz2，关节滑块 GUI 的启动代码已注释，需要外部节点提供 `/joint_states`。若只做手动展示，可在另一终端运行 `ros2 run joint_state_publisher_gui joint_state_publisher_gui`；不要与 ros2_control 的关节状态广播器同时发布同一组关节状态。若 RViz2 未显示模型，将 `Fixed Frame` 设置为 `base_link`，再添加 `RobotModel` 显示项。

### 3. 运行运动学监视节点

确保已有 `/joint_states` 输入（来自手动展示或下节的 ros2_control 模拟硬件），在另一个已加载工作空间环境的终端执行：

```bash
ros2 run two_link_arm_kinematics kinematics_monitor
```

节点订阅：

- `/joint_states`（`sensor_msgs/msg/JointState`）
- `/desired_cartesian_velocity`（`geometry_msgs/msg/Twist`）

发送期望末端笛卡尔速度，节点会以当前关节状态构造并求解一次约束 QP：

```bash
ros2 topic pub --once /desired_cartesian_velocity geometry_msgs/msg/Twist \
  "{linear: {x: 0.05, y: 0.02}}"
```

当前正式控制流程为：

1. 根据关节位置硬限位、最大关节速度和速度阻尼器计算动态速度上下界。
2. 根据末端位置与固定障碍物的距离决定是否激活避障约束。
3. 构造以末端速度跟踪误差为目标的 QP，并为避障约束加入有上下界的 Slack 变量。
4. 使用 OSQP 求解关节速度，随后检查任务误差、约束余量以及 KKT 驻点、互补松弛、原始可行性和对偶可行性条件。

当前参数包括：控制步长 `0.01 s`、关节速度上限 `0.05 rad/s`、速度阻尼安全距离 `0.5 rad`；障碍物位于 `(0.95, 0.40) m`，影响距离为 `0.20 m`，安全距离为 `0.05 m`，避障 Slack 上限为 `0.005`。节点目前仍由 `/desired_cartesian_velocity` 消息触发一次求解，并未建立独立的 `100 Hz` 定时控制循环。

文件中还保留以下研究实验函数，但默认不参与上述正式 QP 主线：

- Moore-Penrose、自适应 DLS 与不同零空间投影方式的比较
- 严格任务优先级递归
- 人工构造的不可行 QP
- 单 Slack 与多 Slack 松弛变量实验

如需复现实验，应按代码末尾的说明临时启用对应 `experiment_*()` 调用；实验完成后重新注释该调用。

### 4. ros2_control 轨迹执行与监控

当前 URDF 使用 `mock_components/GenericSystem` 模拟硬件，不是 Gazebo 动力学仿真或真实机械臂驱动。`config/controllers.yaml` 将控制器管理器更新频率设为 `100 Hz`，三个关节使用位置命令接口及位置、速度状态接口。

在已加载环境的终端 A 启动控制器与 RViz2（该启动文件已包含 `robot_state_publisher`，不必同时运行 `display.launch.py`）：

```bash
ros2 launch two_link_arm_description ros2_control.launch.py
```

在终端 B 确认 `joint_state_broadcaster` 和 `joint_trajectory_controller` 均为 `active`，然后先启动监控器：

```bash
ros2 control list_controllers
ros2 run two_link_arm_kinematics trajectory_monitor
```

在终端 C 启动轨迹生成器：

```bash
ros2 run two_link_arm_kinematics trajectory_generator
```

生成器启动约 1 秒后只发布一次 `trajectory_msgs/msg/JointTrajectory`，目标话题为 `/joint_trajectory_controller/joint_trajectory`；消息包含关节名称、位置、速度、加速度和各点的 `time_from_start`。监控器应先启动，以便接收这条一次性发布的轨迹。需要重发时重新启动生成器。

生成器支持两种模式，当前 `test_mode` 在源码中设为 `common`，尚未暴露为 ROS 参数：

- `common`：所有关节共用路径参数 `s(t)`，按 `q(t) = q_start + s(t) * (q_goal - q_start)` 生成关节空间直线路径。
- `independent`：各关节独立规划梯形/三角速度轨迹，再按最长持续时间缩放，使所有关节同时到达；这不等于共用相同的路径进度。

当前示例使用 `q_start = [0, 0, 0]`、`q_goal = [1.0, -0.5, 0.8]`，各关节最大速度为 `0.5 rad/s`、最大加速度为 `1.0 rad/s²`，采样间隔为 `0.01 s`。这些值是轨迹实验参数，与 QP 节点中的速度约束不同。起点是硬编码示例值，不会自动读取实际关节位置；不要将此示例直接接入真实硬件。

`trajectory_monitor` 同时订阅期望轨迹和 `/joint_states`，以消息时间戳对齐轨迹时间，在相邻轨迹点之间线性插值期望位置，按关节名称重排反馈位置，每 `0.2 s` 输出 `q_des - q_actual` 及其范数。该监控器采用位置线性插值，其结果不应视为控制器内部插值误差的精确复现。

`kinematics_monitor.py` 仍保留大量三次、五次及同步轨迹的历史注释代码，相关启动实验和轨迹发布器当前均已注释；实际轨迹生成与发布由独立的 `trajectory_generator` 节点承担。

## 测试

```bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

## 目录结构

```text
ros2_ws/
├── README.md
├── reference/
│   └── finite_difference_velocity_reference.py
└── src/
    ├── my_turtle_launch/
    ├── two_link_arm_description/
    └── two_link_arm_kinematics/
```

`reference/finite_difference_velocity_reference.py` 是带角度展开、时间间隔检查和低通滤波的差分速度参考实现，不会被当前节点自动加载。`build/`、`install/` 和 `log/` 是 `colcon` 生成目录，已通过 `.gitignore` 排除，不需要提交到仓库。
