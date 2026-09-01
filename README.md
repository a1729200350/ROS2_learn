# ROS2_learn

一个面向 ROS 2 入门与机械臂运动学实验的学习工作空间，当前包含双 `turtlesim` 启动、三连杆机械臂 URDF/RViz 展示，以及三连杆正运动学、雅可比、奇异性分析与约束逆运动学节点。

## 功能包

| 功能包 | 类型 | 内容 |
| --- | --- | --- |
| `my_turtle_launch` | `ament_python` | 同时启动两个带命名空间的 `turtlesim` 节点，分别使用红色和蓝色背景 |
| `two_link_arm_description` | `ament_cmake` | 三连杆平面机械臂 URDF，并启动 `robot_state_publisher`、关节滑块 GUI 和 RViz2 |
| `two_link_arm_kinematics` | `ament_python` | 订阅关节状态和期望笛卡尔速度，计算三连杆运动学状态，并使用 OSQP 求解带动态关节速度边界、速度阻尼器和软避障 Slack 的约束逆运动学问题；保留 DLS、严格任务优先级和 Slack 对比实验 |

三连杆模型的连杆长度为 `L1 = 0.5 m`、`L2 = 0.4 m`、`L3 = 0.4 m`，与 URDF 和运动学节点中的参数一致。

## 环境

- Ubuntu 24.04 或 WSL2 Ubuntu
- ROS 2 Jazzy
- Python 3、NumPy、SciPy、OSQP
- RViz2、`joint_state_publisher_gui`、`robot_state_publisher` 和 `turtlesim`

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

启动后可通过关节状态发布器中的 `joint1`、`joint2`、`joint3` 滑块改变机械臂姿态，并在 RViz2 中观察模型。若 RViz2 初次未显示模型，请将 `Fixed Frame` 设置为 `base_link`，再添加 `RobotModel` 显示项。

### 3. 运行运动学监视节点

保持三连杆显示程序运行，在另一个已加载工作空间环境的终端执行：

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
