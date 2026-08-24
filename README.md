# ROS2_learn

一个面向 ROS 2 入门与机械臂运动学实验的学习工作空间，当前包含双 `turtlesim` 启动、三连杆机械臂 URDF/RViz 展示，以及三连杆正运动学、雅可比与奇异性分析节点。

## 功能包

| 功能包 | 类型 | 内容 |
| --- | --- | --- |
| `my_turtle_launch` | `ament_python` | 同时启动两个带命名空间的 `turtlesim` 节点，分别使用红色和蓝色背景 |
| `two_link_arm_description` | `ament_cmake` | 三连杆平面机械臂 URDF，并启动 `robot_state_publisher`、关节滑块 GUI 和 RViz2 |
| `two_link_arm_kinematics` | `ament_python` | 订阅关节状态，计算末端位置、雅可比、末端速度、奇异值、条件数和可操作度；比较 Moore-Penrose 伪逆、自适应阻尼最小二乘法，并演示零空间关节限位回避 |

三连杆模型的连杆长度为 `L1 = 0.5 m`、`L2 = 0.4 m`、`L3 = 0.4 m`，与 URDF 和运动学节点中的参数一致。

## 环境

- Ubuntu 24.04 或 WSL2 Ubuntu
- ROS 2 Jazzy
- Python 3、NumPy
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

启动后可通过关节状态发布器中的 `joint1`、`joint2` 滑块改变机械臂姿态，并在 RViz2 中观察模型。若 RViz2 初次未显示模型，请将 `Fixed Frame` 设置为 `base_link`，再添加 `RobotModel` 显示项。

### 3. 运行运动学监视节点

保持三连杆显示程序运行，在另一个已加载工作空间环境的终端执行：

```bash
ros2 run two_link_arm_kinematics kinematics_monitor
```

节点订阅：

- `/joint_states`（`sensor_msgs/msg/JointState`）
- `/desired_cartesian_velocity`（`geometry_msgs/msg/Twist`）

发送期望末端笛卡尔速度，可查看普通伪逆和自适应 DLS 的关节速度计算结果：

```bash
ros2 topic pub --once /desired_cartesian_velocity geometry_msgs/msg/Twist \
  "{linear: {x: 0.05, y: 0.02}}"
```

节点会根据最小奇异值自动调整阻尼：当 `sigma_min >= 0.10` 时不使用阻尼，否则在最大阻尼 `0.05` 范围内平滑增大阻尼。关节限位回避速度会投影到雅可比零空间中，因此不会改变主任务的末端速度。

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
