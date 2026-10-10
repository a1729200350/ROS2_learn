# ROS2_learn

一个面向 ROS 2 入门与机械臂运动学实验的学习工作空间，包含双 `turtlesim`、三连杆机械臂 URDF/RViz、约束逆运动学、动力学、操作空间控制与离线仿真，以及 ros2_control 模拟硬件和 Gazebo 模型配置。

## 功能包

| 功能包 | 类型 | 内容 |
| --- | --- | --- |
| `my_turtle_launch` | `ament_python` | 同时启动两个带命名空间的 `turtlesim` 节点，分别使用红色和蓝色背景 |
| `three_link_arm_description` | `ament_cmake` | 三连杆 URDF、RViz、ros2_control 模拟硬件与位置轨迹控制配置，以及 Gazebo 专用 URDF、启动文件和力矩控制器配置 |
| `three_link_arm_kinematics` | `ament_python` | 运动学、动力学、关节空间控制、轨迹生成与监控，以及操作空间控制 ROS 2 节点、计算模块与离线实验 |

三连杆模型的连杆长度为 `L1 = 0.5 m`、`L2 = 0.4 m`、`L3 = 0.4 m`，与 URDF 和运动学节点中的参数一致。

## 文档导航

| 目录文档 | 适合查看的内容 |
| --- | --- |
| [模型、启动与参数](src/three_link_arm_description/README.md) | 三种启动方式、控制器配置和 OSC 参数表 |
| [运动学与控制功能包](src/three_link_arm_kinematics/README.md) | ROS 节点入口、计算模块、跟踪消息字段和实验边界 |
| [辅助脚本](scripts/README.md) | 控制器状态监视、故障暂停条件和录包绘图 |
| [OSC 实验记录](osc_records/README.md) | 两组 MCAP 数据、话题统计和已保存的曲线 |

## 环境

- Ubuntu 24.04 或 WSL2 Ubuntu
- ROS 2 Jazzy
- Python 3、NumPy、SciPy、OSQP；离线分析 CSV 与绘图还需要 pandas、Matplotlib
- RViz2、`joint_state_publisher_gui`、`robot_state_publisher` 和 `turtlesim`
- ros2_control、ros2_controllers、`controller_manager`、`controller_manager_msgs`、`trajectory_msgs`、`std_msgs`、`std_srvs`
- Gazebo 示例额外需要 `ros_gz_sim`、`ros_gz_bridge`、`gz_ros2_control`；查看 MCAP 实验录包需要对应的 rosbag2 存储插件

## 获取代码与安装依赖

```bash
cd ~
git clone git@github.com:a1729200350/ROS2_learn.git ros2_ws
cd ros2_ws
```

当前 Gazebo 启动文件通过固定路径 `~/ros2_ws/scripts/osc_controller_state_watch.py` 启动监视脚本，绘图脚本也固定读取 `~/ros2_ws/osc_records/baseline_01`，因此上面的目录名称与位置有实际意义。已有工作空间不必重新克隆；使用其他路径时需先核对这两处路径假设。

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
python -m pip install numpy scipy osqp pandas matplotlib
```

`.venv/` 仅用于本机运行环境，不需要提交到仓库。

运动学包已声明 `trajectory_msgs`、SciPy，以及 OSC 服务使用的 `std_srvs` 和自动激活使用的 `controller_manager_msgs`。QP 节点使用 OSQP，力矩与跟踪消息使用 `std_msgs`，但依赖清单目前未显式列出这两项。请确认运行环境已安装并可导入它们。

Gazebo 启动文件和 URDF 使用的 `ros_gz_sim`、`ros_gz_bridge`、`gz_ros2_control` 当前也未在描述包的依赖清单中显式声明，运行该示例前需单独确认环境具备这些包。

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
ros2 launch three_link_arm_description display.launch.py
```

当前 `display.launch.py` 只启动 `robot_state_publisher` 和 RViz2，关节滑块 GUI 的启动代码已注释，需要外部节点提供 `/joint_states`。若只做手动展示，可在另一终端运行 `ros2 run joint_state_publisher_gui joint_state_publisher_gui`；不要与 ros2_control 的关节状态广播器同时发布同一组关节状态。若 RViz2 未显示模型，将 `Fixed Frame` 设置为 `base_link`，再添加 `RobotModel` 显示项。

### 3. 运行运动学监视节点

确保已有 `/joint_states` 输入（来自手动展示或下节的 ros2_control 模拟硬件），在另一个已加载工作空间环境的终端执行：

```bash
ros2 run three_link_arm_kinematics kinematics_monitor
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

本节使用的 URDF 采用 `mock_components/GenericSystem` 模拟硬件，不是 Gazebo 动力学仿真或真实机械臂驱动。`config/controllers.yaml` 将控制器管理器更新频率设为 `100 Hz`，三个关节使用位置命令接口及位置、速度状态接口。Gazebo 专用配置见第 10 节。

在已加载环境的终端 A 启动控制器与 RViz2（该启动文件已包含 `robot_state_publisher`，不必同时运行 `display.launch.py`）：

```bash
ros2 launch three_link_arm_description ros2_control.launch.py
```

在终端 B 确认 `joint_state_broadcaster` 和 `joint_trajectory_controller` 均为 `active`，然后先启动监控器：

```bash
ros2 control list_controllers
ros2 run three_link_arm_kinematics trajectory_monitor
```

在终端 C 启动轨迹生成器：

```bash
ros2 run three_link_arm_kinematics trajectory_generator
```

生成器启动约 1 秒后只发布一次 `trajectory_msgs/msg/JointTrajectory`，目标话题为 `/joint_trajectory_controller/joint_trajectory`；消息包含关节名称、位置、速度、加速度和各点的 `time_from_start`。监控器应先启动，以便接收这条一次性发布的轨迹。需要重发时重新启动生成器。

生成器支持两种模式，当前 `test_mode` 在源码中设为 `common`，尚未暴露为 ROS 参数：

- `common`：所有关节共用路径参数 `s(t)`，按 `q(t) = q_start + s(t) * (q_goal - q_start)` 生成关节空间直线路径。
- `independent`：各关节独立规划梯形/三角速度轨迹，再按最长持续时间缩放，使所有关节同时到达；这不等于共用相同的路径进度。

当前示例使用 `q_start = [0, 0, 0]`、`q_goal = [1.0, -0.5, 0.8]`，各关节最大速度为 `0.5 rad/s`、最大加速度为 `1.0 rad/s²`，采样间隔为 `0.01 s`。这些值是轨迹实验参数，与 QP 节点中的速度约束不同。起点是硬编码示例值，不会自动读取实际关节位置；不要将此示例直接接入真实硬件。

`trajectory_monitor` 同时订阅期望轨迹和 `/joint_states`，以消息时间戳对齐轨迹时间，在相邻轨迹点之间线性插值期望位置，按关节名称重排反馈位置，每 `0.2 s` 输出 `q_des - q_actual` 及其范数。该监控器采用位置线性插值，其结果不应视为控制器内部插值误差的精确复现。

`kinematics_monitor.py` 仍保留大量三次、五次及同步轨迹的历史注释代码，相关启动实验和轨迹发布器当前均已注释；实际轨迹生成与发布由独立的 `trajectory_generator` 节点承担。

### 5. 动力学与关节空间控制实验

`three_link_arm_kinematics` 还提供 `dynamics_monitor`、`joint_space_control_monitor` 和 `computed_torque_control_monitor` 三个命令。启动上一节的 ros2_control 模拟硬件和轨迹生成器后，可分别在新终端运行：

```bash
ros2 run three_link_arm_kinematics dynamics_monitor
ros2 run three_link_arm_kinematics joint_space_control_monitor
ros2 run three_link_arm_kinematics computed_torque_control_monitor
```

`dynamics_monitor` 从关节状态和轨迹消息计算动力学量；`joint_space_control_monitor` 以 100 Hz 计算关节空间控制结果。两者用于观察和记录实验结果。`computed_torque_control_monitor` 以 100 Hz 计算力矩并发布 `std_msgs/msg/Float64MultiArray` 到 `/joint_effort_command`。当前 `controllers.yaml` 使用位置命令接口，因此该力矩话题不会直接驱动这里的轨迹控制器；不要把它当成已接通的力矩控制闭环。

当前 `computed_torque_control_monitor` 的关节控制增益在源码中设置为 `kp = [10, 10, 10]`、`kd = [4.43, 4.43, 4.43]`，尚未暴露为 ROS 参数。

动力学计算集中在 `dynamics_model.py`，`manual_tests/` 包含重力、质量矩阵、速度项和关节空间控制的手动实验脚本。

### 6. 独立关节动力学仿真与跟踪数据分析

`joint_dynamics_simulator` 从 `/joint_effort_command` 接收三个关节的力矩，按 `dynamics_model.py` 的模型以 `0.01 s` 步长积分，并发布 `/joint_states`。它只在收到首条有效力矩后开始积分；力矩命令超过 `0.1 s` 未更新时会暂停积分，本轮节点不会自动恢复。此节点是独立的软件动力学实验，不要与上一节的 ros2_control 模拟硬件同时向 `/joint_states` 发布状态。

```bash
ros2 run three_link_arm_kinematics joint_dynamics_simulator
```

仿真器支持 ROS 参数 `initial_q`，默认为 `[0.0, 0.0, 0.0]`，必须包含三个有限关节角；初始关节速度为零。运行下文 OSC 实验时需要使用非奇异初始构型。

`computed_torque_control_monitor` 在发布力矩的同一次控制计算中，还会向 `/computed_torque_tracking` 发布 `std_msgs/msg/Float64MultiArray`。消息依次包含 13 个数值：`t, q1, q2, q3, qd1, qd2, qd3, e1, e2, e3, tau1, tau2, tau3`，其中时间、期望位置和误差均由控制器提供。

`trajectory_tracking_analyzer` 现在只订阅 `/computed_torque_tracking`，收到一条有效消息就向启动目录下的 `tracking_data.csv` 写入一行并刷新文件；长度不是 13 或包含 NaN/Inf 的消息会被拒绝。旧的独立计时、轨迹插值和定时记录代码保留为注释，当前不会执行。退出时会关闭 CSV 文件。该脚本以写入模式打开 CSV，重新运行会覆盖启动目录中的同名文件。

```bash
ros2 run three_link_arm_kinematics trajectory_tracking_analyzer
```

记录完整实验时，先启动记录器、动力学仿真器和计算力矩节点，再启动轨迹生成器。记录频率取决于控制器实际发布消息的频率，控制器定时周期为 `0.01 s`。

仓库当前不再附带根目录的旧 `tracking_data.csv` 和 `tracking_kd_2.csv`，历史版本仍可在 Git 提交记录中查看。以下离线分析脚本固定读取启动目录中的 `tracking_data.csv`，需要先用记录器生成数据；CSV 本身未记录控制增益，不能仅凭当前源码参数判断采集时的增益。脚本不是 ROS 2 命令行入口；在工作空间根目录运行，使用 pandas 输出三个关节在运动阶段（`0–2.5 s`）、稳态阶段（`>3 s`）的最大/RMS 跟踪误差，以及最大力矩：

```bash
python3 src/three_link_arm_kinematics/three_link_arm_kinematics/analyze_tracking_error.py
```

脚本还使用 Matplotlib 显示 5 张图：三个关节各自的位置跟踪对比图、三个关节共用的误差图和力矩图。当前调用 `plt.show()` 显示窗口，不会自动保存图片；交互查看需要可用的图形显示环境。运动/稳态分界时间在脚本中固定，分析其他轨迹前应核对这些时间是否适用。

### 7. 操作空间运动学、动力学与控制实验

以下模块以平面三连杆机械臂的末端位置 `[x, y]` 为二维任务，不包含末端姿态控制：

| 模块 | 当前实现 |
| --- | --- |
| `operational_space_kinematics.py` | 正运动学、`J`、解析 `J_dot`，以及 `x_dot = J q_dot`、`x_ddot = J q_ddot + J_dot q_dot` |
| `operational_space_controller.py` | 任务空间前馈加速度加 PD 反馈，通过 Moore-Penrose 伪逆将修正后的任务加速度映射到关节加速度；默认 `kp = (10, 10)`、`kd = (4.43, 4.43)` |
| `operational_space_dynamics.py` | 操作空间惯量 `Lambda`、动态一致广义逆 `J_bar`、偏差项 `mu`/`p`、任务力及 `tau = J.T @ F`，以及动态一致零空间投影 `N`/`N.T`；可通过 `dynamics` 参数传入动力学模型，省略时使用默认模型 |

这些 NumPy 计算类由离线实验及新增的 `operational_space_control_monitor` ROS 2 节点使用；前述关节空间计算力矩节点仍使用其原有控制流程。操作空间惯量计算使用直接线性求解，要求 `J M^-1 J.T` 可逆；当前没有奇异构型阻尼处理。

`manual_tests/` 中提供以下离线数值实验，使用脚本内固定的关节状态并打印结果：

| 脚本 | 实验内容 |
| --- | --- |
| `test_operational_space_kinematics.py` | 用有限差分核对 `J_dot` 和末端加速度 |
| `test_operational_space_controller.py` | 任务空间 PD 加速度命令与关节加速度映射 |
| `test_operational_space_inverse_dynamics.py` | 关节逆动力学、加速度恢复和动力学方程残差 |
| `test_operational_space_dynamics.py` | 操作空间惯量、动态一致广义逆与普通伪逆对比 |
| `test_operational_space_bias.py` | 操作空间偏差项、任务力与加速度恢复 |
| `test_dynamic_null_space.py` | 投影幂等性、`J N = 0`、`J M^-1 N.T = 0`、主任务叠加零空间力矩，以及零空间姿态 PD 控制后的任务加速度核对 |

在工作空间根目录运行，例如：

```bash
PYTHONPATH=src/three_link_arm_kinematics python3 -B src/three_link_arm_kinematics/manual_tests/test_operational_space_kinematics.py
```

将命令末尾文件名替换为表中其他脚本即可运行对应实验。它们目前主要打印数值供检查，不等同于带断言的自动化测试，也不验证 ROS 闭环或硬件运行。

### 8. 操作空间闭环、模型误差与奇异点实验

`manual_tests/` 新增连续离线仿真实验，使用半隐式欧拉积分。默认仿真时长为 `4 s`，前 `2 s` 使用五次时间缩放移动末端，之后保持终点；动力学积分步长为 `1 ms`，两次控制更新之间保持上一条力矩。

| 脚本 | 当前实验 |
| --- | --- |
| `test_operational_space_closed_loop_simulation.py` | 在相同初始状态下比较 `2 / 5 / 10 / 20 ms` 控制周期，输出任务误差、最大力矩及关节速度，并绘制误差范数对比图 |
| `test_operational_space_model_mismatch.py` | 控制器使用标称模型，仿真机器人采用标称参数或将第二连杆质量、惯量增加 `10%`；分别开启/关闭零空间姿态任务，共四组对比，并输出最终状态、最小奇异值和最大关节速度 |
| `test_operational_space_singularity.py` | 比较正常、较直、接近伸直和完全伸直初始构型，沿末端径向向内移动 `0.05 m`；初始 `sigma_min(J) < 1e-8` 时跳过严格逆求解，记录其余工况的误差、速度、力矩与最小奇异值 |

`simulation_core.py` 为模型误差与奇异点实验提供公共 `run_osc_simulation()` 函数，将仿真机器人模型与控制器模型分开，并允许设置控制周期、积分步长、初始关节位置、目标位移及零空间姿态任务开关。它要求控制周期是积分步长的正整数倍，返回时间、误差、关节状态、力矩和最小奇异值数组。默认控制周期为 `2 ms`，默认末端位移为 `[0.05, -0.03] m`。

在工作空间根目录运行：

```bash
PYTHONPATH=src/three_link_arm_kinematics python3 -B src/three_link_arm_kinematics/manual_tests/test_operational_space_closed_loop_simulation.py
PYTHONPATH=src/three_link_arm_kinematics python3 -B src/three_link_arm_kinematics/manual_tests/test_operational_space_model_mismatch.py
PYTHONPATH=src/three_link_arm_kinematics python3 -B src/three_link_arm_kinematics/manual_tests/test_operational_space_singularity.py
```

这些脚本打印指标并使用 Matplotlib 显示对比图，不会自动保存 CSV 或图片。无图形显示环境时，可在命令前加 `MPLBACKEND=Agg` 运行数值部分。奇异点实验会打印并跳过初始奇异或运行失败的工况；这属于边界探查，尚未加入阻尼逆或在线奇异性避让。这些实验验证离线模型中的行为，不代表 ROS 节点闭环或真实硬件验证。

### 9. ROS 2 操作空间控制与实验录包

`operational_space_control_monitor` 订阅 `/joint_states`，按关节名称读取位置与速度，以 `0.01 s` 定时周期计算主任务与零空间力矩，并发布到 `/joint_effort_command`。首次有效反馈且 ROS 时钟可用后，节点建立末端起点并进入保持模式；调用 `/osc_start` 后，才用 `2 s` 五次时间缩放移动 `[0.01, -0.006] m`，随后保持目标。等待服务调用期间仍会发布保持控制力矩。

节点已将重力、主任务/姿态增益、姿态目标、末端位移、轨迹时长和逐关节力矩限制暴露为启动时 ROS 参数。[OSC 参数表](src/three_link_arm_description/README.md#osc-启动参数)列出了当前配置；Gazebo 启动会自动加载 `config/osc_params.yaml`，直接 `ros2 run` 不会自动加载该文件。参数在构造节点时读取，当前没有控制参数热更新回调。

默认动力学重力参数为 `g = 9.8`，零空间期望关节位置为 `[0.5, -0.8, 1.2]`，零空间原始力矩为 `G + Kp_posture (q_desired - q) - Kd_posture q_dot`，再经 `N.T` 投影。主任务增益默认为 `kp = (10, 10)`、`kd = (4.43, 4.43)`，姿态增益为 `Kp_posture = I`、`Kd_posture = 0.5 I`。

`task_space_trajectory.py` 提供共用的 `sample_task_trajectory()` 函数，供该 ROS 节点与 `simulation_core.py` 使用：返回期望位置、速度和加速度，要求持续时间大于零，负时间按起点处理，轨迹结束后保持终点且期望速度、加速度为零。

完成构建并加载环境后，在终端 A 启动独立动力学仿真器，设置非奇异初始构型：

```bash
ros2 run three_link_arm_kinematics joint_dynamics_simulator --ros-args -p initial_q:="[0.5, -0.8, 1.2]"
```

在终端 B 的工作空间根目录启动 OSC 节点：

```bash
ros2 run three_link_arm_kinematics operational_space_control_monitor --ros-args \
  --params-file src/three_link_arm_description/config/osc_params.yaml
```

看到节点输出“OSC 保持模式”后，在另一个已加载环境的终端启动轨迹：

```bash
ros2 service call /osc_start std_srvs/srv/Trigger "{}"
```

服务返回 `success: true` 表示轨迹已启动；节点未就绪、处于故障状态或轨迹已经启动时会拒绝请求，同一次节点运行不支持再次触发轨迹。

节点另提供 `/osc_ready`（`std_srvs/srv/Trigger`），供 Gazebo 自动激活流程检查就绪状态。它要求节点无故障、反馈及正常力矩已产生，反馈年龄不超过 `0.2 s`、正常力矩年龄不超过 `0.1 s`、轨迹尚未启动、各关节速度绝对值不超过 `0.1 rad/s`，且未经限幅的原始需求力矩均不超过对应的 `torque_limit`（默认每关节 `10 N·m`）。因此，限幅后的输出处于范围内不代表已就绪。检查失败时返回具体原因；该服务本身不激活控制器，也不启动轨迹。`/osc_start` 的回调没有复用这些完整就绪检查。

这个实验由 OSC 节点内部生成任务轨迹，无需 `trajectory_generator`。运行时只保留这一套状态源和力矩源，不要同时启动 ros2_control/Gazebo 状态发布器或另一个计算力矩节点。

节点通过 `/osc_tracking` 发布 16 个数值，顺序为 `t, x_d(2), x(2), error(2), q(3), q_dot(3), tau(3)`，类型为 `std_msgs/msg/Float64MultiArray`；[字段索引](src/three_link_arm_kinematics/README.md#osc-接口)供离线读取使用。`tau` 为实际发布的限幅后力矩指令，不是原始需求力矩，也不是测量力矩。等待启动时 `t = 0`；统计从成功触发轨迹后开始，在轨迹时间约 `4 s` 时打印一次误差及力矩统计，等待时间不计入 RMS，之后继续保持目标。现有 `trajectory_tracking_analyzer` 只接收 `/computed_torque_tracking` 的 13 字段数据，不能直接记录这个新话题。

当前保护条件包括反馈超过 `0.2 s` 未更新、`sigma_min(J) < 1e-6`、计算失败、非有限力矩或任一力矩绝对值超过 `100`。触发后节点锁定，立即发布 `[0, 0, 0]`，并在后续定时回调中继续发布零力矩，不会自动恢复。持续的零力矩消息不会触发独立动力学仿真器的命令超时暂停；零力矩也不等于保持关节姿态。

上述异常值检查通过后，节点才将各关节力矩裁剪到 `[-torque_limit, +torque_limit]`，饱和警告最多每秒打印一次。`100 N·m` 异常值阈值与可配置的执行器限幅是两层不同检查；超过前者会锁定故障，不会仅靠限幅继续运行。

轨迹时间使用节点的 ROS 时钟；Gazebo 实验需设置 `use_sim_time:=true` 并确保 `/clock` 桥接有效，独立软件仿真默认使用系统时间。反馈超时及耗时诊断仍采用 `time.monotonic()`。超时日志包含反馈年龄、回调间隔、上一轮完整控制耗时和上一轮力矩发布耗时。

仓库内 `osc_ros2_run1/` 保存一份历史 MCAP 实验录包及 `metadata.yaml`，约 `68.65 s`、共 `17,506` 条消息，仅包含 `/joint_states`、`/joint_effort_command` 和 `/osc_tracking`。该录包未随本次控制逻辑更新而重新采集。可只读查看录包信息：

```bash
ros2 bag info osc_ros2_run1
```

新增的 `osc_records/` 保存 `baseline_01`、`saturation_01` 两组录包和三张基线曲线，详见[实验记录说明](osc_records/README.md)。这些数据包含 `/effort_controller/commands`，与上述历史录包的力矩话题不同；目录名称和当前 YAML 配置不能单独证明录制时的实际参数。

### 10. Gazebo 模型与自动激活流程

`three_link_arm_gazebo.urdf` 包含世界固定基座、碰撞与惯性参数，并通过 `gz_ros2_control/GazeboSimSystem` 提供三个关节的 effort 命令接口和 position/velocity/effort 状态接口；初始关节位置配置为 `[0.5, -0.8, 1.2]`。

URDF 还为 Gazebo 配置了 `link1`、`link2` 的红色材质和 `link3` 的绿色材质。

`config/three_link_arm_controllers.yaml` 配置 `100 Hz` 控制器管理器、`joint_state_broadcaster` 和 `effort_controller`（`effort_controllers/JointGroupEffortController`），与第 4 节的位置轨迹控制配置分开。

```bash
ros2 launch three_link_arm_description three_link_arm_gazebo.launch.py
```

启动文件启动 Gazebo 空世界和 `robot_state_publisher`，通过 `ros_gz_bridge/parameter_bridge` 将 Gazebo 时钟单向桥接到 ROS `/clock`，并按以下顺序组织后续节点：

1. 模型生成进程成功退出后，启动 `joint_state_broadcaster` 的 spawner。
2. 状态广播器 spawner 成功退出后，加载 `effort_controller`，先保持 `inactive`。
3. 力矩控制器 spawner 成功退出后，同时启动 OSC 节点和 `osc_auto_activator`。OSC 加载描述包中的 `config/osc_params.yaml`，设置 `use_sim_time: true`，并将 `/joint_effort_command` 重映射到 `/effort_controller/commands`。
4. `osc_auto_activator` 等待 `/osc_ready` 检查通过，再通过 `/controller_manager/switch_controller` 以 `STRICT` 模式激活 `effort_controller`。

两次 spawner 均配置了 `120 s` 的控制器管理器等待超时；前置进程非零退出时，不启动对应的下一步。自动激活节点的就绪等待期限为 `30 s`，切换请求的服务端超时为 `3 s`、客户端等待为 `5 s`；等待超时或切换失败会记录错误并退出，不会自动重试整个启动流程。

自动激活只接通力矩控制器，不会调用 `/osc_start`。看到“effort_controller 自动激活成功”后，可先确认两个控制器均为 `active`，再手动触发轨迹：

```bash
ros2 control list_controllers
ros2 service call /osc_start std_srvs/srv/Trigger "{}"
```

此启动文件已包含 OSC 节点及控制器启动流程，不需要再单独启动第 9 节的独立动力学仿真器或第二个 OSC 节点。源码中的启动顺序和就绪门槛不等于运行结果，是否成功激活仍需以本次日志和控制器状态为准。

启动文件还会独立启动 `scripts/osc_controller_state_watch.py`。监视器只在观察到 `effort_controller` 激活后进入保护阶段：随后控制器失去 `active`、从未收到跟踪消息且激活已超过 `2 s`，或已收到的 `/osc_tracking` 中断超过 `0.5 s`，都会触发一次故障锁定并请求暂停 Gazebo 的 `empty` 世界，再读取世界状态确认暂停。它不自动恢复或重新激活控制器；完整条件、路径依赖和暂停失败日志见[辅助脚本说明](scripts/README.md)。

## 测试

```bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

## 目录结构

```text
ros2_ws/
├── README.md
├── scripts/
│   ├── README.md
│   ├── osc_controller_state_watch.py
│   └── plot_osc_error.py
├── osc_records/
│   ├── README.md
│   ├── baseline_01/
│   ├── saturation_01/
│   ├── baseline_01_error.png
│   ├── baseline_01_torque.png
│   └── baseline_01_velocity.png
├── osc_ros2_run1/
│   ├── metadata.yaml
│   └── osc_ros2_run1_0.mcap
├── reference/
│   └── finite_difference_velocity_reference.py
└── src/
    ├── my_turtle_launch/
    ├── three_link_arm_description/   # README、launch、config、urdf
    └── three_link_arm_kinematics/    # README、节点、计算模块与实验
```

`reference/finite_difference_velocity_reference.py` 是带角度展开、时间间隔检查和低通滤波的差分速度参考实现，不会被当前节点自动加载。`build/`、`install/` 和 `log/` 是 `colcon` 生成目录，已通过 `.gitignore` 排除，不需要提交到仓库。
