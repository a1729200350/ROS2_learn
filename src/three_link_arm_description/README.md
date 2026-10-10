# three_link_arm_description

三连杆机械臂的模型、显示、控制器与 Gazebo 启动配置。返回[工作空间说明](../../README.md)。

## 启动入口

以下命令在工作空间构建完成并加载 ROS 2 与 `install/setup.bash` 后执行。三种入口用于不同实验，不要同时启动多套关节状态源。

| 命令（`ros2 launch three_link_arm_description …`） | 用途 | 状态与控制来源 |
| --- | --- | --- |
| [`display.launch.py`](launch/display.launch.py) | RViz 模型显示 | 只启动状态转换与显示；需外部 `/joint_states`，GUI 启动代码当前已注释 |
| [`ros2_control.launch.py`](launch/ros2_control.launch.py) | 位置轨迹实验 | `mock_components/GenericSystem` 模拟硬件，不进行 Gazebo 动力学仿真 |
| [`three_link_arm_gazebo.launch.py`](launch/three_link_arm_gazebo.launch.py) | Gazebo 力矩控制实验 | Gazebo 状态、OSC 力矩、就绪检查、自动激活及独立状态监视 |

## 模型与配置对应关系

| 文件 | 作用 |
| --- | --- |
| [`urdf/three_link_arm.urdf`](urdf/three_link_arm.urdf) | RViz 与模拟硬件使用的三连杆模型 |
| [`config/controllers.yaml`](config/controllers.yaml) | `joint_state_broadcaster` 与位置型 `joint_trajectory_controller` |
| [`urdf/three_link_arm_gazebo.urdf`](urdf/three_link_arm_gazebo.urdf) | 世界固定基座、碰撞/惯性参数及 `gz_ros2_control` effort 接口；初始关节角 `[0.5, -0.8, 1.2]` |
| [`config/three_link_arm_controllers.yaml`](config/three_link_arm_controllers.yaml) | Gazebo 使用的状态广播器与 `effort_controller`，管理器更新频率 `100 Hz` |
| [`config/osc_params.yaml`](config/osc_params.yaml) | OSC 启动参数；由 Gazebo launch 自动传给控制节点 |

`launch/`、`config/`、`urdf/` 通过 CMake 安装到功能包共享目录。新增配置文件后应重新构建，并确认当前终端加载的是本工作空间。

## OSC 启动参数

以下为当前 `osc_params.yaml` 的值，也与节点内默认值一致：

| 参数 | 当前值 | 含义 |
| --- | --- | --- |
| `gravity` | `9.8` | 控制器动力学模型的重力加速度，m/s² |
| `task_kp` | `[10.0, 10.0]` | 末端 x/y 位置误差增益 |
| `task_kd` | `[4.43, 4.43]` | 末端 x/y 速度误差增益 |
| `posture_target` | `[0.5, -0.8, 1.2]` | 零空间姿态目标，rad；不直接设置仿真初始状态 |
| `posture_kp` | `1.0` | 零空间姿态比例增益，乘以三阶单位矩阵 |
| `posture_kd` | `0.5` | 零空间姿态阻尼增益，乘以三阶单位矩阵 |
| `trajectory_delta` | `[0.01, -0.006]` | 相对首次有效反馈末端位置的 x/y 位移，m |
| `torque_limit` | `[10.0, 10.0, 10.0]` | 各关节输出力矩绝对值上限，N·m；须为三个有限正数 |
| `move_duration` | `2.0` | 调用 `/osc_start` 后的移动时长，s |

参数在节点初始化时读取，当前不支持通过运行时改参数来同步更新控制计算。Gazebo launch 另设 `use_sim_time: true`；单独运行控制节点时，需要显式传入 `--params-file` 才会读取这个 YAML。默认轨迹、原始力矩就绪门槛与限幅后的输出含义见[控制功能包说明](../three_link_arm_kinematics/README.md)。

## Gazebo 启动与检查

当前顺序为：模型生成成功 → 状态广播器 → 加载但不激活力矩控制器 → 启动 OSC 与自动激活器 → `/osc_ready` 通过 → 激活力矩控制器。

```bash
ros2 launch three_link_arm_description three_link_arm_gazebo.launch.py
```

在另一已加载环境的终端确认控制器状态，看到自动激活成功后再手动启动轨迹：

```bash
ros2 control list_controllers
ros2 service call /osc_start std_srvs/srv/Trigger "{}"
```

自动激活不会自动启动轨迹。此 launch 已包含 OSC，不应再运行第二个力矩控制节点或独立的 `joint_dynamics_simulator`。

独立监视器随 launch 启动，使用 `/usr/bin/python3` 执行固定路径 `~/ros2_ws/scripts/osc_controller_state_watch.py`。它的暂停服务与状态检查固定针对 `empty` 世界；工作空间位置和世界名称不同会影响行为。依赖、故障条件及副作用见[脚本说明](../../scripts/README.md)。

本配置用于仿真实验；静态配置、启动顺序和参数检查不等于已经通过 Gazebo 闭环或真实硬件验证。
