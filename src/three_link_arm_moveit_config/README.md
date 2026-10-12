# three_link_arm_moveit_config

三连杆机械臂的 MoveIt 配置包，包含 Setup Assistant 生成的配置与组件启动入口。返回[工作空间说明](../../README.md)；模型和独立 Gazebo JTC 实验见[描述包](../three_link_arm_description/README.md)。

## 当前配置

| 项目 | 内容 |
| --- | --- |
| 机器人名称 | `three_link_arm` |
| 规划组 | `arm`，由 `arm_base` 到 `tool0` 的链组成 |
| 命名姿态 | `home`：`joint1 = 0.5`、`joint2 = -0.8`、`joint3 = 1.2` rad |
| 虚拟关节 | 固定的 `world_joint`，连接 `world` 与 `base_link` |
| 控制器映射 | `joint_trajectory_controller`，动作类型 `FollowJointTrajectory`，`action_ns: follow_joint_trajectory` |
| 默认速度/加速度缩放 | 均为 `0.1`；不等于全程实测速度或加速度 |

## 文件索引

| 文件 | 用途与当前状态 |
| --- | --- |
| [`config/three_link_arm.srdf`](config/three_link_arm.srdf) | 规划组、`home` 姿态、虚拟关节和相邻连杆碰撞排除表 |
| [`config/kinematics.yaml`](config/kinematics.yaml) | 当前为 `{}`，没有显式配置逆运动学求解器 |
| [`config/joint_limits.yaml`](config/joint_limits.yaml) | 各关节最大速度 `2 rad/s`；加速度上限开关当前为 `false` |
| [`config/moveit_controllers.yaml`](config/moveit_controllers.yaml) | MoveItSimpleControllerManager 到 JTC 动作接口的映射 |
| [`config/ros2_controllers.yaml`](config/ros2_controllers.yaml) | `100 Hz` 管理器、状态广播器与 position 型 JTC 配置 |
| [`config/three_link_arm.urdf.xacro`](config/three_link_arm.urdf.xacro) | 包含描述包基础 URDF，并调用本包控制 Xacro |
| [`config/three_link_arm.ros2_control.xacro`](config/three_link_arm.ros2_control.xacro) | 生成 `mock_components/GenericSystem`、position 命令与 position/velocity 状态接口；不是 Gazebo 系统 |
| [`config/initial_positions.yaml`](config/initial_positions.yaml) | 生成的 mock 系统初值 `[0, 0, 0]`，与命名姿态 `home` 不同 |
| [`config/pilz_cartesian_limits.yaml`](config/pilz_cartesian_limits.yaml) | Pilz 笛卡尔运动限制配置 |
| [`config/moveit.rviz`](config/moveit.rviz) | MoveIt RViz 显示配置 |
| [`.setup_assistant`](.setup_assistant) | Setup Assistant 的配置来源与生成设置 |

`home` 是可选择的命名姿态，不表示机器人会自动移动到该位置。SRDF 中的碰撞排除表也不能替代对具体规划场景的验证。

## 已有启动入口

以下文件调用 `moveit_configs_utils` 的对应生成函数，不是五个需要无条件一起运行的进程：

| 文件 | 组件用途 |
| --- | --- |
| [`launch/move_group.launch.py`](launch/move_group.launch.py) | MoveIt 规划组件 |
| [`launch/moveit_rviz.launch.py`](launch/moveit_rviz.launch.py) | MoveIt RViz 界面 |
| [`launch/rsp.launch.py`](launch/rsp.launch.py) | 机器人状态发布组件 |
| [`launch/static_virtual_joint_tfs.launch.py`](launch/static_virtual_joint_tfs.launch.py) | 虚拟关节对应的静态变换 |
| [`launch/setup_assistant.launch.py`](launch/setup_assistant.launch.py) | 配置助手，供后续检查或编辑配置 |

在 ROS 2 Jazzy 环境完成依赖安装、构建并加载工作空间后，可分别使用 `ros2 launch three_link_arm_moveit_config <文件名>` 选择组件。此包目前没有 `demo.launch.py` 或统一的 Gazebo/控制器/规划器启动文件；单独启动一个组件不保证规划或执行已经可用。

## 与 Gazebo JTC 的关系及边界

描述包新增的 JTC 实验使用 `three_link_arm_jtc_gazebo.urdf` 和 `three_link_arm_jtc_controllers.yaml`，提供 Gazebo position 接口并将 JTC 留在 `inactive`。本包的动作映射使用相同控制器名称，但这些文件并没有自动完成激活和联通；MoveIt 的控制器映射本身也不会替代 Gazebo 控制器的启动配置。

当前需要区分以下事项：

- `kinematics.yaml` 尚未指定求解器，不能据此宣称任意末端位姿逆解可用；本模型只有三个转动关节。
- 生成的控制 Xacro 使用 mock 后端，而描述包有单独的 Gazebo 后端。基础 URDF 已含 ros2_control 配置，使用包装 Xacro 前还需核对展开后的硬件定义，不能直接把它当作已验证的 Gazebo 模型。
- 命名姿态、mock 初值及 Gazebo 初值属于不同配置层；不要将 `[0, 0, 0]` 当作 OSC 的非奇异启动姿态。
- 本次文档根据现有文件说明配置，不代表已经运行 MoveIt 规划、控制器激活或 Gazebo 轨迹执行验收。
