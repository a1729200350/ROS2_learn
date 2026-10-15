# three_link_arm_moveit_config

三连杆机械臂的 MoveIt 配置包，包含 Setup Assistant 生成的配置与组件启动入口。返回[工作空间说明](../../README.md)；模型和独立 Gazebo JTC 实验见[描述包](../three_link_arm_description/README.md)。

本包现在还包含[轨迹工具](tools/README.md)和[离线实验](manual_tests/)，用于将 MoveIt 保存的关节轨迹交给另一套 OSC effort 节点执行；样例及派生数据见[轨迹记录目录](../../moveit_trajectory_records/README.md)。

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
| [`config/joint_limits.yaml`](config/joint_limits.yaml) | 三关节均启用速度和加速度限制，上限分别为 `2.0 rad/s` 和 `1.0 rad/s²` |
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

生成的组件入口使用 `moveit_configs_utils`；新增 Gazebo 入口则显式创建 `move_group` 节点。这些入口按用途选择，不是需要无条件一起运行的进程：

| 文件 | 组件用途 |
| --- | --- |
| [`launch/move_group.launch.py`](launch/move_group.launch.py) | MoveIt 规划组件 |
| [`launch/move_group_gazebo.launch.py`](launch/move_group_gazebo.launch.py) | Gazebo 专用规划组件：仿真时间、允许执行、禁用自动控制器管理 |
| [`launch/moveit_rviz.launch.py`](launch/moveit_rviz.launch.py) | MoveIt RViz 界面 |
| [`launch/rsp.launch.py`](launch/rsp.launch.py) | 机器人状态发布组件 |
| [`launch/static_virtual_joint_tfs.launch.py`](launch/static_virtual_joint_tfs.launch.py) | 虚拟关节对应的静态变换 |
| [`launch/setup_assistant.launch.py`](launch/setup_assistant.launch.py) | 配置助手，供后续检查或编辑配置 |

在 ROS 2 Jazzy 环境完成依赖安装、构建并加载工作空间后，可分别使用 `ros2 launch three_link_arm_moveit_config <文件名>` 选择组件。此包目前没有 `demo.launch.py` 或统一的 Gazebo/控制器/规划器启动文件；单独启动一个组件不保证规划或执行已经可用。

## Gazebo 规划组件入口

先按[描述包说明](../three_link_arm_description/README.md#独立-gazebo-jtc-实验)启动独立 JTC 仿真，再在已加载环境的另一终端运行：

```bash
ros2 launch three_link_arm_moveit_config move_group_gazebo.launch.py
```

此入口的显式配置如下：

| 配置 | 当前值与作用范围 |
| --- | --- |
| `moveit_manage_controllers` | `false`，由现有独立流程负责控制器启动与激活 |
| `use_sim_time` | `true`，使用 Gazebo 桥接的 `/clock` |
| `allow_trajectory_execution` | `true`，允许规划组件发起执行，不表示控制器已自动就绪 |
| 场景与模型发布 | 开启语义模型、规划场景、几何、状态及变换更新发布 |

它只创建一个 `move_group` 节点，不创建第二个 `controller_manager`，也不启动 Gazebo、RViz、状态广播器或 JTC。不要同时再启动普通 `move_group.launch.py`。描述包仍以 `--inactive` 加载 JTC，执行前必须经独立流程确认其为 `active`；单独启动此入口不会完成这一步。

## 与 Gazebo JTC 的关系及边界

本节讨论 MoveIt 动作接口驱动 JTC。另一条文件轨迹路径使用 `three_link_arm_moveit_osc.launch.py`、`moveit_osc_effort_monitor` 和 effort 控制器，不通过 JTC，也不是新注册的 MoveIt 动作控制器。两条路径应分开运行。

描述包的 JTC 实验使用 `three_link_arm_jtc_gazebo.urdf` 和 `three_link_arm_jtc_controllers.yaml`，提供 Gazebo position 接口，并将插件 `position_proportional_gain` 设为 `0.2`。本包的动作映射使用相同控制器名称；MoveIt 的控制器映射本身不会替代 Gazebo 控制器的启动配置，也不会在禁用自动管理时替你激活它。

当前需要区分以下事项：

- `kinematics.yaml` 尚未指定求解器，不能据此宣称任意末端位姿逆解可用；本模型只有三个转动关节。
- 生成的控制 Xacro 使用 mock 后端，而描述包有单独的 Gazebo 后端。基础 URDF 已含 ros2_control 配置，使用包装 Xacro 前还需核对展开后的硬件定义，不能直接把它当作已验证的 Gazebo 模型。
- 命名姿态、mock 初值及 Gazebo 初值属于不同配置层；不要将 `[0, 0, 0]` 当作 OSC 的非奇异启动姿态。
- 仓库保留三组 JTC 参考/反馈状态记录的元数据、曲线与一份分析摘要，完整 MCAP 仅本地保存，见[JTC 实验记录](../../jtc_records/README.md)。已有结果可用于查看对应时段的跟踪情况；重新分析需先取得完整录包，本次文档更新不代表重新运行或完成全部规划/执行验收。

## MoveIt 轨迹到 OSC 的工具与实验

当前流程是采集 `/display_planned_path` 的新规划，保存 `t/q/dq/ddq`，转换为 `arm_base` 下的二维末端参考，再做 `100 Hz` 五次 Hermite 插值、动力学一致性检查和离线闭环实验，最后由文件驱动 ROS 节点执行。具体命令、依赖与文件覆盖行为见[工具 README](tools/README.md)。

`tools/` 和 `manual_tests/` 目前没有在 CMake 中安装，也未注册 `ros2 run` 入口；应从源码路径执行。数学核对、离线积分结果、已有 Gazebo 录包和真实硬件验收是不同证据，不能互相替代。
