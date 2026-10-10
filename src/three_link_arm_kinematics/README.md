# three_link_arm_kinematics

平面三连杆机械臂的运动学、动力学、轨迹与控制实验功能包。完整环境、构建和分终端运行流程见[工作空间说明](../../README.md)。

## ROS 2 节点入口

以下入口由 [`setup.py`](setup.py) 注册，构建并加载环境后使用 `ros2 run three_link_arm_kinematics <入口>`。表格是索引，不表示应同时启动所有节点。

| 入口 | 用途与边界 |
| --- | --- |
| `kinematics_monitor` | 接收关节状态与期望笛卡尔速度，执行带约束的 QP；由速度消息触发求解 |
| `trajectory_generator` | 启动后发布一次关节轨迹；起点是源码中的示例值，不自动读取反馈 |
| `trajectory_monitor` | 对齐期望轨迹与关节反馈，打印位置误差 |
| `dynamics_monitor` | 动力学量观察 |
| `joint_space_control_monitor` | 关节空间控制计算与观察 |
| `computed_torque_control_monitor` | 发布关节力矩与 13 字段 `/computed_torque_tracking` |
| `joint_dynamics_simulator` | 独立软件动力学积分与关节状态发布；不是 Gazebo |
| `trajectory_tracking_analyzer` | 将 13 字段跟踪消息写入启动目录的 `tracking_data.csv`；会覆盖同名文件 |
| `operational_space_control_monitor` | 二维末端任务与零空间姿态控制、力矩限幅和 16 字段 OSC 跟踪消息 |
| `osc_auto_activator` | 等待 `/osc_ready` 后激活 Gazebo 力矩控制器；不调用 `/osc_start` |

节点源文件位于 [`three_link_arm_kinematics/`](three_link_arm_kinematics/)。位置型 mock 控制器不会消费 `/joint_effort_command`；独立软件仿真和 Gazebo 是两条不同的力矩闭环路径，不能混用状态源。

## 计算模块与实验

| 文件或目录 | 内容 |
| --- | --- |
| [`dynamics_model.py`](three_link_arm_kinematics/dynamics_model.py) | 关节空间动力学模型 |
| [`operational_space_kinematics.py`](three_link_arm_kinematics/operational_space_kinematics.py) | 末端位置、雅可比与雅可比导数 |
| [`operational_space_controller.py`](three_link_arm_kinematics/operational_space_controller.py) | 任务空间前馈加 PD 加速度控制 |
| [`operational_space_dynamics.py`](three_link_arm_kinematics/operational_space_dynamics.py) | 操作空间惯量、偏差项、动态一致广义逆与零空间投影 |
| [`task_space_trajectory.py`](three_link_arm_kinematics/task_space_trajectory.py) | 五次时间缩放的任务轨迹采样，结束后保持终点 |
| [`manual_tests/`](manual_tests/) | 数值核对、闭环积分、模型误差、零空间和奇异点实验 |
| [`test/`](test/) | 包的版权、代码风格与文档字符串检查入口，不是闭环验收测试 |

OSC 任务是平面末端 `[x, y]`，不控制末端姿态。直接逆解要求任务惯量矩阵可逆，完全伸直的 `[0, 0, 0]` 构型对该二维任务奇异；应区分离线数值实验和 ROS/Gazebo 的实际运行结果。

## OSC 接口

| 接口 | 类型 | 含义 |
| --- | --- | --- |
| `/joint_states` | `sensor_msgs/msg/JointState` | 关节位置与速度输入，按关节名称读取 |
| `/joint_effort_command` | `std_msgs/msg/Float64MultiArray` | 三关节限幅后的力矩指令；Gazebo launch 重映射到 `/effort_controller/commands` |
| `/osc_tracking` | `std_msgs/msg/Float64MultiArray` | 同次控制计算得到的 16 个跟踪数值 |
| `/osc_ready` | `std_srvs/srv/Trigger` | 自动激活前检查；不启动轨迹 |
| `/osc_start` | `std_srvs/srv/Trigger` | 从保持状态启动一次轨迹；同次运行不支持重新触发 |

`/osc_tracking` 的零基索引如下，切片区间右端不包含在内：

| 索引/切片 | 数据 | 单位 |
| --- | --- | --- |
| `0` | 轨迹时间 `t`，等待启动时为零 | s |
| `1:3` | 期望末端位置 `x_d` | m |
| `3:5` | 实际末端位置 `x` | m |
| `5:7` | 末端误差 `x_d - x` | m |
| `7:10` | 关节位置 `q` | rad |
| `10:13` | 关节速度 `q_dot` | rad/s |
| `13:16` | 限幅后发布的力矩指令 `tau` | N·m |

最后三个字段不是原始需求力矩或传感器测量力矩。现有 CSV 记录器只接收 `/computed_torque_tracking`，不能直接用来记录 OSC 的 16 字段消息；OSC 录包与曲线见[实验记录](../../osc_records/README.md)。

## 参数、就绪与故障

启动参数与当前默认值见[配置说明](../three_link_arm_description/README.md#osc-启动参数)。从工作空间根目录单独运行时可显式加载配置：

```bash
ros2 run three_link_arm_kinematics operational_space_control_monitor --ros-args \
  --params-file src/three_link_arm_description/config/osc_params.yaml
```

该命令只启动控制节点，仍需要合适的状态源与力矩接收端。首次有效反馈后先保持；`/osc_start` 成功才开始移动。

`/osc_ready` 检查无故障、反馈与正常力矩新鲜度、尚未启动轨迹、低关节速度，并要求原始需求力矩不超过逐关节限幅。限幅后的输出正常不等于原始需求满足就绪门槛；`/osc_start` 不复用完整的就绪检查。

控制器先做有限值、奇异性、反馈超时和 `100 N·m` 原始力矩异常值检查，再执行可配置的力矩裁剪。故障后锁定并持续发布零力矩；零力矩不是姿态保持，也不会让仍持续收到命令的独立动力学仿真器触发超时暂停。Gazebo 的独立[状态监视脚本](../../scripts/README.md)是另一层暂停机制。
