import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from two_link_arm_kinematics.joint_space_controller import (JointSpaceController)
class JointSpaceControlMonitor(Node):
  """ROS2 在线关节空间控制监视器。"""
  def __init__(self):
    super().__init__('joint_space_control_monitor')
    # PD 控制器
    # 先进行学习测试增益，不是实际机器人最终参数
    self.controller = JointSpaceController(
      kp=[10.0, 10.0, 10.0],
      kd=[2.0, 2.0, 2.0]
    )
    # 实际关节状态
    self.latest_q = None
    self.latest_q_dot = None
    # 当前期望轨迹
    self.trajectory_times = None
    self.trajectory_positions = None
    self.trajectory_velocities = None
    self.trajectory_accelerations = None
    # 轨迹开始时刻 [s]
    self.trajectory_start_time = None
    # /joint_states
    self.joint_state_subscription = (
      self.create_subscription(
        JointState,
        '/joint_states',
        self.joint_state_callback,
        10
      )
    )
    # JointTrajectory
    self.trajectory_subscription = (
      self.create_subscription(
        JointTrajectory,
        '/joint_trajectory_controller/joint_trajectory',
        self.trajectory_callback,
        10
      )
    )
    # 100 Hz 控制计算
    self.control_timer = self.create_timer(
      0.01,
      self.control_loop
    )
    # 100 Hz 计算，但只每 0.5 s 打一次日志
    self.log_counter = 0
    self.log_every_n = 50
    self.get_logger().info(
      '关节空间控制监视器启动.'
    )
  # 实际状态
  def joint_state_callback(self, msg):
    joint_names = (
      'joint1',
      'joint2',
      'joint3'
    )

    if not all(
      name in msg.name
      for name in joint_names
    ):
      return

    indices = [
      msg.name.index(name)
      for name in joint_names
    ]

    max_index = max(indices)

    if len(msg.position) <= max_index:
      return

    self.latest_q = np.array([
      msg.position[index]
      for index in indices
    ], dtype=float)

    # 当前 mock hardware 虽然提供 velocity，
    # 但我们已经观察到它运动时仍可能一直为 0。
    if len(msg.velocity) > max_index:
      self.latest_q_dot = np.array([
        msg.velocity[index]
        for index in indices
      ], dtype=float)
    else:
      self.latest_q_dot = np.zeros(3)
  # 接收期望轨迹
  def trajectory_callback(self, msg):
    joint_names = (
      'joint1',
      'joint2',
      'joint3'
    )

    if not all(
      name in msg.joint_names
      for name in joint_names
    ):
      self.get_logger().warning(
        '轨迹缺少 joint1/joint2/joint3'
      )
      return

    if len(msg.points) == 0:
      self.get_logger().warning(
        '收到空轨迹'
      )
      return

    indices = [
      msg.joint_names.index(name)
      for name in joint_names
    ]

    max_index = max(indices)

    times = []
    positions = []
    velocities = []
    accelerations = []

    for point in msg.points:

      if len(point.positions) <= max_index:
        continue

      if len(point.velocities) <= max_index:
        continue

      if len(point.accelerations) <= max_index:
        continue

      t = (
        point.time_from_start.sec
        +
        point.time_from_start.nanosec * 1e-9
      )

      times.append(t)

      positions.append([
        point.positions[index]
        for index in indices
      ])

      velocities.append([
        point.velocities[index]
        for index in indices
      ])

      accelerations.append([
        point.accelerations[index]
        for index in indices
      ])

    if len(times) == 0:
      self.get_logger().warning(
        '轨迹中没有有效点'
      )
      return

    self.trajectory_times = np.array(
      times,
      dtype=float
    )

    self.trajectory_positions = np.array(
      positions,
      dtype=float
    )

    self.trajectory_velocities = np.array(
      velocities,
      dtype=float
    )

    self.trajectory_accelerations = np.array(
      accelerations,
      dtype=float
    )

    # 确定轨迹开始时间
    # trajectory_generator 的 header.stamp
    # 就是在创建消息时设置的 ROS 时间。
    header_time = (
      msg.header.stamp.sec
      +
      msg.header.stamp.nanosec * 1e-9
    )

    if header_time > 0.0:
      self.trajectory_start_time = header_time
    else:
      self.trajectory_start_time = (
        self.get_clock().now().nanoseconds
        * 1e-9
      )

    self.get_logger().info(
      '\n'
      '========== 收到控制轨迹 ==========\n'
      f'轨迹点数量 = {len(times)}\n'
      f'持续时间 = {times[-1]:.3f} s'
    )

  # 根据当前时间获得 q_d、q_dot_d、q_ddot_d
  def sample_desired_state(self, t):
    if self.trajectory_times is None:
      return None
    times = self.trajectory_times

    # 轨迹开始之前
    if t <= times[0]:
      return (
        self.trajectory_positions[0].copy(),
        self.trajectory_velocities[0].copy(),
        self.trajectory_accelerations[0].copy()
      )
    # 轨迹结束以后：保持终点
    if t >= times[-1]:
      return (
        self.trajectory_positions[-1].copy(),
        self.trajectory_velocities[-1].copy(),
        self.trajectory_accelerations[-1].copy()
      )

    # 找到：
    # times[i] <= t < times[i+1]
    i = np.searchsorted(
      times,
      t,
      side='right'
    ) - 1

    t0 = times[i]
    t1 = times[i + 1]

    alpha = (
      (t - t0)
      /
      (t1 - t0)
    )
      # q 和 q_dot 线性插值
    q_d = (
      (1.0 - alpha)
      * self.trajectory_positions[i]
      +
      alpha
      * self.trajectory_positions[i + 1]
    )
    q_dot_d = (
      (1.0 - alpha)
      * self.trajectory_velocities[i]
      +
      alpha
      * self.trajectory_velocities[i + 1]
    )
      # 梯形速度轨迹中加速度本身是分段常数，
      # 这里暂时使用左侧轨迹点的加速度，
      # 不对加速度做线性插值。
    q_ddot_d = (
      self.trajectory_accelerations[i].copy()
    )

    return (
      q_d,
      q_dot_d,
      q_ddot_d
    )

  # 100 Hz 在线控制计算
  def control_loop(self):

    # 还没有实际状态
    if self.latest_q is None:
      return

    if self.latest_q_dot is None:
      return

    # 还没有收到轨迹
    if self.trajectory_start_time is None:
      return

    now = (
      self.get_clock().now().nanoseconds
      * 1e-9
    )

    elapsed_time = (
      now
      - self.trajectory_start_time
    )

    desired = self.sample_desired_state(
      elapsed_time
    )

    if desired is None:
      return

    (
      q_d,
      q_dot_d,
      q_ddot_d
    ) = desired

    q = self.latest_q.copy()
    q_dot = self.latest_q_dot.copy()
      # 在线 PD 控制计算
    (
      tau_pd,
      e,
      e_dot
    ) = self.controller.calculate_pd_torque(
      q,
      q_dot,
      q_d,
      q_dot_d
    )
    # 只降低日志频率，不降低控制计算频率
    self.log_counter += 1

    if self.log_counter < self.log_every_n:
      return

    self.log_counter = 0

    self.get_logger().info(
      '\n'
      '========== 在线关节空间 PD ==========\n'
      f't = {elapsed_time:.3f} s\n\n'
      f'q      = {q} rad\n'
      f'q_d    = {q_d} rad\n'
      f'e      = {e} rad\n\n'
      f'q_dot   = {q_dot} rad/s\n'
      f'q_dot_d = {q_dot_d} rad/s\n'
      f'e_dot   = {e_dot} rad/s\n\n'
      f'q_ddot_d = {q_ddot_d} rad/s^2\n'
      f'tau_PD   = {tau_pd} N*m'
    )


def main(args=None):

  rclpy.init(args=args)

  node = JointSpaceControlMonitor()

  rclpy.spin(node)

  node.destroy_node()

  rclpy.shutdown()


if __name__ == '__main__':
    main()