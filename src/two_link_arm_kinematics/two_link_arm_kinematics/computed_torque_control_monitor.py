# """
# Computed Torque Controller Monitor

# 功能：

# 订阅：
#     /joint_states
#     /joint_trajectory_controller/joint_trajectory

# 计算：

#     tau_ff = M(qd)qdd_d + V(qd,qdot_d) + G(qd)

#     tau_pd = Kp(qd-q) + Kd(qdot_d-qdot)

#     tau_total = tau_ff + tau_pd

# 目前只计算，不发送 effort command。
# """
"""
e = q_d - q

e_dot = q_dot_d - q_dot

q_ddot_cmd =q_ddot_d+ Kd e_dot + Kp e

tau_total = M(q) q_ddot_cmd + V(q, q_dot) + G(q)

目前只计算，不发送 effort command。
"""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from two_link_arm_kinematics.joint_space_controller import (JointSpaceController)
from two_link_arm_kinematics.dynamics_model import (DynamicsModel)
class ComputedTorqueControlMonitor(Node):
  def __init__(self):
    super().__init__(
      'computed_torque_control_monitor'
    )
    # PD 控制器
    self.controller = JointSpaceController(
      kp=[10.0, 10.0, 10.0],
      kd=[2.0, 2.0, 2.0]
    )
    # 动力学模型
    self.dynamics = DynamicsModel()
    # 当前实际状态
    self.q = None
    self.q_dot = None
    # 期望轨迹
    self.times = None
    self.q_d_traj = None
    self.q_dot_d_traj = None
    self.q_ddot_d_traj = None
    self.start_time = None
    # ROS2 订阅
    self.create_subscription(
      JointState,
      '/joint_states',
      self.joint_state_callback,
      10
    )

    self.create_subscription(
      JointTrajectory,
      '/joint_trajectory_controller/joint_trajectory',
      self.trajectory_callback,
      10
    )
    # 100Hz 控制循环
    self.timer = self.create_timer(
      0.01,
      self.control_loop
    )
    self.log_counter = 0
    self.get_logger().info(
        '计算力矩控制监视器已启动.'
    )

  # 实际状态
  def joint_state_callback(self, msg):

    names = [
      'joint1',
      'joint2',
      'joint3'
    ]


    if not all(
      name in msg.name
      for name in names
    ):
      return


    index = [
      msg.name.index(name)
      for name in names
    ]


    self.q = np.array(
      [
        msg.position[i]
        for i in index
      ],
      dtype=float
    )


    if len(msg.velocity) > max(index):

      self.q_dot = np.array(
        [
          msg.velocity[i]
          for i in index
        ],
        dtype=float
      )

    else:
      self.q_dot = np.zeros(3)

    # 接收轨迹
  
  def trajectory_callback(self, msg):
    names = [
      'joint1',
      'joint2',
      'joint3'
    ]

    if not all(
      name in msg.joint_names
      for name in names
    ):
      return

    index = [
      msg.joint_names.index(name)
      for name in names
    ]

    times = []
    q_list = []
    qdot_list = []
    qddot_list = []

    for p in msg.points:
      t = (
        p.time_from_start.sec
        +
        p.time_from_start.nanosec
        *1e-9
      )
      times.append(t)
      q_list.append(
        [
          p.positions[i]
          for i in index
        ]
      )
      qdot_list.append(
        [
          p.velocities[i]
          for i in index
        ]
      )
      qddot_list.append(
        [
          p.accelerations[i]
          for i in index
        ]
      )

    self.times = np.array(times)
    self.q_d_traj = np.array(q_list)
    self.q_dot_d_traj = np.array(qdot_list)
    self.q_ddot_d_traj = np.array(qddot_list)


    self.start_time = (
      self.get_clock()
      .now()
      .nanoseconds
      *
      1e-9
    )
    self.get_logger().info(
      '\n'
      '========== 收到计算力矩轨迹 ==========\n'
      f'points = {len(times)}'
    )

  # 轨迹插值
  def sample_trajectory(self, t):
    if self.times is None:
      return None
    if t <= self.times[0]:
      return (
        self.q_d_traj[0],
        self.q_dot_d_traj[0],
        self.q_ddot_d_traj[0]
      )
    if t >= self.times[-1]:
      return (
        self.q_d_traj[-1],
        self.q_dot_d_traj[-1],
        self.q_ddot_d_traj[-1]
      )
    i = np.searchsorted(
      self.times,
      t
    ) - 1
    alpha = (
      t-self.times[i]
    ) / (
      self.times[i+1]
      -
      self.times[i]
    )
    q_d = (
      (1-alpha)
      *
      self.q_d_traj[i]
      +
      alpha
      *
      self.q_d_traj[i+1]
    )
    q_dot_d = (
      (1-alpha)
      *
      self.q_dot_d_traj[i]
      +
      alpha
      *
      self.q_dot_d_traj[i+1]
    )

    q_ddot_d = (
      self.q_ddot_d_traj[i]
    )

    return (
      q_d,
      q_dot_d,
      q_ddot_d
    )

  # 控制循环
  def control_loop(self):
    if self.q is None:
      return
    if self.q_dot is None:
      return
    if self.start_time is None:
      return
    now = (
      self.get_clock()
      .now()
      .nanoseconds
      *
      1e-9
    )
    t = now - self.start_time
    desired = self.sample_trajectory(t)
    if desired is None:
      return
    (
      q_d,
      q_dot_d,
      q_ddot_d
    ) = desired
    # # 逆动力学
    # tau_ff = (
    #   self.dynamics
    #   .calculate_inverse_dynamics(
    #     q_d,
    #     q_dot_d,
    #     q_ddot_d
    #   )
    # )
    # # PD
    # (
    #   tau_pd,
    #   e,
    #   e_dot
    # ) = self.controller.calculate_pd_torque(
    #   self.q,
    #   self.q_dot,
    #   q_d,
    #   q_dot_d
    # )
    # # 总力矩
    # tau_total = (
    #   tau_ff
    #   +
    #   tau_pd
    # )

    # 经典 Computed Torque Control
    tau_total,e,e_dot,q_ddot_cmd = self.controller.calculate_classical_computed_torque(
      self.dynamics,
      self.q,
      self.q_dot,
      q_d,
      q_dot_d,
      q_ddot_d
    )

    self.log_counter += 1

    if self.log_counter < 50:
      return

    self.log_counter = 0

    self.get_logger().info(

      # '\n'
      # '========== Computed Torque ==========\n'
      # f't = {t:.3f}s\n\n'
      # f'q = {self.q}\n'
      # f'q_d = {q_d}\n\n'
      # f'e = {e}\n\n'
      # f'tau_ff = {tau_ff}\n'
      # f'tau_pd = {tau_pd}\n\n'
      # f'tau_total = {tau_total} N*m'

      '\n'
      '========== Classical Computed Torque ==========\n'
      f't = {t:.3f}s\n\n'
      f'q = {self.q}\n'
      f'q_d = {q_d}\n\n'
      f'e = {e}\n'
      f'e_dot = {e_dot}\n\n'
      f'q_ddot_d = {q_ddot_d}\n'
      f'q_ddot_cmd = {q_ddot_cmd}\n\n'
      f'tau_total = {tau_total} N*m'
    )



def main(args=None):

  rclpy.init(args=args)

  node = ComputedTorqueControlMonitor()

  rclpy.spin(node)


  node.destroy_node()

  rclpy.shutdown()



if __name__ == '__main__':

  main()