import numpy as np
import rclpy
from two_link_arm_kinematics.dynamics_monitor import DynamicsMonitor
from two_link_arm_kinematics.joint_space_controller import (JointSpaceController)
if __name__ == '__main__':
  # DynamicsMonitor 是 ROS2 Node，所以先初始化 ROS2
  rclpy.init()
  dynamics = DynamicsMonitor()
  controller = JointSpaceController(
    kp=[10.0, 10.0, 10.0],
    kd=[2.0, 2.0, 2.0]
  )
  # 实际状态
  q = np.array([
    0.4,
    -0.2,
    0.3
  ])
  q_dot = np.array([
    0.1,
    -0.1,
    0.2
  ])
  # 期望状态 对应之前轨迹 point 125
  q_d = np.array([
    0.5,
    -0.25,
    0.4
  ])
  q_dot_d = np.array([
    0.5,
    -0.25,
    0.4
  ])
  q_ddot_d = np.array([
    0.0,
    0.0,
    0.0
  ])
  # #  已验证得到的 point 125 inverse dynamics
  # tau_ff = np.array([
  #   -7.07536673,
  #   -2.13570402,
  #   -0.94535415
  # ])
  tau_ff = dynamics.calculate_inverse_dynamics(
    q_d,
    q_dot_d,
    q_ddot_d
  )
  # 前馈 + PD
  (
    tau,
    tau_ff_out,
    tau_pd,
    e,
    e_dot
  ) = controller.calculate_control_torque(
    q,
    q_dot,
    q_d,
    q_dot_d,
    tau_ff
  )

  print('e =', e)
  print('e_dot =', e_dot)
  print('tau_ff =', tau_ff_out)
  print('tau_pd =', tau_pd)
  print('tau =', tau)
  dynamics.destroy_node()
  rclpy.shutdown()