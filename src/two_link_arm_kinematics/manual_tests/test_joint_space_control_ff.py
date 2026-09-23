import numpy as np
# import rclpy
# from two_link_arm_kinematics.dynamics_monitor import DynamicsMonitor
from two_link_arm_kinematics.dynamics_model import DynamicsModel
from two_link_arm_kinematics.joint_space_controller import (JointSpaceController)
if __name__ == '__main__':
  # #DynamicsMonitor 是 ROS2 Node，所以先初始化 ROS2
  # rclpy.init()
  # dynamics = DynamicsMonitor()
  model = DynamicsModel()
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
  # tau_ff = dynamics.calculate_inverse_dynamics(
  #   q_d,
  #   q_dot_d,
  #   q_ddot_d
  # )
  tau_ff = model.calculate_inverse_dynamics(
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

  # 位置误差、速度误差和 PD 力矩可直接手算
  np.testing.assert_allclose(
      e, [0.1, -0.05, 0.1],
      rtol=0.0, atol=1e-12
  )

  np.testing.assert_allclose(
      e_dot, [0.4, -0.15, 0.2],
      rtol=0.0, atol=1e-12
  )

  np.testing.assert_allclose(
      tau_pd, [1.8, -0.8, 1.4],
      rtol=0.0, atol=1e-12
  )

  # 当前参数和轨迹状态对应的前馈基准
  np.testing.assert_allclose(
      tau_ff,
      [-7.07536673, -2.13570402, -0.94535415],
      rtol=1e-6,
      atol=1e-7
  )

  np.testing.assert_allclose(
      tau_ff_out, tau_ff,
      rtol=0.0, atol=1e-12
  )

  # 非零误差时的总力矩基准
  np.testing.assert_allclose(
      tau,
      [-5.27536673, -2.93570402, 0.45464585],
      rtol=1e-6,
      atol=1e-7
  )

  # 当位置、速度都与期望一致时，PD 力矩应为零
  zero_error_result = controller.calculate_control_torque(
      q_d,
      q_dot_d,
      q_d,
      q_dot_d,
      tau_ff
  )

  np.testing.assert_allclose(
      zero_error_result[2],
      np.zeros(3),
      rtol=0.0,
      atol=1e-12
  )

  # 此时总力矩等于前馈力矩
  np.testing.assert_allclose(
      zero_error_result[0],
      tau_ff,
      rtol=0.0,
      atol=1e-12
  )


  print('e =', e)
  print('e_dot =', e_dot)
  print('tau_ff =', tau_ff_out)
  print('tau_pd =', tau_pd)
  print('tau =', tau)
  # dynamics.destroy_node()
  # rclpy.shutdown()
  print("\n前馈 + PD 检查通过。")