import numpy as np
#import rclpy
from three_link_arm_kinematics.dynamics_model import DynamicsModel
if __name__ == "__main__":
  # # 初始化 ROS2
  # rclpy.init()
  # node = DynamicsMonitor()
  model = DynamicsModel()
  q = np.array([
    0.5,
    -0.8,
    1.2
  ])

  q_dot = np.array([
    1.0,
    0.5,
    -0.3
  ])

  q_ddot = np.array([
    0.2,
    -0.1,
    0.3
  ])
  M = model.calculate_mass_matrix(q)
  V = model.calculate_velocity_term(q,q_dot)
  G = model.calculate_gravity(q)
  # tau_inertia = M @ q_ddot
  # tau = (
  #     tau_inertia
  #     + V
  #     + G
  # )
  # 保留惯性力矩，便于分项观察
  tau_inertia = M @ q_ddot
  # 实际调用模型的逆动力学接口
  tau = model.calculate_inverse_dynamics(
    q,
    q_dot,
    q_ddot
  )

  #第一次运行：
  # tau = model.calculate_inverse_dynamics(...)
  # print(tau)
  # 得到：
  # [-4.28957866 0.16332989 -1.02995476]
  # 然后复制进去：
  # expected_tau=np.array([...])

  # 当前模型参数和上述输入对应的回归基准
  expected_tau = np.array([
    -4.28957866,
    0.16332989,
    -1.02995476
  ])
  np.testing.assert_allclose(
    tau,
    expected_tau,
    rtol=1e-6,
    atol=1e-7
  )
  
  print("\n====================")
  print("q =", q)
  print("q_dot =", q_dot)
  print("q_ddot =", q_ddot)
  print("\nM(q)=")
  print(M)
  print("\nM(q) @ q_ddot =")
  print(tau_inertia)
  print("\nV(q, q_dot) =")
  print(V)
  print("\nG(q) =")
  print(G)
  print("\ntau =")
  print(tau)
  print("\n逆动力学基准检查通过。")

