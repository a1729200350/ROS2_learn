import numpy as np
import rclpy
from two_link_arm_kinematics.dynamics_model import DynamicsModel
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
  tau_inertia = M @ q_ddot
  tau = (
      tau_inertia
      + V
      + G
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

