import numpy as np
import rclpy
from dynamics_monitor import DynamicsMonitor
if __name__ == "__main__":
  # 初始化 ROS2
  rclpy.init()
  node = DynamicsMonitor()
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
  M = node.calculate_mass_matrix(q)
  V = node.calculate_velocity_term(q,q_dot)
  G = node.calculate_gravity(q)
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

  node.destroy_node()
  rclpy.shutdown()
