import numpy as np
import math
import rclpy
from dynamics_monitor import DynamicsMonitor
def test(q):
  M = node.calculate_mass_matrix(np.array(q))
  print("\n====================")
  print("q =", q)
  print("M(q)=")
  print(M)
  symmetry_error = np.linalg.norm(M - M.T)
  eigenvalues = np.linalg.eigvalsh(M)
  print("对称误差 =",symmetry_error)
  print("特征值 =",eigenvalues)

if __name__ == "__main__":

    # 初始化 ROS2
    rclpy.init()
    node = DynamicsMonitor()
    test(
        [
            0,
            math.pi/2,
            0
        ]
    )

    test(
        [
            0.5,
            -0.8,
            1.2
        ]
    )
    # 销毁唯一的节点
    node.destroy_node()
    # 关闭 ROS2
    rclpy.shutdown()