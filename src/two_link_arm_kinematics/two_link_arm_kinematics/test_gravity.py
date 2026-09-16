import numpy as np
import math
import rclpy
from dynamics_monitor import DynamicsMonitor
def test(q):
  G = node.calculate_gravity(np.array(q))
  print("\n====================")
  print("q =", q)
  print("G(q)=")
  print(G)


if __name__ == "__main__":

    # 初始化 ROS2
    rclpy.init()
    node = DynamicsMonitor()
    test(
        [
            0,
            0,
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