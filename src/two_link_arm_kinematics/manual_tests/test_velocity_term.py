import numpy as np
import math
import rclpy
from dynamics_monitor import DynamicsMonitor
def test(node,q,q_dot):
  q = np.array(q, dtype=float)
  q_dot = np.array(q_dot, dtype=float)
  V = node.calculate_velocity_term(q,q_dot)
  print("\n====================")
  print("q =", q)
  print("q_dot =", q_dot)
  print("V(q)=",V)

if __name__ == "__main__":

    # 初始化 ROS2
    rclpy.init()
    node = DynamicsMonitor()
    # 测试1：
    # 速度为0时，V必须为0
    test(
        node,
        [0.5, -0.8, 1.2],
        [0.0, 0.0, 0.0]
    )

    # 测试2：
    # 零位 + 非零速度
    test(
        node,
        [0.0, 0.0, 0.0],
        [1.0, 0.5, -0.3]
    )
    # 测试3：
    # 一般姿态 + 一般速度
    test(
        node,
        [0.5, -0.8, 1.2],
        [1.0, 0.5, -0.3]
    )
    # 测试4：
    # 速度整体放大 2 倍
    test(
    node,
    [0.5, -0.8, 1.2],
    [2.0, 1.0, -0.6]
)
    # 销毁唯一的节点
    node.destroy_node()
    # 关闭 ROS2
    rclpy.shutdown()