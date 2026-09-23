import numpy as np
# import math
# import rclpy
# from three_link_arm_kinematics.dynamics_monitor import DynamicsMonitor
from three_link_arm_kinematics.dynamics_model import DynamicsModel
# def test(q):
#   G = node.calculate_gravity(np.array(q))
#   print("\n====================")
#   print("q =", q)
#   print("G(q)=")
#   print(G)

def potential_energy(model, q):
    """
    当前安装方向下的重力势能。

    局部 +X 对应世界向上方向。
    势能中与关节角无关的常数可以省略。
    """
    q1, q2, q3 = q
    # 三根连杆质心沿局部 X 方向的位置
    x1 = model.r1 * np.cos(q1)
    x2 = (
        model.L1 * np.cos(q1)
        + model.r2 * np.cos(q1 + q2)
    )
    x3 = (
        model.L1 * np.cos(q1)
        + model.L2 * np.cos(q1 + q2)
        + model.r3 * np.cos(q1 + q2 + q3)
    )
    return model.g * (
        model.m1 * x1
        + model.m2 * x2
        + model.m3 * x3
    )

def check_gravity(model, q):
    q = np.asarray(q, dtype=float)
    G = model.calculate_gravity(q)
    #断言语句  assert 条件 错误提示 
    assert G.shape == (3,), "重力项必须包含三个关节力矩"
    assert np.all(np.isfinite(G)), "重力项包含非有限值"

    # 独立地对势能求数值梯度
    h = 1e-6
    gradient = np.zeros(3)

    for i in range(3):
        delta_q = np.zeros(3)
        delta_q[i] = h

        gradient[i] = (
            potential_energy(model, q + delta_q)
            - potential_energy(model, q - delta_q)
        ) / (2.0 * h)

    # 动力学中的重力项应该满足 G = ∂U/∂q
    np.testing.assert_allclose(
        G,
        gradient,
        rtol=1e-6,
        atol=1e-6
    )

    print("\n====================")
    print("q =", q)
    print("G(q) =", G)
    print("势能数值梯度 =", gradient)

# if __name__ == "__main__":

#     # 初始化 ROS2
#     rclpy.init()
#     node = DynamicsMonitor()
#     test(
#         [
#             0,
#             0,
#             0
#         ]
#     )
#     test(
#         [
#             0.5,
#             -0.8,
#             1.2
#         ]
#     )
#     # 销毁唯一的节点
#     node.destroy_node()
#     # 关闭 ROS2
#     rclpy.shutdown()

if __name__ == "__main__":
    model = DynamicsModel()

    # 包含竖直、水平、弯曲和一般姿态
    for q in [
        [0.0, 0.0, 0.0],
        [np.pi / 2, 0.0, 0.0],
        [0.0, np.pi / 2, 0.0],
        [0.5, -0.8, 1.2]
    ]:
        check_gravity(model, q)

    # 竖直零位：重力作用线经过关节轴，没有重力力矩
    np.testing.assert_allclose(
        model.calculate_gravity([0.0, 0.0, 0.0]),
        [0.0, 0.0, 0.0],
        rtol=0.0,
        atol=1e-10
    )

    # 水平伸展：重力补偿力矩的已知基准
    np.testing.assert_allclose(
        model.calculate_gravity([np.pi / 2, 0.0, 0.0]),
        [-16.5789, -6.2784, -1.5696],
        rtol=0.0,
        atol=1e-9
    )

    # 检查重力参数是否真正参与计算
    q = np.array([0.5, -0.8, 1.2])
    original_G = model.calculate_gravity(q)
    model.g *= 2.0

    np.testing.assert_allclose(
        model.calculate_gravity(q),
        2.0 * original_G,
        rtol=1e-10,
        atol=1e-12
    )

    print("\n重力项检查通过。")
