import numpy as np
# import math
# import rclpy
# from two_link_arm_kinematics.dynamics_monitor import DynamicsMonitor
from two_link_arm_kinematics.dynamics_model import DynamicsModel
# def test(node,q,q_dot):
#   q = np.array(q, dtype=float)
#   q_dot = np.array(q_dot, dtype=float)
#   V = node.calculate_velocity_term(q,q_dot)
#   print("\n====================")
#   print("q =", q)
#   print("q_dot =", q_dot)
#   print("V(q)=",V)

# if __name__ == "__main__":

#     # 初始化 ROS2
#     rclpy.init()
#     node = DynamicsMonitor()
#     # 测试1：
#     # 速度为0时，V必须为0
#     test(
#         node,
#         [0.5, -0.8, 1.2],
#         [0.0, 0.0, 0.0]
#     )

#     # 测试2：
#     # 零位 + 非零速度
#     test(
#         node,
#         [0.0, 0.0, 0.0],
#         [1.0, 0.5, -0.3]
#     )
#     # 测试3：
#     # 一般姿态 + 一般速度
#     test(
#         node,
#         [0.5, -0.8, 1.2],
#         [1.0, 0.5, -0.3]
#     )
#     # 测试4：
#     # 速度整体放大 2 倍
#     test(
#     node,
#     [0.5, -0.8, 1.2],
#     [2.0, 1.0, -0.6]
# )
#     # 销毁唯一的节点
#     node.destroy_node()
#     # 关闭 ROS2
#     rclpy.shutdown()

def check_velocity(model, q, q_dot, expected):
    q = np.asarray(q, dtype=float)
    q_dot = np.asarray(q_dot, dtype=float)

    V = model.calculate_velocity_term(q, q_dot)

    assert V.shape == (3,), "速度项必须包含三个关节力矩"
    assert np.all(np.isfinite(V)), "速度项包含非有限值"

    # 与给定基准比较
    np.testing.assert_allclose(
        V,
        expected,
        rtol=1e-6,
        atol=1e-7
    )

    # 沿当前速度方向扰动关节位置，
    # 近似计算质量矩阵的时间导数 M_dot。
    h = 1e-6
    M_dot = (
        model.calculate_mass_matrix(q + h * q_dot)
        - model.calculate_mass_matrix(q - h * q_dot)
    ) / (2.0 * h)

    # 能量一致性：
    # q_dot.T @ V = 0.5 * q_dot.T @ M_dot @ q_dot
    velocity_power = q_dot @ V
    inertia_power = 0.5 * q_dot @ M_dot @ q_dot

    np.testing.assert_allclose(
        velocity_power,
        inertia_power,
        rtol=1e-6,
        atol=1e-6
    )

    print("\n====================")
    print("q =", q)
    print("q_dot =", q_dot)
    print("V =", V)
    print("能量关系残差 =", velocity_power - inertia_power)


if __name__ == "__main__":
    model = DynamicsModel()

    # 1. 速度为零，速度相关力矩应为零
    check_velocity(
        model,
        [0.5, -0.8, 1.2],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0]
    )

    # 2. 当前模型在零位的质量矩阵一阶导数为零
    check_velocity(
        model,
        [0.0, 0.0, 0.0],
        [1.0, 0.5, -0.3],
        [0.0, 0.0, 0.0]
    )

    # 3. 第三关节单独运动，也会影响前面的关节
    check_velocity(
        model,
        [0.0, 0.0, np.pi / 2],
        [0.0, 0.0, 1.0],
        [-0.144, -0.064, 0.0]
    )

    # 4. 一般姿态、一般速度
    q = np.array([0.5, -0.8, 1.2])
    q_dot = np.array([1.0, 0.5, -0.3])

    check_velocity(
        model,
        q,
        q_dot,
        [0.24981621, -0.09269509, 0.16536710]
    )

    # 5. V 对速度是二次的：速度加倍，V 变成四倍
    np.testing.assert_allclose(
        model.calculate_velocity_term(q, 2.0 * q_dot),
        4.0 * model.calculate_velocity_term(q, q_dot),
        rtol=1e-6,
        atol=1e-7
    )

    # 6. 非法差分步长应被拒绝
    for bad_epsilon in [0.0, -1e-6, np.nan, np.inf]:
        with np.testing.assert_raises(ValueError):
            model.calculate_velocity_term(
                q, q_dot, epsilon=bad_epsilon
            )

    # 7. 错误维度和非有限输入应被拒绝
    with np.testing.assert_raises(ValueError):
        model.calculate_velocity_term([0.0, 0.0], q_dot)

    with np.testing.assert_raises(ValueError):
        model.calculate_velocity_term(q, [0.0, np.nan, 0.0])

    with np.testing.assert_raises(ValueError):
        model.calculate_inverse_dynamics(
            q, q_dot, [0.0, 0.0, np.inf]
        )

    print("\n速度项与输入检查通过。")
