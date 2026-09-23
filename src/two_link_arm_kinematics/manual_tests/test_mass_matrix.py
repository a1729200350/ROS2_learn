import numpy as np
# import math
# import rclpy
# from two_link_arm_kinematics.dynamics_monitor import DynamicsMonitor
from two_link_arm_kinematics.dynamics_model import DynamicsModel
# def test(q):
#   M = node.calculate_mass_matrix(np.array(q))
#   print("\n====================")
#   print("q =", q)
#   print("M(q)=")
#   print(M)
#   symmetry_error = np.linalg.norm(M - M.T)
#   eigenvalues = np.linalg.eigvalsh(M)
#   print("对称误差 =",symmetry_error)
#   print("特征值 =",eigenvalues)

def check_mass_matrix(model, q):
    q = np.asarray(q, dtype=float)
    M = model.calculate_mass_matrix(q)
    # 检查矩阵形状和数值有效性
    assert M.shape == (3, 3), "质量矩阵必须是 3x3"
    assert np.all(np.isfinite(M)), "质量矩阵包含非有限值"
    # 质量矩阵应该对称
    np.testing.assert_allclose(
        M,
        M.T,
        rtol=0.0,
        atol=1e-12
    )
    # 对称质量矩阵应该正定
    eigenvalues = np.linalg.eigvalsh(M)
    assert np.all(eigenvalues > 0.0), (
        f"质量矩阵不是正定矩阵，特征值为 {eigenvalues}"
    )
    print("\n====================")
    print("q =", q)
    print("M(q) =\n", M)
    print("特征值 =", eigenvalues)


# if __name__ == "__main__":


#     # 初始化 ROS2
#     rclpy.init()
#     node = DynamicsMonitor()
#     test(
#         [
#             0,
#             math.pi/2,
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
    # 零位、弯曲姿态、一般姿态
    for q in [
        [0.0, 0.0, 0.0],
        [0.0, np.pi / 2, 0.0],
        [0.5, -0.8, 1.2]
    ]:
        check_mass_matrix(model, q)
    # 当前参数下，零位质量矩阵的已知基准
    expected_M0 = np.array([
        [1.46520, 0.66166, 0.18683],
        [0.66166, 0.34166, 0.10683],
        [0.18683, 0.10683, 0.04283]
    ])
    np.testing.assert_allclose(
        model.calculate_mass_matrix([0.0, 0.0, 0.0]),
        expected_M0,
        rtol=1e-10,
        atol=1e-12
    )
    # 检查惯量参数是否真正参与计算
    # 连杆 3 的角速度为 dq1 + dq2 + dq3，
    # 所以 I3 增加 delta 后，M 的每个元素都增加 delta。
    changed_model = DynamicsModel()
    delta = 0.001
    changed_model.I3 += delta
    q = np.array([0.5, -0.8, 1.2])
    np.testing.assert_allclose(
        changed_model.calculate_mass_matrix(q)
        - model.calculate_mass_matrix(q),
        delta * np.ones((3, 3)),
        rtol=1e-9,
        atol=1e-12
    )
    print("\n质量矩阵检查通过。")
