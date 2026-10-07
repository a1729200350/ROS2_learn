import numpy as np
from three_link_arm_kinematics.operational_space_kinematics import (OperationalSpaceKinematics,)
def main():

    model = OperationalSpaceKinematics()
    np.set_printoptions(precision=10,suppress=True,)

    # 测试状态
    q = np.array([0.5, -0.8, 1.2,])
    q_dot = np.array([ 1.0, 0.5, -0.3,])
    q_ddot = np.array([ 0.2, -0.1, 0.3,])

    # 1. 验证 J_dot
    J_dot_analytic = model.jacobian_dot( q, q_dot,)
    h_jacobian = 1e-5
    J_plus = model.jacobian( q + h_jacobian * q_dot)
    J_minus = model.jacobian( q - h_jacobian * q_dot)
    J_dot_numeric = ( J_plus - J_minus ) / ( 2.0 * h_jacobian )
    J_dot_error = np.linalg.norm( J_dot_analytic - J_dot_numeric)

    print("\n" "1. J_dot 验证\n")
    print("\nJ_dot analytic =\n",J_dot_analytic,)
    print( "\nJ_dot numeric =\n", J_dot_numeric,)
    print("\n||J_dot analytic - J_dot numeric|| =",f"{J_dot_error:.12e}",)

    # 2. 解析计算末端加速度
    J = model.jacobian(q)
    x_ddot_from_q_ddot = ( J @ q_ddot)
    x_ddot_from_jacobian = ( J_dot_analytic @ q_dot )
    x_ddot_analytic = ( x_ddot_from_q_ddot + x_ddot_from_jacobian )

    print("\n""2. 末端加速度分解\n")
    print("\nJ @ q_ddot =",x_ddot_from_q_ddot,)
    print("\nJ_dot @ q_dot =",x_ddot_from_jacobian,)

    print("\nx_ddot analytic =", x_ddot_analytic,)

    # 3. 用 FK 的二阶数值导数独立验证
    # q(t+h) ≈ q + q_dot*h + 1/2*q_ddot*h^2
    # q(t-h) ≈ q - q_dot*h + 1/2*q_ddot*h^2
    # x_ddot ≈ [x(t+h)-2x(t)+x(t-h)] / h^2
    h_acceleration = 1e-4
    q_plus = ( q + q_dot * h_acceleration + 0.5 * q_ddot * h_acceleration ** 2 )
    q_minus = ( q - q_dot * h_acceleration + 0.5 * q_ddot * h_acceleration ** 2)
    x_plus = model.forward_kinematics( q_plus)
    x_zero = model.forward_kinematics( q)
    x_minus = model.forward_kinematics( q_minus)
    x_ddot_numeric = ( x_plus - 2.0 * x_zero + x_minus ) / ( h_acceleration ** 2 )
    x_ddot_error = np.linalg.norm( x_ddot_analytic - x_ddot_numeric )

    print("\n""3. x_ddot 独立验证\n")
    print("\nx_ddot analytic =",x_ddot_analytic,)
    print("\nx_ddot numeric =",x_ddot_numeric,)
    print("\n||analytic - numeric|| =",f"{x_ddot_error:.12e}",)

    # 4. 特殊实验：
    # q_ddot = 0
    # 验证：
    # 即使关节角加速度为 0，
    # J_dot @ q_dot 仍可能不为 0
    q_ddot_zero = np.zeros(3)
    x_ddot_zero_joint_acc = (model.task_acceleration(q,q_dot, q_ddot_zero,))
    print("\n" "4. q_ddot = 0 实验\n")
    print("\nq_ddot =", q_ddot_zero,)
    print("\nx_ddot =", x_ddot_zero_joint_acc,)
    print("\nJ_dot @ q_dot =", J_dot_analytic @ q_dot,)
if __name__ == '__main__':
    main()