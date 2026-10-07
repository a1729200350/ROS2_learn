import numpy as np
from three_link_arm_kinematics.operational_space_controller import (OperationalSpaceController,)
from three_link_arm_kinematics.dynamics_model import (DynamicsModel,)
def main():
  np.set_printoptions(precision=10,suppress=True,)

  # 模型
  controller = OperationalSpaceController(kp=(10.0, 10.0), kd=(4.43, 4.43),)
  dynamics = DynamicsModel()

  # 当前关节状态
  # 与上一实验保持完全一致
  q = np.array([0.5, -0.8, 1.2,])
  q_dot = np.array([ 1.0, 0.5, -0.3,])
  # 当前末端状态
  x = (controller.kinematics.forward_kinematics(q))
  J = ( controller.kinematics.jacobian(q))
  x_dot = ( J @ q_dot)
  # 期望任务空间状态
  # 与上一实验保持一致
  x_d = (x + np.array([ 0.02, -0.01,]))
  x_dot_d = ( x_dot.copy())
  x_ddot_d = np.array([ 0.10,-0.05,])
  # 1. 操作空间控制器
  # x_d
  #   ↓
  # x_ddot_c
  #   ↓
  # q_ddot_c
  control_result = ( controller.calculate(q,q_dot,x_d, x_dot_d, x_ddot_d,) )
  x_ddot_command = (control_result[ 'x_ddot_command'])
  q_ddot_command = (control_result[ 'q_ddot_command' ])
  print("\n""1. 操作空间控制器输出\n")
  print( "\nx_ddot_command =", x_ddot_command,)
  print( "\nq_ddot_command =",q_ddot_command,)
  # 2. 逆动力学
  # tau =  M(q) q_ddot_c  + V(q, q_dot)  + G(q)
  M = (dynamics .calculate_mass_matrix(q))
  V = (dynamics .calculate_velocity_term( q,q_dot,))
  G = (dynamics .calculate_gravity(q))
  tau_inertia = ( M @ q_ddot_command)
  tau = ( tau_inertia + V + G )
  print( "\n""2. 逆动力学\n")
  print("\nM @ q_ddot_command =", tau_inertia,)
  print("\nV =", V,)
  print( "\nG =", G,)
  print("\ntau =", tau,)
  # 3. 从动力学方程反算 q_ddot
  # M q_ddot + V + G = tau
  # 所以： q_ddot =   M^-1 (tau - V - G)
  # 数值实现不用显式求 M^-1，
  # 使用 np.linalg.solve()
  q_ddot_recovered = ( np.linalg.solve(M, tau - V - G,) )
  q_ddot_error = (q_ddot_recovered  - q_ddot_command)

  print("\n""3. 关节加速度恢复验证\n")
  print( "\nq_ddot command =", q_ddot_command,)
  print( "\nq_ddot recovered =", q_ddot_recovered,)
  print( "\nq_ddot error =", q_ddot_error,)
  print("\n||q_ddot error|| =", f"{np.linalg.norm(q_ddot_error):.12e}",)

  # 4. 再恢复末端加速度
  # x_ddot = J q_ddot + J_dot q_dot

  J_dot = (controller.kinematics.jacobian_dot( q, q_dot,))
  x_ddot_recovered = ( J @ q_ddot_recovered + J_dot @ q_dot)
  x_ddot_error = ( x_ddot_recovered - x_ddot_command)
  print( "\n""4. 末端加速度恢复验证\n")
  print( "\nx_ddot command =", x_ddot_command,)
  print("\nx_ddot recovered =",  x_ddot_recovered,)
  print("\nx_ddot error =", x_ddot_error,)
  print("\n||x_ddot error|| =",f"{np.linalg.norm(x_ddot_error):.12e}",)

  # 5. 动力学方程残差
  # residual =   M q_ddot + V + G - tau
  dynamics_residual = (M @ q_ddot_recovered + V + G - tau)

  print( "\n""5. 动力学方程残差\n")
  print( "\nresidual =",dynamics_residual,)
  print( "\n||residual|| =", f"{np.linalg.norm(dynamics_residual):.12e}",)


if __name__ == '__main__':
  main()