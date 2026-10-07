import numpy as np
from three_link_arm_kinematics.operational_space_dynamics import (OperationalSpaceDynamics,)
def main():
  np.set_printoptions(precision=10,suppress=True,)
  model = OperationalSpaceDynamics()
  q = np.array([0.5,-0.8,1.2,])
  result = (model.calculate_dynamic_consistent_inverse( q))
  J = result['J']
  M = result['M']
  Lambda = result['Lambda']
  J_bar = result['J_bar']
  task_inverse_inertia = (result['task_inverse_inertia'])

  # 1. Lambda
  print("\n""1. 操作空间惯量\n")
  print("\nJ M^-1 J^T =\n",task_inverse_inertia,)
  print("\nLambda =\n",Lambda,)

  # 2. 对称性
  symmetry_error = (np.linalg.norm(Lambda - Lambda.T))
  print("\n""2. Lambda 对称性\n")
  print("\n||Lambda - Lambda.T|| =", f"{symmetry_error:.12e}",)

  # 3. 正定性
  eigenvalues = (np.linalg.eigvalsh(Lambda))
  print("\n""3. Lambda 正定性\n")
  print("\neigenvalues =",eigenvalues,)

  # 4. 逆矩阵关系
  identity_check = (Lambda@ task_inverse_inertia)
  identity_error = (np.linalg.norm(identity_check - np.eye(2)))
  print("\n""4. Lambda 逆向检查\n")
  print("\nLambda @ (J M^-1 J^T) =\n",identity_check,)
  print("\nidentity error =",f"{identity_error:.12e}",)

  # 5. 动态一致广义逆
  print("\n""5. 动力学一致逆\n")
  print("\nJ_bar =\n",J_bar,)
  J_Jbar = ( J @ J_bar)
  print("\nJ @ J_bar =\n",J_Jbar,)
  print("\n||J J_bar - I|| =",f"{np.linalg.norm(J_Jbar - np.eye(2)):.12e}",)

  # 6. 与 MP 伪逆比较
  J_pinv = np.linalg.pinv(J)
  print("\n""6. MP 与动态一致逆\n")
  print("\nJ_pinv =\n",J_pinv,)
  print("\nJ_bar =\n",J_bar,)

  # 给一个任务空间速度
  x_dot = np.array([0.10,-0.05,])
  q_dot_mp = (J_pinv @ x_dot)
  q_dot_dynamic = (J_bar @ x_dot)
  print("\nx_dot =",x_dot,)
  print("\nq_dot MP =", q_dot_mp,)
  print("\nq_dot dynamic =",q_dot_dynamic,)
  print("\nJ q_dot MP =",J @ q_dot_mp,)
  print( "\nJ q_dot dynamic =", J @ q_dot_dynamic,)

  # 普通二范数
  norm_mp = np.linalg.norm(q_dot_mp)
  norm_dynamic = np.linalg.norm(q_dot_dynamic)

  # 动能指标
  kinetic_mp = (0.5* q_dot_mp @ M @ q_dot_mp)
  kinetic_dynamic = (0.5* q_dot_dynamic @ M @ q_dot_dynamic)

  print( "\n||q_dot MP|| =", norm_mp,)
  print("\n||q_dot dynamic|| =", norm_dynamic,)
  print("\n0.5 q_dot_MP^T M q_dot_MP =",kinetic_mp,)
  print("\n0.5 q_dot_dynamic^T M q_dot_dynamic =",kinetic_dynamic,)


if __name__ == '__main__':
  main()