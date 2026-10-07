import numpy as np
from three_link_arm_kinematics.operational_space_dynamics import (OperationalSpaceDynamics,)
def main():
  np.set_printoptions(precision=10, suppress=True,)
  model = OperationalSpaceDynamics()
  q = np.array([ 0.5, -0.8,1.2,])
  result = (model.calculate_null_space_projector( q))
  J = result['J']
  M = result['M']
  J_bar = result['J_bar']
  N = result['N']
  N_T = result['N_T']

  # 1. 查看 N
  print("1. 动态零空间投影器\n")
  print("\nN =\n",N,)
  print("\nN_T =\n", N_T,)

  # 2. 幂等性
  # N^2 = N
  idempotence_error = (np.linalg.norm( N @ N - N))

  print("2. 幂等性检查\n")
  print( "\n||N^2 - N|| =",f"{idempotence_error:.12e}",)

  # 3. J N = 0
  JN = (J @ N)

  print("3. 构型空间零值检查\n")
  print("\nJ @ N =\n",JN,)
  print("\n||J N|| =",f"{np.linalg.norm(JN):.12e}",)

  # 4. 最关键：
  # J M^-1 N^T = 0
  M_inv_NT = np.linalg.solve(M,N_T,)
  dynamic_null_check = (J @ M_inv_NT)

  print("4. 动态一致性检查\n")
  print( "\nJ M^-1 N_T =\n", dynamic_null_check,)
  print("\n||J M^-1 N_T|| =",f"{np.linalg.norm(dynamic_null_check):.12e}",)

  # 5. 给一个任意关节力矩 tau_0
  tau_0 = np.array([2.0, -1.0,  0.5,])
  tau_null = (N_T @ tau_0)
  q_ddot_null = np.linalg.solve( M, tau_null, )
  x_ddot_null = (J @ q_ddot_null)

  print("5. 零空间力矩实验\n" )
  print("\ntau_0 =", tau_0,)
  print("\ntau_null =", tau_null,)
  print("\nq_ddot_null =",q_ddot_null,)
  print("\n由 tau_null 引起的 x_ddo =",x_ddot_null,)
  print("\n||x_ddot_null|| =",f"{np.linalg.norm(x_ddot_null):.12e}",)

if __name__ == '__main__':
  main()