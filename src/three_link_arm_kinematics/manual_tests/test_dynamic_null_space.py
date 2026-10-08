import numpy as np
from three_link_arm_kinematics.operational_space_dynamics import (OperationalSpaceDynamics,)
def main():
  np.set_printoptions(precision=10, suppress=True,)
  model = OperationalSpaceDynamics()
  q = np.array([ 0.5, -0.8,1.2,])
  q_dot = np.array([1.0,0.5,-0.3])
  x_ddot_command = np.array([0.30,-0.15,])
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

  # 6. 完整操作空间主任务
  task_result = model.calculate_task_force(q, q_dot,x_ddot_command,)
  tau_task = task_result['tau']
  V = task_result['V']
  G = task_result['G']
  J_dot = task_result['J_dot']
  # 只有主任务时的关节加速度
  q_ddot_task = np.linalg.solve(M,tau_task - V - G,)
  # 完整力矩：
  # tau = J^T F + N^T tau_0
  tau_total = (tau_task+ tau_null)
  # 加入零空间力矩之后的关节加速度
  q_ddot_total = np.linalg.solve( M,tau_total - V - G,)
  # 两种情况下的任务空间加速度
  x_ddot_task = (J @ q_ddot_task+ J_dot @ q_dot)
  x_ddot_total = (J @ q_ddot_total + J_dot @ q_dot)
  # 关节加速度变化
  q_ddot_difference = (q_ddot_total - q_ddot_task)
  # 任务空间加速度变化
  x_ddot_difference = ( x_ddot_total - x_ddot_task)

  print("\n6. 主任务 + 零空间力矩完整实验\n")
  print("\ntau_task =", tau_task)
  print("\ntau_null =", tau_null)
  print("\ntau_total =", tau_total)
  print("\nq_ddot_task =", q_ddot_task)
  print("\nq_ddot_total =", q_ddot_total)
  print("\nq_ddot_total - q_ddot_task =", q_ddot_difference,)
  print("\nx_ddot_task =", x_ddot_task)
  print("\nx_ddot_total =", x_ddot_total)
  print("\nx_ddot_total - x_ddot_task =",x_ddot_difference,)
  print("\n||x_ddot 差值|| =",f"{np.linalg.norm(x_ddot_difference):.12e}",)

  # 7. 零空间姿态控制
  q_posture_desired = np.array([0.0,0.0,0.0,])
  Kp_null = np.diag([1.0,1.0,1.0,])
  Kd_null = np.diag([0.5, 0.5, 0.5,])
  # 姿态 PD 的原始力矩
  tau_0_posture = ( Kp_null @ (q_posture_desired - q)- Kd_null @ q_dot)
  # 投影到动态一致零空间
  tau_posture = (N_T @ tau_0_posture)
  # 完整控制力矩
  tau_total_posture = (tau_task + tau_posture)
  # 对应关节加速度
  q_ddot_total_posture = np.linalg.solve( M,tau_total_posture - V - G,)
  # 恢复任务空间加速度
  x_ddot_total_posture = (J @ q_ddot_total_posture + J_dot @ q_dot)
  print("\n7. 动态一致零空间姿态控制\n")
  print("\nq_posture_desired =", q_posture_desired,)
  print( "\ntau_0_posture =", tau_0_posture,)
  print("\ntau_posture = N_T @ tau_0_posture =",tau_posture,)
  print("\ntau_total_posture =", tau_total_posture,)
  print("\nq_ddot_total_posture =",q_ddot_total_posture,)
  print("\nx_ddot_total_posture =", x_ddot_total_posture,)
  print("\n||x_ddot_total_posture - x_ddot_command|| =",f"{np.linalg.norm( x_ddot_total_posture - x_ddot_command):.12e}",)

if __name__ == '__main__':
  main()