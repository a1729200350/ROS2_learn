import numpy as np
from three_link_arm_kinematics.operational_space_dynamics import ( OperationalSpaceDynamics,)
def main():
  np.set_printoptions(precision=10,suppress=True,)
  model = OperationalSpaceDynamics()
  q = np.array([ 0.5, -0.8,1.2,])
  q_dot = np.array([ 1.0,0.5, -0.3,])
  x_ddot_command = np.array([0.30, -0.15,])
  result = ( model.calculate_task_force( q, q_dot, x_ddot_command,))

  # 1. mu 和 p  
  print("\n""1. 操作空间偏差项\n")
  print("\nmu 源于动力学 =",result['mu_from_dynamics'],)
  print("\nmu 源于几何学 =",result['mu_from_geometry'],)
  print("\nmu =", result['mu'],)
  print("\np =",result['p'],)

  # 2. 任务空间力
  print("\n""2. 操作空间力\n")
  print("\nLambda @ x_ddot_command =",result['force_inertia'],)
  print("\nF =",result['F'],)
  print("\nτ = J.T @ F =",result['tau'],)

  # 3. 用关节动力学反算 q_ddot
  # M q_ddot + V + G = tau
  q_ddot = np.linalg.solve(result['M'],(result['tau'] - result['V'] - result['G']),)

  # 4. 再恢复实际任务空间加速度
  x_ddot_recovered = ( result['J'] @ q_ddot + result['J_dot'] @ q_dot)
  error = ( x_ddot_recovered - x_ddot_command )
  print( "\n" "3. 全动态验证\n")
  print( "\nq_ddot recovered =", q_ddot,)
  print("\nx_ddot command =",x_ddot_command,)
  print( "\nx_ddot recovered =", x_ddot_recovered,)
  print( "\nerror =", error,)
  print( "\n||error|| =",f"{np.linalg.norm(error):.12e}",)

if __name__ == '__main__':
  main()