import numpy as np
from three_link_arm_kinematics.operational_space_controller import ( OperationalSpaceController,)
def main():
  np.set_printoptions( precision=10, suppress=True,)
  controller = OperationalSpaceController( kp=(10.0, 10.0), kd=(4.43, 4.43),)

  # 当前机器人状态
  q = np.array([0.5,-0.8, 1.2,])
  q_dot = np.array([1.0, 0.5,-0.3,])

  # 当前末端状态
  x = (controller.kinematics.forward_kinematics(q))
  J = ( controller.kinematics.jacobian(q))
  x_dot = (J @ q_dot)

  # 构造一个简单的期望状态
  # 当前位置基础上：
  # x 方向 +0.02 m
  # y 方向 -0.01 m
  # 为了这次测试更清楚：
  # 令期望速度 = 当前速度
  # 因此速度误差为 0
  x_d = (x + np.array([ 0.02, -0.01,]))
  x_dot_d = ( x_dot.copy() )
  x_ddot_d = np.array([ 0.10, -0.05,])

  # 计算控制命令
  result = controller.calculate(q, q_dot, x_d, x_dot_d, x_ddot_d,)
  print("\n""1. 当前任务空间状态\n")
  print("\nx =",result['x'],)
  print( "\nx_dot =", result['x_dot'],)
  print( "\nx_d =", x_d,)
  print("\ne_x =",result['e_x'],)
  print( "\ne_x_dot =",result['e_x_dot'],)
  print("\nx_ddot_command =", result['x_ddot_command'], )
  print( "\n" "2. 加速度映射\n")
  print( "\nJ_dot @ q_dot =", result['geometric_acceleration'],)
  print("\nx_ddot_command - J_dot @ q_dot =", result[ 'corrected_task_acceleration'],)
  print("\nq_ddot_command =",result['q_ddot_command' ],)
  # 3. 最重要的验证
  # x_ddot_actual
  #     = J q_ddot_c + J_dot q_dot
  # 理论上应当等于：
  # x_ddot_command
  x_ddot_achieved = ( result['J'] @ result['q_ddot_command'] + result['J_dot'] @ q_dot)
  acceleration_error = (x_ddot_achieved- result['x_ddot_command'])

  print("\n""3. 控制映射验证\n")
  print( "\nx_ddot command =",result['x_ddot_command'],)
  print( "\nx_ddot achieved =", x_ddot_achieved,)
  print("\nerror =",acceleration_error,)
  print("\n||error|| =", f"{np.linalg.norm(acceleration_error):.12e}",)

if __name__ == '__main__':
  main()