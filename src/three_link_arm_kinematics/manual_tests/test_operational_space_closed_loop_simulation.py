import numpy as np
import matplotlib.pyplot as plt
from three_link_arm_kinematics.operational_space_controller import (OperationalSpaceController,)
from three_link_arm_kinematics.operational_space_dynamics import (OperationalSpaceDynamics,)
def sample_task_trajectory(t,move_duration,x_start,x_goal,):
  """
  使用五次时间缩放生成任务空间轨迹。

  t:
    当前仿真时间 [s]

  move_duration:
    运动持续时间 [s]

  x_start:
    任务空间起点 [m]

  x_goal:
    任务空间终点 [m]
  """
  delta_x = x_goal - x_start
  # 轨迹结束以后保持终点
  if t >= move_duration:
    return ( x_goal.copy(),np.zeros(2),np.zeros(2),)
  # 归一化时间
  r = t / move_duration
  # 五次时间缩放
  s = (10.0 * r**3 - 15.0 * r**4 + 6.0 * r**5)
  # s 对真实时间的一阶导数
  s_dot = ( 30.0 * r**2 - 60.0 * r**3 + 30.0 * r**4) / move_duration
  # s 对真实时间的二阶导数
  s_ddot = ( 60.0 * r - 180.0 * r**2 + 120.0 * r**3) / move_duration**2
  x_d = ( x_start + s * delta_x)
  x_dot_d = ( s_dot * delta_x)
  x_ddot_d = ( s_ddot * delta_x)
  return ( x_d, x_dot_d, x_ddot_d,)

def run_simulation(dt_control): 
  dt_sim = 0.001
  simulation_duration = 4.0
  move_duration = 2.0
  control_steps = round(dt_control / dt_sim)
  total_steps = round(simulation_duration / dt_sim)
  controller = OperationalSpaceController(
    kp=(10.0, 10.0),
    kd=(4.43, 4.43),
  )
  operational_dynamics = OperationalSpaceDynamics()

  # 每次实验都从完全相同的初始状态开始
  q = np.array([0.5, -0.8, 1.2])
  q_dot = np.zeros(3)
  x_start = controller.kinematics.forward_kinematics(q)
  x_goal = x_start + np.array([0.05, -0.03])
  q_posture_desired = np.zeros(3)
  Kp_posture = np.eye(3)
  Kd_posture = 0.5 * np.eye(3)
  time_history = []
  error_history = []
  torque_history = []
  q_dot_history = []

  # 两次控制更新之间保持的力矩
  tau_hold = np.zeros(3)
  for step in range(total_steps + 1):
    t = step * dt_sim
    x_d, x_dot_d, x_ddot_d = sample_task_trajectory(t, move_duration, x_start, x_goal)

    # 当前实际末端位置
    x = controller.kinematics.forward_kinematics(q)

    # 每隔 control_steps 个仿真步更新一次控制力矩
    if step % control_steps == 0:
      command = controller.calculate_task_acceleration_command( q, q_dot, x_d, x_dot_d, x_ddot_d)
      x_ddot_command = command["x_ddot_command"]
      task_result = operational_dynamics.calculate_task_force( q, q_dot, x_ddot_command)
      tau_task = task_result["tau"]
      null_result = (operational_dynamics.calculate_null_space_projector(q))
      N_T = null_result["N_T"]
      tau_0 = (
        Kp_posture @ (q_posture_desired - q)
        - Kd_posture @ q_dot
      )
      tau_posture = N_T @ tau_0
      tau_hold = tau_task + tau_posture

    # 记录实际状态与当前期望轨迹的误差
    time_history.append(t)
    error_history.append((x_d - x).copy())
    torque_history.append(tau_hold.copy())
    q_dot_history.append(q_dot.copy())

    # 最后一个时刻只记录，不再积分
    if step == total_steps:
      break

    # 仿真器每 1 ms 都重新计算实际动力学
    # 暂时复用已有接口读取 M、V、G
    plant_result = operational_dynamics.calculate_task_force(q, q_dot, np.zeros(2))
    M = plant_result["M"]
    V = plant_result["V"]
    G = plant_result["G"]

    # 在控制更新间隔内，tau_hold 保持不变
    q_ddot = np.linalg.solve(M, tau_hold - V - G)

    # 半隐式欧拉积分
    q_dot = q_dot + q_ddot * dt_sim
    q = q + q_dot * dt_sim

  time_history = np.asarray(time_history)
  error_history = np.asarray(error_history)
  torque_history = np.asarray(torque_history)
  q_dot_history = np.asarray(q_dot_history)
  rms_error = np.sqrt(np.mean(error_history**2, axis=0))
  max_error = np.max(np.abs(error_history), axis=0)
  final_error_norm = np.linalg.norm(error_history[-1])
  max_torque = np.max(np.abs(torque_history), axis=0)
  max_joint_speed = np.max(np.abs(q_dot_history))

  return {
    "dt_control": dt_control,
    "time": time_history,
    "error": error_history,
    "rms_error": rms_error,
    "max_error": max_error,
    "final_error_norm": final_error_norm,
    "max_torque": max_torque,
    "max_joint_speed": max_joint_speed,
  }

# def main():
#   np.set_printoptions(precision=8,suppress=True,)
#   controller = OperationalSpaceController( kp=(10.0, 10.0), kd=(4.43, 4.43),)
#   operational_dynamics = (OperationalSpaceDynamics())

#   # 仿真参数
#   dt = 0.002
#   move_duration = 2.0
#   simulation_duration = 4.0
#   # 初始关节状态
#   q = np.array([ 0.5, -0.8, 1.2,])
#   q_dot = np.zeros(3)

#   # 任务空间起点和终点
#   x_start = ( controller.kinematics.forward_kinematics(q))
#   x_goal = (x_start + np.array([ 0.05, -0.03,]))

#   # 零空间姿态控制
#   q_posture_desired = np.zeros(3)
#   Kp_posture = np.diag([ 1.0, 1.0, 1.0,])
#   Kd_posture = np.diag([0.5, 0.5, 0.5,])

#   # 数据记录
#   time_history = []
#   x_history = []
#   x_desired_history = []
#   error_history = []
#   q_history = []
#   q_dot_history = []
#   torque_history = []

#   # 闭环仿真

#   time_values = np.arange( 0.0, simulation_duration + dt, dt,)
#   for t in time_values:
#     # 1. 当前任务空间期望状态
#     ( x_d, x_dot_d, x_ddot_d, ) = sample_task_trajectory( t, move_duration, x_start, x_goal,)

#     # 2. 任务空间反馈
#     task_command = ( controller.calculate_task_acceleration_command( q, q_dot, x_d, x_dot_d, x_ddot_d,))
#     x = task_command["x"]
#     x_ddot_command = (task_command[ "x_ddot_command"])

#     # 3. 严格操作空间动力学
#     task_dynamics = ( operational_dynamics.calculate_task_force( q, q_dot,x_ddot_command,))
#     tau_task = task_dynamics["tau"]
#     M = task_dynamics["M"]
#     V = task_dynamics["V"]
#     G = task_dynamics["G"]

#     # 4. 动态一致零空间
#     null_result = ( operational_dynamics .calculate_null_space_projector(q))
#     N_T = null_result["N_T"]

#     # 5. 姿态二级任务
#     tau_0 = (Kp_posture @ (q_posture_desired- q) - Kd_posture @ q_dot)
#     tau_posture = ( N_T @ tau_0)

#     # 6. 最终控制力矩
#     tau_total = ( tau_task + tau_posture)

#     # 7. 真实动力学
#     q_ddot = np.linalg.solve( M, tau_total - V - G,)

#     # 记录当前状态
#     time_history.append(t)
#     x_history.append(x.copy())
#     x_desired_history.append( x_d.copy())
#     error_history.append((x_d - x).copy())
#     q_history.append(q.copy())
#     q_dot_history.append(q_dot.copy())
#     torque_history.append(tau_total.copy())

#     # 8. 半隐式欧拉积分
#     q_dot = ( q_dot + q_ddot * dt)
#     q = ( q + q_dot * dt )

#   # 转换成 numpy 数组
#   time_history = np.asarray(time_history)
#   x_history = np.asarray(x_history)
#   x_desired_history = np.asarray( x_desired_history)
#   error_history = np.asarray(error_history)
#   q_history = np.asarray(q_history)
#   torque_history = np.asarray(torque_history)

#   # 跟踪指标
#   rms_error = np.sqrt(np.mean( error_history**2, axis=0,))
#   max_error = np.max(np.abs(error_history), axis=0,)
#   final_error = ( error_history[-1])
#   max_torque = np.max( np.abs(torque_history), axis=0,)
#   print("\nOperational-Space Closed Loop ")
#   print("\nx_start =",x_start,)
#   print("\nx_goal =",x_goal,)
#   print("\nRMS task error =",rms_error,)
#   print("\nMax task error =", max_error,)
#   print("\nFinal task error =",final_error,)
#   print( "\nMax |tau| =", max_torque,)
#   print("\nFinal q =",q,)

#   # 图 1：末端轨迹
#   plt.figure()
#   plt.plot(
#     x_desired_history[:, 0],
#     x_desired_history[:, 1],
#     "--",
#     label="desired",
#   )
#   plt.plot(
#     x_history[:, 0],
#     x_history[:, 1],
#     label="actual",
#   )
#   plt.xlabel("x [m]")
#   plt.ylabel("y [m]")
#   plt.title("任务空间轨迹")
#   plt.grid(True)
#   plt.legend()
#   plt.tight_layout()

#   # 图 2：任务空间误差
#   plt.figure()
#   plt.plot(
#     time_history,
#     error_history[:, 0],
#     label="x error",
#   )
#   plt.plot(
#     time_history,
#     error_history[:, 1],
#     label="y error",
#   )
#   plt.xlabel("时间 [s]")
#   plt.ylabel("任务 误差 [m]")
#   plt.title("任务空间跟踪误差")
#   plt.grid(True)
#   plt.legend()
#   plt.tight_layout()

#   # 图 3：关节运动
#   plt.figure()
#   for joint_index in range(3):
#     plt.plot(
#       time_history,
#       q_history[:, joint_index],
#       label=(f"joint{joint_index + 1}"),
#     )

#   plt.xlabel("Time [s]")
#   plt.ylabel("关节 位置 [rad]")
#   plt.title("关节运动")
#   plt.grid(True)
#   plt.legend()
#   plt.tight_layout()

#   # 图 4：关节控制力矩

#   plt.figure()
#   for joint_index in range(3):
#     plt.plot(
#       time_history,
#       torque_history[:, joint_index],
#       label=(f"joint{joint_index + 1}"),
#     )
#   plt.xlabel("时间 [s]")
#   plt.ylabel("力矩 [N*m]")
#   plt.title("关节力矩")
#   plt.grid(True)
#   plt.legend()
#   plt.tight_layout()
#   plt.show()

def main():
  control_periods = [ 0.002, 0.005, 0.010, 0.020,]
  results = []
  for dt_control in control_periods:
    result = run_simulation(dt_control)
    results.append(result)

  print("\n========== 采样周期比较 ==========\n")
  print(
    f"{'dt(ms)':>8}"
    f"{'RMSx(mm)':>12}"
    f"{'RMSy(mm)':>12}"
    f"{'MaxEx(mm)':>12}"
    f"{'MaxEy(mm)':>12}"
    f"{'FinalE(mm)':>13}"
    f"{'MaxTau1':>12}"
    f"{'MaxQdot':>12}"
  )

  for result in results:
    rms = result["rms_error"] * 1000.0
    max_e = result["max_error"] * 1000.0
    final_e = result["final_error_norm"] * 1000.0
    print(
      f"{result['dt_control'] * 1000:8.1f}"
      f"{rms[0]:12.5f}"
      f"{rms[1]:12.5f}"
      f"{max_e[0]:12.5f}"
      f"{max_e[1]:12.5f}"
      f"{final_e:13.6f}"
      f"{result['max_torque'][0]:12.4f}"
      f"{result['max_joint_speed']:12.4f}"
    )

  # 只画一张不同控制周期的误差对比图
  plt.figure()
  for result in results:
    error_norm_mm = (np.linalg.norm(result["error"], axis=1)* 1000.0)
    label = f"{result['dt_control'] * 1000:g} ms"
    plt.plot( result["time"], error_norm_mm, label=label,)

  plt.xlabel("Time [s]")
  plt.ylabel("Task Error Norm [mm]")
  plt.title("Control Sampling Period Comparison")
  plt.grid(True)
  plt.legend()
  plt.tight_layout()
  plt.show()


if __name__ == "__main__":
  main()