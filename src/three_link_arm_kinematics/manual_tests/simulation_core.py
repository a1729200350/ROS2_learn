import numpy as np
from three_link_arm_kinematics.dynamics_model import DynamicsModel
from three_link_arm_kinematics.operational_space_controller import (OperationalSpaceController,)
from three_link_arm_kinematics.operational_space_dynamics import (OperationalSpaceDynamics,)
def sample_trajectory(t, duration, start, goal):
  """二维末端五次时间缩放轨迹。"""
  if t >= duration:
    return goal.copy(), np.zeros(2), np.zeros(2)
  r = t / duration
  delta = goal - start
  s = 10*r**3 - 15*r**4 + 6*r**5
  s_dot = (30*r**2 - 60*r**3 + 30*r**4) / duration
  s_ddot = (60*r - 180*r**2 + 120*r**3) / duration**2

  return (
    start + s * delta,
    s_dot * delta,
    s_ddot * delta,
  )


def run_osc_simulation( plant_model,
 controller_model=None,
 dt_control=0.002,
 dt_sim=0.001,
 enable_posture=True,
 q_initial=None,
 goal_delta=None,):
  """
  连续离线操作空间控制仿真  OSC (Operational Space control)仿真。
  plant_model: 实际机器人动力学
  controller_model: 控制器使用的动力学
  dt_control: 控制周期 [s]
  dt_sim: 动力学积分周期 [s]
  """
  if controller_model is None:
    controller_model = DynamicsModel()
  ratio = dt_control / dt_sim
  control_steps = round(ratio)
  if (
    dt_sim <= 0
    or dt_control <= 0
    or control_steps < 1
    or not np.isclose(ratio, control_steps)
  ):
    raise ValueError("控制周期必须是仿真步长的正整数倍")

  controller = OperationalSpaceController(
    kp=(10.0, 10.0),
    kd=(4.43, 4.43),
  )

  # 只使用标称模型计算操作空间力矩
  osc = OperationalSpaceDynamics(dynamics=controller_model)
  # 初始关节状态
  if q_initial is None:
    q = np.array([0.5, -0.8, 1.2], dtype=float)
  else:
    q = np.asarray(q_initial, dtype=float).copy()
  if q.shape != (3,) or not np.all(np.isfinite(q)):
    raise ValueError("q_initial 必须包含三个有限关节角")
  q_dot = np.zeros(3)

  q_posture_desired = np.zeros(3)
  Kp_posture = np.eye(3)
  Kd_posture = 0.5 * np.eye(3)
  move_duration = 2.0
  simulation_duration = 4.0

  # 末端起点
  x_start = controller.kinematics.forward_kinematics(q)
  # 末端相对目标位移
  if goal_delta is None:
    delta = np.array([0.05, -0.03])
  else:
    delta = np.asarray(goal_delta, dtype=float)
  if delta.shape != (2,) or not np.all(np.isfinite(delta)):
    raise ValueError("goal_delta 必须是二维有限向量")
  x_goal = x_start + delta

  records = {
    "time": [],
    "error": [],
    "q": [],
    "q_dot": [],
    "tau": [],
    "sigma_min": [],
  }
  tau_hold = np.zeros(3)
  total_steps = round(simulation_duration / dt_sim)
  for step in range(total_steps + 1):
    t = step * dt_sim
    x_d, x_dot_d, x_ddot_d = sample_trajectory(t, move_duration, x_start, x_goal)
    x = controller.kinematics.forward_kinematics(q)
    J = controller.kinematics.jacobian(q)
    singular_values = np.linalg.svd(J,compute_uv=False,)
    sigma_min = singular_values[-1]
    # 控制器按指定周期更新
    if step % control_steps == 0:
      command = controller.calculate_task_acceleration_command(q, q_dot, x_d, x_dot_d, x_ddot_d)
      task = osc.calculate_task_force( q, q_dot, command["x_ddot_command"])

      # null_result = osc.calculate_null_space_projector(q)
      # tau_0 = ( Kp_posture @ (q_posture_desired - q) - Kd_posture @ q_dot)
      # tau_hold = ( task["tau"] + null_result["N_T"] @ tau_0)
      # 一级任务力矩
      tau_task = task["tau"]
      if enable_posture:
        # 动态一致零空间投影器
        null_result = osc.calculate_null_space_projector(q)
        N_T = null_result["N_T"]
        # 原始姿态 PD 力矩
        tau_0 = (Kp_posture @ (q_posture_desired - q) - Kd_posture @ q_dot)
        # 零空间姿态力矩
        tau_posture = N_T @ tau_0
        # 一级任务 + 二级任务
        tau_hold = tau_task + tau_posture
      else:
        # 关闭姿态二级任务
        # 只保留一级任务
        tau_hold = tau_task.copy()


    # 保存本时刻数据
    records["time"].append(t)
    records["error"].append((x_d - x).copy())
    records["q"].append(q.copy())
    records["q_dot"].append(q_dot.copy())
    records["tau"].append(tau_hold.copy())
    records["sigma_min"].append(sigma_min)
    if step == total_steps:
      break

    # 真实机器人动力学：与控制器模型分离
    M_real = plant_model.calculate_mass_matrix(q)
    V_real = plant_model.calculate_velocity_term(q, q_dot)
    G_real = plant_model.calculate_gravity(q)
    q_ddot = np.linalg.solve( M_real, tau_hold - V_real - G_real,)

    # 半隐式欧拉积分
    q_dot = q_dot + q_ddot * dt_sim
    q = q + q_dot * dt_sim

    if not ( np.all(np.isfinite(q)) and np.all(np.isfinite(q_dot))):
      raise FloatingPointError(f"仿真状态在 t={t:.4f}s 出现非有限数值")

  return { key: np.asarray(values) for key, values in records.items() }
