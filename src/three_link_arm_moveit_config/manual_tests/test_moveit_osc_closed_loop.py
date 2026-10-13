from pathlib import Path
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from three_link_arm_kinematics.operational_space_controller import (OperationalSpaceController,)
from three_link_arm_kinematics.operational_space_dynamics import ( OperationalSpaceDynamics,)

DT_SIM = 0.001
DT_CTRL = 0.010
HOLD_SECONDS = 1.0
TORQUE_LIMIT = np.array([10.0, 10.0, 10.0])
K_P0 = np.eye(3) * 1.0
K_D0 = np.eye(3) * 0.5
def sample_joint_reference(t_query, t, q, dq, ddq):
  """从100Hz参考轨迹获取当前关节位置、速度、加速度。"""
  if t_query >= t[-1]:
    return q[-1].copy(), np.zeros(3), np.zeros(3)
  def interpolated(a):
      return np.array([ np.interp(t_query, t, a[:, j]) for j in range(3)])
  return (
      interpolated(q),
      interpolated(dq),
      interpolated(ddq),
  )
def main():
  if len(sys.argv) != 2:
    raise SystemExit( "用法: python test_moveit_osc_closed_loop.py " "*_interp_100hz.npz")
  source = Path(sys.argv[1]).expanduser().resolve()
  with np.load(source, allow_pickle=False) as d:
    t_ref = d["t"].copy()
    q_ref = d["q"].copy()
    dq_ref = d["dq"].copy()
    ddq_ref = d["ddq"].copy()
    names = d["joint_names"].tolist()
    task_frame = str(d["task_frame"])

  n = len(t_ref)
  if ( names != ["joint1", "joint2", "joint3"]or task_frame != "arm_base"):
    raise ValueError(f"关节顺序或坐标系错误: {names}, {task_frame}")
  if (
    n < 2
    or t_ref.shape != (n,)
    or any( a.shape != (n, 3)for a in [q_ref, dq_ref, ddq_ref])
  ):
    raise ValueError("输入轨迹形状错误")
  if any( not np.all(np.isfinite(a)) for a in [t_ref, q_ref, dq_ref, ddq_ref]):
    raise ValueError("轨迹存在非有限数值")
  if abs(t_ref[0]) > 1e-9 or np.any(np.diff(t_ref) <= 0):
    raise ValueError("轨迹时间必须从0开始并严格递增")
  control_steps = round(DT_CTRL / DT_SIM)
  if abs( DT_CTRL / DT_SIM - control_steps ) > 1e-12:
    raise ValueError("控制周期必须是积分周期的整数倍")

  # 1. 复用已有 OSC
  controller = OperationalSpaceController( kp=(10.0, 10.0), kd=(4.43, 4.43),)
  osc = OperationalSpaceDynamics()
  osc.dynamics.g = 9.8

  # 2. 初始化实际机械臂
  q = q_ref[0].copy()
  dq = dq_ref[0].copy()
  tau_hold = np.zeros(3)
  simulation_duration = t_ref[-1] + HOLD_SECONDS
  n_steps = int( np.ceil(simulation_duration / DT_SIM) )
  history = {
    key: [] for key in [
      "time", "q", "q_d", "e_q",
      "x", "x_d", "e_x",
      "tau", "tau_raw", "sigma_actual",
    ]
  }
  saturated_steps = 0

  # 3. 闭环仿真
  for step in range(n_steps + 1):
    time_now = step * DT_SIM
    # 控制器每10ms更新一次
    if step % control_steps == 0:
      # 3.1 从 MoveIt 轨迹读取期望关节状态
      qd, dqd, ddqd = sample_joint_reference( time_now, t_ref, q_ref, dq_ref, ddq_ref,)
      # 3.2 转换为任务空间参考
      Jd = controller.kinematics.jacobian(qd)
      xd = controller.kinematics.forward_kinematics(qd)
      dxd = Jd @ dqd
      ddxd = ( Jd @ ddqd + controller.kinematics.jacobian_dot( qd, dqd ) @ dqd)
      # 3.3 检查当前实际姿态的奇异性
      J = controller.kinematics.jacobian(q)
      sigma = np.linalg.svd( J, compute_uv=False)[-1]
      if (
        sigma < 0.005
        or np.max(np.abs(q)) > 3.14
        or np.max(np.abs(dq)) > 10.0
        or np.linalg.norm(q - qd) > 1.0
      ):
        raise RuntimeError( f"t={time_now:.3f}s 安全检查失败：" f"sigma={sigma:.5g}, q={q}, dq={dq}")

      # 3.4 OSC 主任务加速度命令
      task_cmd = (controller.calculate_task_acceleration_command( q, dq, xd, dxd, ddxd,))

      # 3.5 严格 OSC 动力学计算
      dyn = osc.calculate_task_force( q, dq, task_cmd["x_ddot_command"],)
      tau_task = dyn["tau"]

      # 3.6 动态一致零空间
      null_result = ( osc.calculate_null_space_projector(q))
      N_T = null_result["N_T"]

      # 3.7 移动姿态目标的逆动力学前馈
      tau_id = ( dyn["M"] @ ddqd + dyn["V"] + dyn["G"])
      # 3.8 零空间姿态控制
      tau_0 = ( tau_id + K_P0 @ (qd - q) + K_D0 @ (dqd - dq) )
      tau_posture = N_T @ tau_0

      # 3.9 合成任务力矩和零空间力矩
      tau_raw = tau_task + tau_posture
      if not np.all(np.isfinite(tau_raw)):
        raise RuntimeError( f"t={time_now:.3f}s 力矩出现非法数值" )

      # 3.10 力矩限幅
      if np.any(np.abs(tau_raw) > TORQUE_LIMIT):
        saturated_steps += 1
      tau_hold = np.clip( tau_raw, -TORQUE_LIMIT, TORQUE_LIMIT,)

      # 3.11 记录当前控制采样的数据
      x_actual = task_cmd["x"]
      record = {
        "time": time_now,
        "q": q.copy(),
        "q_d": qd.copy(),
        "e_q": (qd - q).copy(),
        "x": x_actual.copy(),
        "x_d": xd.copy(),
        "e_x": (xd - x_actual).copy(),
        "tau": tau_hold.copy(),
        "tau_raw": tau_raw.copy(),
        "sigma_actual": sigma,
      }

      for key, value in record.items():
        history[key].append(value)

    if step == n_steps:
      break

    # 4. 被控对象动力学，1ms积分
    M = osc.dynamics.calculate_mass_matrix(q)
    V = osc.dynamics.calculate_velocity_term(q, dq)
    G = osc.dynamics.calculate_gravity(q)
    qdd = np.linalg.solve(M, tau_hold - V - G)

    # 半隐式欧拉积分
    dq = dq + qdd * DT_SIM
    q = q + dq * DT_SIM

    if ( not np.all(np.isfinite(q)) or not np.all(np.isfinite(dq))):
      raise RuntimeError(f"t={time_now:.3f}s 仿真积分发散")

  # 5. 汇总结果
  h = { key: np.asarray(values)for key, values in history.items() }

  # 只用运动阶段计算跟踪RMS
  motion = h["time"] <= t_ref[-1]
  rms_x = np.sqrt( np.mean(h["e_x"][motion] ** 2, axis=0) )
  max_x = np.max( np.abs(h["e_x"][motion]), axis=0 )
  rms_q = np.sqrt( np.mean(h["e_q"][motion] ** 2, axis=0) )
  max_tau = np.max( np.abs(h["tau"]), axis=0 )
  max_tau_raw = np.max( np.abs(h["tau_raw"]), axis=0 )

  print("\n========== MoveIt -> OSC 离线闭环 ==========")
  print(f"轨迹时长: {t_ref[-1]:.6f} s")
  print(f"控制周期: {DT_CTRL} s")
  print(f"动力学积分周期: {DT_SIM} s")
  print("\n运动阶段末端 RMS [mm]:")
  print(rms_x * 1000)
  print("\n运动阶段末端最大误差 [mm]:")
  print(max_x * 1000)
  print("\n运动阶段关节 RMS [rad]:")
  print(rms_q)
  print("\n运动结束并保持后的末端误差 [mm]:")
  print(h["e_x"][-1] * 1000)
  print("\n实际输出最大力矩 [Nm]:")
  print(max_tau)
  print("\n限幅前最大需求力矩 [Nm]:")
  print(max_tau_raw)
  print("\n发生力矩限幅的控制周期数:")
  print(saturated_steps)
  print("\n实际轨迹最小奇异值:")
  print(np.min(h["sigma_actual"]))

  # 6. 保存分析结果
  output_dir = ( source.parent / (source.stem + "_closed_loop") )
  output_dir.mkdir(parents=True, exist_ok=True)
  np.savez_compressed( output_dir / "tracking.npz",  **h, )

  # 7. 保存误差和力矩曲线
  plots = [
    ("e_x", "Task error (mm)", "task_error.png", 1000.0),
    ("e_q", "Joint error (rad)", "joint_error.png", 1.0),
    ("tau", "Applied torque (Nm)", "torque.png", 1.0),
  ]

  for key, ylabel, filename, factor in plots:
    fig, ax = plt.subplots(figsize=(10, 4))
    labels = (
      ["x", "y"]
      if key == "e_x"
      else ["joint1", "joint2", "joint3"]
    )

    for j in range(h[key].shape[1]):
      ax.plot( h["time"], h[key][:, j] * factor, label=labels[j], )
    ax.axvline( t_ref[-1], color="gray", linestyle=":", label="Trajectory end",)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel(ylabel)
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    fig.savefig(output_dir / filename, dpi=150)
    plt.close(fig)

  print("\n结果保存至:")
  print(output_dir)


if __name__ == "__main__":
  main()
