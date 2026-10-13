from pathlib import Path
import sys
import numpy as np
from three_link_arm_kinematics.operational_space_controller import (OperationalSpaceController,)
from three_link_arm_kinematics.operational_space_dynamics import ( OperationalSpaceDynamics,)
TORQUE_LIMIT = np.array([10.0, 10.0, 10.0])
EXPECTED_JOINTS = ["joint1", "joint2", "joint3"]
def main():
  if len(sys.argv) != 2:
    print("用法: python test_moveit_osc_torque_consistency.py 轨迹.npz")
    sys.exit(1)
  path = Path(sys.argv[1]).expanduser().resolve()
  with np.load(path, allow_pickle=False) as data:
    t = data["t"]
    q_ref = data["q"]
    dq_ref = data["dq"]
    ddq_ref = data["ddq"]
    x_ref = data["x"]
    dx_ref = data["dx"]
    ddx_ref = data["ddx"]
    joint_names = data["joint_names"].tolist()
  N = len(t)
  if joint_names != EXPECTED_JOINTS:
    raise ValueError(f"关节顺序错误: {joint_names}")
  if (
    N < 2
    or q_ref.shape != (N, 3)
    or dq_ref.shape != (N, 3)
    or ddq_ref.shape != (N, 3)
    or x_ref.shape != (N, 2)
    or dx_ref.shape != (N, 2)
    or ddx_ref.shape != (N, 2)
  ):
    raise ValueError("输入轨迹形状不符合预期")
  if not all(np.all(np.isfinite(a)) for a in (
    t, q_ref, dq_ref, ddq_ref,
    x_ref, dx_ref, ddx_ref,
  )):
    raise ValueError("输入轨迹存在非有限数值")
  controller = OperationalSpaceController( kp=(10.0, 10.0), kd=(4.43, 4.43),)
  osc = OperationalSpaceDynamics()
  # 与 ROS2 OSC 节点默认配置保持一致
  osc.dynamics.g = 9.8
  tau_osc_history = []
  tau_id_history = []
  sigma_history = []
  reconstruction_errors = []
  reference_errors = []

  for k in range(N):
    # 理想状态：实际关节位置、速度等于期望值
    q = q_ref[k]
    dq = dq_ref[k]
    ddq = ddq_ref[k]
    J = controller.kinematics.jacobian(q)
    sigma_min = np.linalg.svd( J, compute_uv=False)[-1]
    if sigma_min < 1e-6:
      raise RuntimeError( f"轨迹点 {k} 接近奇异位置: " f"sigma_min={sigma_min:.6e}")

    # 验证 MoveIt 任务空间参考与现有运动学一致
    x_check = controller.kinematics.forward_kinematics(q)
    dx_check = J @ dq
    ddx_check = ( J @ ddq + controller.kinematics.jacobian_dot(q, dq) @ dq)
    reference_error = max(
      np.max(np.abs(x_check - x_ref[k])),
      np.max(np.abs(dx_check - dx_ref[k])),
      np.max(np.abs(ddx_check - ddx_ref[k])),
    )

    # 一级任务：原 OSC 加速度控制
    command = controller.calculate_task_acceleration_command(
      q, dq,
      x_ref[k],
      dx_ref[k],
      ddx_ref[k],
    )
    task_result = osc.calculate_task_force(q, dq, command["x_ddot_command"])
    tau_task = task_result["tau"]
    # 动态一致零空间
    null_result = osc.calculate_null_space_projector(q)
    N_T = null_result["N_T"]
    # 关节空间完整逆动力学前馈
    M = task_result["M"]
    V = task_result["V"]
    G = task_result["G"]
    tau_id = M @ ddq + V + G
    # 理想跟踪状态下，姿态 PD 误差为零
    tau_posture = N_T @ tau_id
    # OSC 总力矩
    tau_osc = tau_task + tau_posture
    reconstruction_error = np.max( np.abs(tau_osc - tau_id))
    tau_osc_history.append(tau_osc)
    tau_id_history.append(tau_id)
    sigma_history.append(sigma_min)
    reconstruction_errors.append(reconstruction_error)
    reference_errors.append(reference_error)
  tau_osc_history = np.asarray(tau_osc_history)
  tau_id_history = np.asarray(tau_id_history)
  sigma_history = np.asarray(sigma_history)
  max_torque = np.max(np.abs(tau_osc_history), axis=0)
  saturated = np.any( np.abs(tau_osc_history) > TORQUE_LIMIT, axis=1,)

  print("\n========== MoveIt -> OSC 力矩一致性测试 ==========")
  print("轨迹点数:", N)
  print(f"轨迹时长: {t[-1]:.6f} s")
  print("\n任务空间参考最大一致性误差:")
  print(f"{max(reference_errors):.6e}")
  print("\nOSC 与逆动力学最大力矩差:")
  print(f"{max(reconstruction_errors):.6e} Nm")
  print("\n三个关节最大绝对需求力矩 (Nm):")
  print(max_torque)
  print("\n超过 ±10 Nm 限制的采样点数:")
  print(int(np.sum(saturated)))
  min_index = int(np.argmin(sigma_history))
  print("\n最小 Jacobian 奇异值:")
  print(f"{sigma_history[min_index]:.9e}")
  print(f"出现时间: {t[min_index]:.4f} s")

  if max(reference_errors) > 1e-7:
    raise RuntimeError("运动学参考不一致")
  if max(reconstruction_errors) > 1e-6:
    raise RuntimeError("OSC 动力学分解不一致")
  print("\n动力学一致性验证通过")
  if np.any(saturated):
    print("警告：存在超出力矩限制的轨迹点")
  else:
    print("理想轨迹前馈力矩未超出 ±10 Nm")

if __name__ == "__main__":
  main()
