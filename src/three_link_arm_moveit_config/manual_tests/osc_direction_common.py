"""固定姿态下的奇异方向实验，假定动力学偏置已补偿。"""
import numpy as np
from three_link_arm_kinematics.operational_space_controller import (OperationalSpaceController,)
from three_link_arm_kinematics.operational_space_dynamics import (OperationalSpaceDynamics,)
from osc_analysis_common import (
  load_joint_trajectory,
  calculate_matrices,
  find_worst_configuration,
)

# 两份方向实验共用的离线分析参数。
ALPHA_LOW = 0.002
ALPHA_FULL = 0.020
LAMBDA_LIMIT = 100.0
ALPHA_TOL = 1e-10

def smooth_weight(alpha):
  """通过Smoothstep平滑调整各任务方向的保留比例。"""
  s = np.clip((alpha - ALPHA_LOW) / (ALPHA_FULL - ALPHA_LOW), 0.0, 1.0)
  return 3.0 * s**2 - 2.0 * s**3

def calculate_force(alpha, U, method, b_test):
  """根据方向权重计算任务力，保留原实验的数值秩判定。"""
  b_direction = U.T @ b_test
  if method == "strict":
    weights = (alpha > ALPHA_TOL).astype(float)
  elif method == "damped":
    epsilon = max(0.0, 1.0 / LAMBDA_LIMIT - alpha[0])
    weights = alpha / (alpha + epsilon)
  elif method == "degraded":
    weights = smooth_weight(alpha)
  else:
    raise ValueError(f"未知方案: {method}")

  F_direction = np.zeros(len(alpha))
  for i in range(len(alpha)):
    if alpha[i] > ALPHA_TOL:
      F_direction[i] = weights[i] * b_direction[i] / alpha[i]
  return U @ F_direction, weights

def analyze_configuration(label, q, controller, osc, b_test):
  J, M, A = calculate_matrices(q, controller, osc)
  U_J, sigma_J, Vt_J = np.linalg.svd(J, full_matrices=True)
  alpha, U_A = np.linalg.eigh(A)
  if alpha[0] < -ALPHA_TOL:
    raise RuntimeError("A出现明显负特征值")
  alpha = np.maximum(alpha, 0.0)

  weak_J = U_J[:, -1]
  weak_A = U_A[:, 0]
  alignment = abs(np.dot(weak_J, weak_A))
  print(f"\n========== {label} ==========")
  print("关节姿态 q [rad]:", q)
  print("Jacobian奇异值:", sigma_J)
  print("Jacobian最弱任务方向:", weak_J)
  print("A的特征值:", alpha)
  print("动力学最弱任务方向:", weak_A)
  print("两个弱方向的绝对点积:", f"{alignment:.9f}")
  print("要求的任务加速度 [m/s^2]:", b_test)

  for name, method in [
    ("严格逆/伪逆", "strict"),
    ("自适应正则化", "damped"),
    ("平滑任务降级", "degraded"),
  ]:
    F, weights = calculate_force(alpha, U_A, method, b_test)
    tau_task = J.T @ F
    a_actual = A @ F
    a_error = b_test - a_actual
    print(f"\n方案: {name}")
    print("方向保留权重:", weights)
    print("实际任务加速度 [m/s^2]:", a_actual)
    print("加速度误差范数 [m/s^2]:", f"{np.linalg.norm(a_error):.9f}")
    print("任务空间力范数 [N]:", f"{np.linalg.norm(F):.6f}")
    print("关节任务力矩 [Nm]:", tau_task)

def run_direction_experiment(b_test, reference_path, gazebo_path):
  controller = OperationalSpaceController()
  osc = OperationalSpaceDynamics()
  for label, filename in [
    ("MoveIt参考姿态", reference_path),
    ("Gazebo实际姿态", gazebo_path),
  ]:
    t, q = load_joint_trajectory(filename)
    worst = find_worst_configuration(t, q, controller, osc, require_positive=False)
    print(f"\n{label}最敏感时刻: {worst['t']:.4f} s")
    analyze_configuration(label, worst["q"], controller, osc, b_test)

  analyze_configuration(
    "完全伸直的奇异姿态", np.zeros(3), controller, osc, b_test,
  )

  