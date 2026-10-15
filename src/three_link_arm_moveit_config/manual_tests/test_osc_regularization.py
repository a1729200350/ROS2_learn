"""任务正则化、零空间泄漏与分离式正则化的联合对照。"""
import argparse
import numpy as np
from osc_analysis_common import load_joint_trajectory, find_worst_configuration
from three_link_arm_kinematics.operational_space_controller import (OperationalSpaceController,)
from three_link_arm_kinematics.operational_space_dynamics import (OperationalSpaceDynamics,)

LAMBDA_MAX_LIMIT = 100.0
TAU_0 = np.array([1.0, -0.5, 0.3])

def analyze_force(label, filename):
  t, q = load_joint_trajectory(filename)
  controller = OperationalSpaceController()
  osc = OperationalSpaceDynamics()
  worst = find_worst_configuration(t, q, controller, osc)
  time_peak = worst["t"]
  A = worst["A"]
  eigvals, eigvecs = np.linalg.eigh(A)
  lambda_max = 1.0 / eigvals[0]

  print(f"\n========== {label} ==========")
  print(f"最大Lambda特征值: {lambda_max:.6f}")
  print(f"出现时间: {time_peak:.4f} s")
  print(f"A最小特征值: {eigvals[0]:.9f}")
  # 沿最容易放大的任务空间方向，
  # 测试单位加速度命令
  a_cmd = eigvecs[:, 0]
  print("\n正则化对比:")
  print("epsilon    力范数(N)    加速度实现率")
  for eps in [0.0, 0.001, 0.005, 0.01]:
    F = np.linalg.solve( A + eps * np.eye(2), a_cmd,)
    # 假设动力学偏置项已经准确补偿
    a_actual = A @ F
    force_norm = np.linalg.norm(F)
    # 沿期望加速度方向的比例
    tracking_ratio = float( a_cmd @ a_actual)
    print(
      f"{eps:8.4f}  "
      f"{force_norm:12.5f}  "
      f"{tracking_ratio * 100:10.3f}%"
    )

def test_projector(J, M, A, epsilon, tau0):
  """检查零空间残差、投影幂等性及姿态力矩产生的末端加速度。"""
  Lambda_eps = np.linalg.solve( A + epsilon * np.eye(2), np.eye(2),)
  Minv = np.linalg.solve(M, np.eye(3))
  # 动态一致伪逆的正则化版本
  Jbar_eps = Minv @ J.T @ Lambda_eps
  # 关节速度空间矩阵
  N_eps = np.eye(3) - Jbar_eps @ J
  # 力矩空间矩阵
  NT_eps = N_eps.T
  # 三个投影性质残差
  E1 = np.linalg.norm( J @ N_eps, ord="fro",)
  E2 = np.linalg.norm( J @ Minv @ NT_eps, ord="fro",)
  E3 = np.linalg.norm( N_eps @ N_eps - N_eps, ord="fro",)
  # 投影后的姿态力矩
  tau_null = NT_eps @ tau0
  # 该姿态力矩产生的末端加速度增量
  a_leak = J @ np.linalg.solve(M, tau_null)
  # 核对理论恒等式：
  # J M^-1 N_eps^T
  # = epsilon Lambda_eps J M^-1
  theoretical_leak_map = ( epsilon * Lambda_eps @ J @ Minv)
  measured_leak_map = J @ Minv @ NT_eps
  identity_error = np.linalg.norm( measured_leak_map - theoretical_leak_map, ord="fro",)
  return {
    "E1": E1,
    "E2": E2,
    "E3": E3,
    "tau_null_norm": np.linalg.norm(tau_null),
    "a_leak_norm": np.linalg.norm(a_leak),
    "a_leak": a_leak,
    "identity_error": identity_error,
  }

def analyze_projector(label, filename):
  t, q = load_joint_trajectory(filename)
  controller = OperationalSpaceController()
  osc = OperationalSpaceDynamics()
  worst = find_worst_configuration(t, q, controller, osc)
  alpha_min = worst["alpha_min"]
  # 保证正则化 Lambda 的最大特征值 <= 100
  epsilon_adaptive = max( 0.0, 1.0 / LAMBDA_MAX_LIMIT - alpha_min,)
  print(f"\n========== {label} ==========")
  print(f"最敏感时刻: {worst['t']:.4f} s")
  print(f"关节角: {worst['q']}")
  print(f"A 最小特征值: {alpha_min:.9f}")
  print(f"原始 Lambda 最大特征值: {1/alpha_min:.6f}")
  print(f"自适应 epsilon: {epsilon_adaptive:.9f}")
  cases = [ ("严格投影", 0.0), ("固定0.001", 0.001), ("固定0.005", 0.005),("自适应", epsilon_adaptive),]
  print("\n---------- 投影器性质 ----------")
  print(
    f"{'方案':<12}"
    f"{'epsilon':>12}"
    f"{'E1':>14}"
    f"{'E2':>14}"
    f"{'E3':>14}"
  )
  for name, eps in cases:
    result = test_projector( worst["J"], worst["M"], worst["A"], eps,TAU_0,)
    print(
      f"{name:<12}"
      f"{eps:>12.6f}"
      f"{result['E1']:>14.5e}"
      f"{result['E2']:>14.5e}"
      f"{result['E3']:>14.5e}"
    )
  print("\n---------- 零空间力矩泄漏 ----------")
  print("测试原始姿态力矩 tau0 =", TAU_0, "Nm")
  for name, eps in cases:
    result = test_projector( worst["J"], worst["M"], worst["A"], eps,TAU_0,)
    print(f"\n方案: {name}")
    print(f"epsilon: {eps:.9f}")
    print( "投影后力矩范数:", f"{result['tau_null_norm']:.9f} Nm",)
    print("末端加速度泄漏:", result["a_leak"], "m/s^2",)
    print( "泄漏加速度范数:", f"{result['a_leak_norm']:.9e} m/s^2",)
    print("理论恒等式残差:", f"{result['identity_error']:.9e}",)

def analyze_split(label, filename):
  t, q = load_joint_trajectory(filename)
  ctrl = OperationalSpaceController()
  osc = OperationalSpaceDynamics()
  worst = find_worst_configuration(t, q, ctrl, osc)
  alpha = worst["alpha_min"]
  time_peak = worst["t"]
  J, M, A = worst["J"], worst["M"], worst["A"]

  # 自适应正则化
  eps = max(0.0, 1.0 / LAMBDA_MAX_LIMIT - alpha)
  eigenvalues, eigenvectors = np.linalg.eigh(A)
  # 单位加速度命令：最敏感的任务方向
  a_cmd = eigenvectors[:, 0]
  Minv = np.linalg.solve(M, np.eye(3))
  # 方案A：严格投影
  NT_exact = (np.eye(3)  - J.T @ np.linalg.solve(A, J @ Minv))
  # 方案B：正则化投影
  NT_damped = ( np.eye(3)  - J.T @ np.linalg.solve( A + eps * np.eye(2), J @ Minv, ))
  # 方案C：利用零空间基构造严格投影
  U, s, Vt = np.linalg.svd( J, full_matrices=True)
  if s[-1] < 1e-8:
    raise ValueError("Jacobian 接近秩亏，当前一维零空间测试不适用")
  Z = Vt[-1:, :].T  # 3×1
  NT_basis = (M @ Z @ np.linalg.solve( Z.T @ M @ Z,Z.T, ))
  print(f"\n========== {label} ==========")
  print(f"最敏感时刻: {time_peak:.4f} s")
  print(f"alpha_min: {alpha:.9f}")
  print(f"epsilon: {eps:.9f}")
  print("两种严格投影构造差异:",np.linalg.norm(NT_exact - NT_basis),)
  print( "零空间基正交残差:",np.linalg.norm(J @ Z),)
  F_exact = np.linalg.solve(A, a_cmd)
  F_damped = np.linalg.solve( A + eps * np.eye(2), a_cmd)
  cases = [
    ("A 严格OSC", F_exact, NT_exact),
    ("B 全部正则化", F_damped, NT_damped),
    ("C 分离式正则化", F_damped, NT_basis),
  ]

  for name, F, NT in cases:
    tau_task = J.T @ F
    tau_null = NT @ TAU_0
    tau_total = tau_task + tau_null
    # 动力学偏置已补偿的瞬时增量模型
    a_task = J @ np.linalg.solve(M, tau_task)
    a_null = J @ np.linalg.solve(M, tau_null)
    a_total = a_task + a_null
    print(f"\n方案: {name}")
    print( "任务加速度实现率:", f"{100 * (a_cmd @ a_task):.3f}%")
    print("零空间加速度泄漏:", f"{np.linalg.norm(a_null):.9e} m/s^2")
    print( "总加速度误差:",f"{np.linalg.norm(a_total-a_cmd):.9e} m/s^2")
    print("总力矩范数:", f"{np.linalg.norm(tau_total):.6f} Nm")

def main():
  parser = argparse.ArgumentParser(description="OSC正则化对照")
  parser.add_argument("reference")
  parser.add_argument("gazebo")
  parser.add_argument("--case", choices=["all", "force", "projector", "split"], default="all")
  args = parser.parse_args()
  cases = {
    "force": analyze_force,
    "projector": analyze_projector,
    "split": analyze_split,
  }
  selected = cases if args.case == "all" else {args.case: cases[args.case]}
  for name, analyze in selected.items():
    print(f"\n========== 实验主题: {name} ==========")
    analyze("MoveIt参考轨迹", args.reference)
    analyze("Gazebo实际轨迹", args.gazebo)

if __name__ == "__main__":
  main()


