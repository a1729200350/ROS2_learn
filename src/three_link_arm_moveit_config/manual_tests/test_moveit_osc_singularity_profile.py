import sys
import numpy as np
from osc_analysis_common import load_joint_trajectory, calculate_matrices
from three_link_arm_kinematics.operational_space_controller import (OperationalSpaceController,)
from three_link_arm_kinematics.operational_space_dynamics import (OperationalSpaceDynamics,)
def analyze_trajectory(label, filename):
  t, q = load_joint_trajectory(filename)

  controller = OperationalSpaceController()
  osc = OperationalSpaceDynamics()
  sigma_values = []
  condition_values = []
  lambda_max_values = []

  for qk in q:
    J, M, A = calculate_matrices(qk, controller, osc)

    # 雅可比奇异值与条件数。
    singular_values = np.linalg.svd(J, compute_uv=False)
    sigma_min = singular_values[-1]
    condition_J = (
      singular_values[0] / sigma_min
      if sigma_min > 0
      else np.inf
    )

    # 沿用原实验的阈值，计算Lambda最大特征值。
    eig_A = np.linalg.eigvalsh(A)
    lambda_max = (
      1.0 / eig_A[0]
      if eig_A[0] > 1e-12
      else np.inf
    )

    sigma_values.append(sigma_min)
    condition_values.append(condition_J)
    lambda_max_values.append(lambda_max)

  sigma_values = np.asarray(sigma_values)
  condition_values = np.asarray(condition_values)
  lambda_max_values = np.asarray(lambda_max_values)
  idx_sigma = int(np.argmin(sigma_values))
  idx_lambda = int(np.argmax(lambda_max_values))
  print(f"\n========== {label} ==========")
  print("采样点数:", len(t))
  print("\n最小 Jacobian 奇异值:")
  print(f"  数值: {sigma_values[idx_sigma]:.9e}")
  print(f"  时间: {t[idx_sigma]:.4f} s")
  print("\n最大 Jacobian 条件数:")
  print(f"  {np.max(condition_values):.6f}")
  print("\nLambda 最大特征值的全程最大值:")
  print(f"  数值: {lambda_max_values[idx_lambda]:.6f}")
  print(f"  时间: {t[idx_lambda]:.4f} s")
  print("\n最接近奇异点时的关节角:")
  print(q[idx_sigma])


def main():
  if len(sys.argv) != 3:
    raise SystemExit( "用法: python test_moveit_osc_singularity_profile.py " "参考轨迹.npz Gazebo轨迹.npz")
  analyze_trajectory("MoveIt参考轨迹", sys.argv[1])
  analyze_trajectory("Gazebo实际轨迹", sys.argv[2])


if __name__ == "__main__":
  main()
