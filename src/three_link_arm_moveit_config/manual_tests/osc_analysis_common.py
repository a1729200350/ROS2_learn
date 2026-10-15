"""轨迹与操作空间矩阵的离线分析工具。"""
from pathlib import Path
import numpy as np

def load_joint_trajectory(filename):
  """读取分析所需的 t、q，兼容参考轨迹和Gazebo记录。"""
  path = Path(filename).expanduser().resolve()
  with np.load(path, allow_pickle=False) as data:
    t = data["t"].copy()
    q = data["q"].copy()
  if t.ndim != 1 or q.shape != (len(t), 3):
    raise ValueError("轨迹应包含一维t和形状为(N, 3)的q")
  if len(t) == 0:
    raise ValueError("轨迹不能为空")
  if not np.all(np.isfinite(t)) or not np.all(np.isfinite(q)):
    raise ValueError("轨迹包含非有限数值")
  return t, q

def calculate_matrices(q, controller, osc):
  """计算 J、M，以及对称化的 A = J M^-1 J.T。"""
  J = controller.kinematics.jacobian(q)
  M = osc.dynamics.calculate_mass_matrix(q)
  A = J @ np.linalg.solve(M, J.T)
  A = 0.5 * (A + A.T)
  return J, M, A

def find_worst_configuration(t, q, controller, osc, *, require_positive=True):
  """按A的最小特征值寻找最敏感姿态；可选择允许奇异姿态。"""
  worst = None
  for k, qk in enumerate(q):
    J, M, A = calculate_matrices(qk, controller, osc)
    alpha_min = float(np.linalg.eigvalsh(A)[0])
    if require_positive and alpha_min <= 0:
      raise ValueError(f"A不是正定矩阵，t={t[k]:.4f}")
    if worst is None or alpha_min < worst["alpha_min"]:
      worst = {
        "t": float(t[k]),
        "q": qk.copy(),
        "J": J,
        "M": M,
        "A": A,
        "alpha_min": alpha_min,
      }
  if worst is None:
    raise ValueError("轨迹不能为空")
  return worst


