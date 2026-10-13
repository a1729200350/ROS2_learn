from pathlib import Path
import sys
import numpy as np
# 三连杆长度，单位 m
L = np.array([0.5, 0.4, 0.4])
EXPECTED_JOINTS = ["joint1", "joint2", "joint3"]
def fk(q):
  theta = np.cumsum(q)
  return np.array([
    np.sum(L * np.cos(theta)),
    np.sum(L * np.sin(theta)),
  ])

def jacobian(q):
  theta = np.cumsum(q)
  J = np.zeros((2, 3))
  for i in range(3):
    J[0, i] = -np.sum( L[i:] * np.sin(theta[i:]))
    J[1, i] = np.sum( L[i:] * np.cos(theta[i:]))
  return J

def calculate_jdot_dq(q, dq):
  theta = np.cumsum(q)
  omega = np.cumsum(dq)
  return np.array([
    -np.sum(L * np.cos(theta) * omega**2),
    -np.sum(L * np.sin(theta) * omega**2),
  ])

def main():
  if len(sys.argv) != 2:
    print("用法: python convert_moveit_to_osc.py 轨迹.npz")
    sys.exit(1)
  source = Path(sys.argv[1]).expanduser().resolve()
  if not source.is_file():
    raise FileNotFoundError(source)
  # 读取 MoveIt 原始轨迹
  with np.load(source, allow_pickle=False) as data:
    t = data["t"]
    q = data["q"]
    dq = data["dq"]
    ddq = data["ddq"]
    names = data["joint_names"].tolist()
  N = len(t)
  # 检查轨迹结构
  if names != EXPECTED_JOINTS:
    raise ValueError(f"关节顺序不符合预期: {names}")
  if N < 2:
    raise ValueError("轨迹点数量不足")
  if any(a.shape != (N, 3) for a in (q, dq, ddq)):
    raise ValueError("关节轨迹数组形状错误")
  if not all(np.all(np.isfinite(a)) for a in (t, q, dq, ddq)):
    raise ValueError("轨迹包含 NaN 或无穷大")
  if np.any(np.diff(t) <= 0):
    raise ValueError("轨迹时间必须严格递增")

  # 准备 OSC 轨迹数组
  x = np.zeros((N, 2))
  dx = np.zeros((N, 2))
  ddx = np.zeros((N, 2))
  jdot_dq = np.zeros((N, 2))
  sigma_min = np.zeros(N)
  max_jdot_validation_error = 0.0
  eps = 1e-6

  for k in range(N):
    qk = q[k]
    dqk = dq[k]
    ddqk = ddq[k]
    J = jacobian(qk)
    # 位置
    x[k] = fk(qk)
    # 速度
    dx[k] = J @ dqk
    # Jdot * dq
    jdot_dq[k] = calculate_jdot_dq(qk, dqk)
    # 加速度
    ddx[k] = J @ ddqk + jdot_dq[k]
    # 奇异值分析
    sigma = np.linalg.svd( J, compute_uv=False)
    sigma_min[k] = sigma[-1]
    # 数值差分验证 Jdot * dq
    J_plus = jacobian(qk + eps * dqk)
    J_minus = jacobian(qk - eps * dqk)
    numeric = ( (J_plus - J_minus) / (2 * eps)) @ dqk
    error = np.linalg.norm( jdot_dq[k] - numeric)
    max_jdot_validation_error = max( max_jdot_validation_error, error)

  # 保存新文件，不修改原始 MoveIt 轨迹
  output = source.with_name(source.stem + "_osc.npz")
  np.savez_compressed(
    output,
    t=t,
    q=q,
    dq=dq,
    ddq=ddq,
    x=x,
    dx=dx,
    ddx=ddx,
    jdot_dq=jdot_dq,
    sigma_min=sigma_min,
    joint_names=np.array(names),
    task_frame="arm_base",
  )

  print("\n========== MoveIt -> OSC 全轨迹转换 ==========")
  print("轨迹点数:", N)
  print(f"轨迹总时长: {t[-1]:.6f} s")
  print("\n数组形状:")
  print("q:", q.shape)
  print("x:", x.shape)
  print("dx:", dx.shape)
  print("ddx:", ddx.shape)
  print("\n代表性轨迹点:")

  for k in sorted({0, N // 2, N - 1}):
    print(f"\n点 {k}, t={t[k]:.6f} s")
    print("  x   =", x[k])
    print("  dx  =", dx[k])
    print("  ddx =", ddx[k])
    print("  sigma_min =", sigma_min[k])
  min_index = int(np.argmin(sigma_min))
  print("\n========== 数值验证 ==========")
  print( "Jdot*dq 最大数值差分误差:", f"{max_jdot_validation_error:.6e}")
  print( "全轨迹最小 sigma_min:", f"{sigma_min[min_index]:.6e}")
  print( "最小奇异值出现时间:", f"{t[min_index]:.6f} s")
  print("\n转换结果保存至:")
  print(output)


if __name__ == "__main__":
  main()
