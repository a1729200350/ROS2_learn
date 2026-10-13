from pathlib import Path
import sys
import numpy as np
# 复用我们已经验证通过的运动学函数
from convert_moveit_to_osc import ( fk, jacobian,calculate_jdot_dq,)
DT = 0.01  # 离线采样周期：10 ms，100 Hz
def interpolate(t_query, t, q, dq, ddq):
  """在任意轨迹时刻计算 q, dq, ddq。"""
  if t_query < t[0] or t_query > t[-1]:
    raise ValueError("查询时间超出轨迹范围")
  # 选择当前时间所在区间
  k = np.searchsorted(t, t_query, side="right") - 1
  k = min(k, len(t) - 2)
  h = t[k + 1] - t[k]
  s = (t_query - t[k]) / h
  q0, q1 = q[k], q[k + 1]
  v0, v1 = dq[k], dq[k + 1]
  acc0, acc1 = ddq[k], ddq[k + 1]
  delta = q1 - q0
  # 五次 Hermite 多项式系数
  a0 = q0
  a1 = h * v0
  a2 = 0.5 * h**2 * acc0
  a3 = 10 * delta - 6 * h * v0 - 4 * h * v1 - 1.5 * h**2 * acc0 + 0.5 * h**2 * acc1
  a4 = -15 * delta + 8 * h * v0 + 7 * h * v1 + 1.5 * h**2 * acc0  - h**2 * acc1
  a5 =  6 * delta - 3 * h * (v0 + v1) - 0.5 * h**2 * acc0 + 0.5 * h**2 * acc1
  # 插值位置
  qi =  a0 + a1*s + a2*s**2 + a3*s**3 + a4*s**4 + a5*s**5
  # 一阶时间导数
  dqi = ( a1 + 2*a2*s + 3*a3*s**2 + 4*a4*s**3 + 5*a5*s**4 ) / h

  # 二阶时间导数
  ddqi = ( 2*a2 + 6*a3*s + 12*a4*s**2 + 20*a5*s**3) / h**2
  return qi, dqi, ddqi


def main():
  if len(sys.argv) != 2:
    print("用法: python interpolate_moveit_trajectory.py 轨迹_osc.npz")
    sys.exit(1)
  source = Path(sys.argv[1]).expanduser().resolve()
  with np.load(source, allow_pickle=False) as data:
    t = data["t"]
    q = data["q"]
    dq = data["dq"]
    ddq = data["ddq"]
    joint_names = data["joint_names"]

  n = len(t)
  if n < 2:
    raise ValueError("轨迹点数量不足")
  if (
    q.shape != (n, 3)
    or dq.shape != (n, 3)
    or ddq.shape != (n, 3)
  ):
    raise ValueError("轨迹数组形状错误")
  if not all(
    np.all(np.isfinite(a))
    for a in (t, q, dq, ddq)
  ):
    raise ValueError("轨迹包含非法数值")
  if np.any(np.diff(t) <= 0):
    raise ValueError("轨迹时间必须严格递增")

  # 一：验证插值是否准确经过原始节点
  reconstructed = [ interpolate(float(tk), t, q, dq, ddq) for tk in t]
  q_nodes = np.array([v[0] for v in reconstructed])
  dq_nodes = np.array([v[1] for v in reconstructed])
  ddq_nodes = np.array([v[2] for v in reconstructed])
  q_error = np.max(np.abs(q_nodes - q))
  dq_error = np.max(np.abs(dq_nodes - dq))
  ddq_error = np.max(np.abs(ddq_nodes - ddq))
  print("========== 原始节点重建验证 ==========")
  print(f"位置最大误差: {q_error:.6e} rad")
  print(f"速度最大误差: {dq_error:.6e} rad/s")
  print(f"加速度最大误差: {ddq_error:.6e} rad/s^2")
  if max(q_error, dq_error, ddq_error) > 1e-7:
    raise RuntimeError("原始节点重建误差过大")

  # 二：按100Hz生成连续参考的离散采样
  t_dense = np.append( np.arange(t[0], t[-1], DT), t[-1],)
  samples = [ interpolate(float(tk), t, q, dq, ddq) for tk in t_dense]
  q_dense = np.array([v[0] for v in samples])
  dq_dense = np.array([v[1] for v in samples])
  ddq_dense = np.array([v[2] for v in samples])

  # 三：转换为任务空间参考
  N = len(t_dense)
  x = np.zeros((N, 2))
  dx = np.zeros((N, 2))
  ddx = np.zeros((N, 2))
  sigma_min = np.zeros(N)

  for k in range(N):
    qk = q_dense[k]
    dqk = dq_dense[k]
    ddqk = ddq_dense[k]
    J = jacobian(qk)
    x[k] = fk(qk)
    dx[k] = J @ dqk
    ddx[k] = ( J @ ddqk  + calculate_jdot_dq(qk, dqk))
    sigma_min[k] = np.linalg.svd( J, compute_uv=False)[-1]
  # 四：保存结果
  output = source.with_name( source.stem + "_interp_100hz.npz")
  np.savez_compressed(
    output,
    t=t_dense,
    q=q_dense,
    dq=dq_dense,
    ddq=ddq_dense,
    x=x,
    dx=dx,
    ddx=ddx,
    sigma_min=sigma_min,
    joint_names=joint_names,
    task_frame="arm_base",
  )

  index = int(np.argmin(sigma_min))
  print("\n========== 插值结果 ==========")
  print("原始轨迹点数:", n)
  print("插值采样点数:", N)
  print("采样周期:", DT, "s")
  print("总时长:", t_dense[-1], "s")
  print("\n数组形状:")
  print("q:", q_dense.shape)
  print("dq:", dq_dense.shape)
  print("ddq:", ddq_dense.shape)
  print("x:", x.shape)
  print("dx:", dx.shape)
  print("ddx:", ddx.shape)
  print("\n全轨迹最小奇异值:", sigma_min[index])
  print("出现时间:", t_dense[index], "s")
  print("\n保存位置:")
  print(output)


if __name__ == "__main__":
  main()
