from pathlib import Path
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import rosbag2_py
from rclpy.serialization import deserialize_message
from std_msgs.msg import Float64MultiArray
TOPIC = "/osc_tracking"
L = np.array([0.5, 0.4, 0.4])
def jacobian(q):
  theta = np.cumsum(q)
  J = np.zeros((2, 3))
  for i in range(3):
    J[0, i] = -np.sum(L[i:] * np.sin(theta[i:]))
    J[1, i] = np.sum(L[i:] * np.cos(theta[i:]))
  return J
def main():
  if len(sys.argv) != 3:
    raise SystemExit( "用法: python analyze_moveit_osc_bag.py " "BAG目录 参考轨迹.npz")
  bag_path = Path(sys.argv[1]).expanduser().resolve()
  ref_path = Path(sys.argv[2]).expanduser().resolve()
  if not bag_path.is_dir() or not ref_path.is_file():
    raise FileNotFoundError("录包目录或参考轨迹文件不存在")
  with np.load(ref_path, allow_pickle=False) as d:
    t_ref = d["t"].copy()
    q_ref = d["q"].copy()
    names = d["joint_names"].tolist()
  if names != ["joint1", "joint2", "joint3"]:
    raise ValueError("参考轨迹关节顺序错误")
  T = float(t_ref[-1])
  reader = rosbag2_py.SequentialReader()
  reader.open( rosbag2_py.StorageOptions( uri=str(bag_path), storage_id="mcap"),
    rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr",),
  )
  rows = []
  bad_rows = 0
  while reader.has_next():
    topic, raw, _ = reader.read_next()
    if topic != TOPIC:
      continue
    msg = deserialize_message(raw, Float64MultiArray)
    # [elapsed, xd(2), x(2), ex(2),
    #  q(3), dq(3), tau(3)] 共16项
    data = np.asarray(msg.data, dtype=float)
    if data.shape != (16,) or not np.all(np.isfinite(data)):
      bad_rows += 1
      continue
    rows.append(data)
  if not rows:
    raise RuntimeError("没有有效的 /osc_tracking 数据")
  a = np.asarray(rows)
  elapsed = a[:, 0]
  # 轨迹启动前 elapsed=0
  started = elapsed > 1e-6
  if not np.any(started):
    raise RuntimeError( "没有检测到轨迹执行，检查 /osc_start 是否成功")
  print("\n========== 轨迹执行完整性 ==========")
  print("有效跟踪消息数:", len(a))
  print("异常消息数:", bad_rows)
  print("启动前样本数:", np.count_nonzero(~started))
  print(f"轨迹最大时间: {np.max(elapsed):.3f} s")
  print(f"参考轨迹时长: {T:.3f} s")
  if np.max(elapsed) < T - 0.1:
    raise RuntimeError("录包没有覆盖完整轨迹运动阶段")
  # 仅分析轨迹运动阶段
  mask = started & (elapsed <= T)
  m = a[mask]
  if len(m) < 10:
    raise RuntimeError("运动阶段有效数据不足")
  t = m[:, 0]
  xd = m[:, 1:3]
  x = m[:, 3:5]
  ex = m[:, 5:7]
  q = m[:, 7:10]
  dq = m[:, 10:13]
  tau = m[:, 13:16]
  # 还原 MoveIt 期望关节位置
  qd = np.column_stack([np.interp(t, t_ref, q_ref[:, i]) for i in range(3) ])
  eq = qd - q
  rms_x = np.sqrt(np.mean(ex**2, axis=0))
  max_x = np.max(np.abs(ex), axis=0)
  rms_q = np.sqrt(np.mean(eq**2, axis=0))
  max_q = np.max(np.abs(eq), axis=0)
  max_tau = np.max(np.abs(tau), axis=0)
  sigma_min = np.array([ np.linalg.svd(jacobian(qk), compute_uv=False)[-1] for qk in q])
  print("\n========== Gazebo OSC 跟踪分析 ==========")
  print("运动阶段样本数:", len(t))
  print(f"运动阶段覆盖: {t[0]:.3f} ~ {t[-1]:.3f} s")
  print("\n末端 RMS 误差 [mm]:")
  print(rms_x * 1000)
  print("\n末端最大绝对误差 [mm]:")
  print(max_x * 1000)
  print("\n关节 RMS 误差 [rad]:")
  print(rms_q)
  print("\n关节最大绝对误差 [rad]:")
  print(max_q)
  print("\n实际最大绝对输出力矩 [Nm]:")
  print(max_tau)
  print("\n实际轨迹最小奇异值:")
  print(np.min(sigma_min))
  print("出现时间:", t[np.argmin(sigma_min)], "s")
  print("\n记录到的力矩达到限制附近的样本数:")
  print(int(np.count_nonzero( np.any(np.abs(tau) >= 9.999, axis=1))))
  consistency = np.max(np.abs(ex - (xd - x)))
  print("\n跟踪误差字段一致性:", consistency)
  # 分析轨迹结束后的终点保持
  hold = a[elapsed > T]
  if len(hold):
    final_error = hold[-1, 5:7] * 1000
    print("\n录制结束前最后一帧保持误差 [mm]:")
    print(final_error)
  else:
    print("\n没有记录到轨迹结束后的保持数据")
  out = bag_path.parent / (bag_path.name + "_analysis")
  out.mkdir(parents=True, exist_ok=True)
  np.savez_compressed(
    out / "gazebo_tracking.npz",
    t=t, xd=xd, x=x, ex=ex,
    qd=qd, q=q, eq=eq,
    dq=dq, tau=tau,
    sigma_min=sigma_min,
  )

  def plot_data(values, labels, ylabel, filename):
    fig, ax = plt.subplots(figsize=(11, 4))
    for i, label in enumerate(labels):
      ax.plot(t, values[:, i], label=label)
    ax.set_xlabel("Trajectory time (s)")
    ax.set_ylabel(ylabel)
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out / filename, dpi=160)
    plt.close(fig)

  plot_data( ex * 1000, ["X error", "Y error"], "Task error (mm)", "gazebo_task_error.png",)
  plot_data(eq,["joint1", "joint2", "joint3"],"Joint error (rad)","gazebo_joint_error.png",)
  plot_data( tau, ["joint1", "joint2", "joint3"], "Applied torque (Nm)","gazebo_torque.png",)
  print("\n分析结果保存至:")
  print(out)


if __name__ == "__main__":
  main()
