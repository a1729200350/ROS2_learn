from pathlib import Path
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import rosbag2_py
from rclpy.serialization import deserialize_message
from control_msgs.msg import JointTrajectoryControllerState
TOPIC = "/joint_trajectory_controller/controller_state"
def main():
  if len(sys.argv) != 2:
    print("用法: python analyze_moveit_jtc_bag.py BAG目录")
    sys.exit(1)
  bag_path = Path(sys.argv[1]).expanduser().resolve()
  if not bag_path.is_dir():
    raise FileNotFoundError(bag_path)
  reader = rosbag2_py.SequentialReader()
  storage_options = rosbag2_py.StorageOptions(uri=str(bag_path),storage_id="mcap",)
  converter_options = rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr",)
  reader.open(storage_options, converter_options)
  times = []
  reference_positions = []
  feedback_positions = []
  reference_velocities = []
  joint_names = None

  # 逐帧读取 JTC 控制状态
  while reader.has_next():
    topic, raw_data, _ = reader.read_next()
    if topic != TOPIC:
      continue
    msg = deserialize_message(raw_data,JointTrajectoryControllerState,)
    names = list(msg.joint_names)
    if joint_names is None:
      joint_names = names
    elif names != joint_names:
      raise RuntimeError("录制期间关节顺序发生变化")
    n = len(joint_names)
    if (
      len(msg.reference.positions) != n
      or len(msg.feedback.positions) != n
    ):
      continue
    t = ( msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

    times.append(t)
    reference_positions.append(list(msg.reference.positions))
    feedback_positions.append(list(msg.feedback.positions))

    if len(msg.reference.velocities) == n:
      reference_velocities.append(list(msg.reference.velocities))
    else:
      reference_velocities.append([0.0] * n)

  if len(times) < 2:
    raise RuntimeError("有效控制器状态消息不足")

  t = np.asarray(times, dtype=float)
  q_ref = np.asarray(reference_positions, dtype=float)
  q_actual = np.asarray(feedback_positions, dtype=float)
  v_ref = np.asarray(reference_velocities, dtype=float)

  # JTC 的位置跟踪误差
  e = q_ref - q_actual

  # 时间轴采用 Gazebo 仿真时间
  if np.any(np.diff(t) <= 0):
    raise RuntimeError("检测到仿真时间未递增，请检查录包时间戳")

  t = t - t[0]

  # 根据期望速度及期望位置变化识别运动
  position_step = np.zeros_like(q_ref)
  position_step[1:] = np.abs(np.diff(q_ref, axis=0))
  moving = (np.max(np.abs(v_ref), axis=1) > 1e-4)| (np.max(position_step, axis=1) > 1e-7)
  active_indices = np.flatnonzero(moving)

  if len(active_indices) == 0:
    raise RuntimeError("没有检测到期望关节运动，请确认录包包含 Execute 阶段")
  # 运动前后各保留约 0.2s 的数据
  dt = np.median(np.diff(t))
  padding = int(np.ceil(0.2 / dt))
  begin = max(0, active_indices[0] - padding)
  end = min(len(t), active_indices[-1] + padding + 1)
  t_motion = t[begin:end] - t[begin]
  q_ref_motion = q_ref[begin:end]
  q_actual_motion = q_actual[begin:end]
  e_motion = e[begin:end]

  # 估计参考轨迹与反馈轨迹之间的等效滞后
  v_ref_motion = v_ref[begin:end]
  print("\n========== 等效跟踪滞后分析 ==========")
  for i, name in enumerate(joint_names):
    v = v_ref_motion[:, i]
    err = e_motion[:, i]
    # 仅采用速度足够大的样本，避免静止区间干扰
    threshold = max(1e-4, 0.1 * np.max(np.abs(v)))
    mask = np.abs(v) > threshold
    if np.count_nonzero(mask) < 10:
      print(f"{name}: 有效运动样本不足")
      continue
    # 最小二乘拟合 e ≈ tau * v
    tau = np.dot(v[mask], err[mask]) / np.dot(v[mask], v[mask])
    predicted_error = tau * v[mask]
    residual_rms = np.sqrt(np.mean((err[mask] - predicted_error) ** 2))
    print(f"{name}:")
    print(f"  等效滞后: {tau * 1000:.3f} ms")
    print(f"  拟合残差 RMS: {residual_rms:.6e} rad")


  # 对检测到的运动区间计算 RMS 和最大绝对误差
  rms = np.sqrt(np.mean(e_motion ** 2, axis=0))
  max_error = np.max(np.abs(e_motion), axis=0)

  print("\n========== JTC 轨迹跟踪分析 ==========")
  print(f"全部有效样本数: {len(t)}")
  print(f"运动区间样本数: {len(t_motion)}")
  print(f"运动分析时长: {t_motion[-1]:.3f} s")

  for i, name in enumerate(joint_names):
    print(f"\n{name}")
    print(f"  RMS:      {rms[i]:.9e} rad")
    print(f"  最大误差: {max_error[i]:.9e} rad")
    print(f"  RMS:      {np.degrees(rms[i]):.9e} deg")

  output_dir = bag_path.parent / (bag_path.name + "_analysis")
  output_dir.mkdir(parents=True, exist_ok=True)

  # 图1：期望关节位置与实际关节位置
  fig1, axes1 = plt.subplots(
    len(joint_names), 1,
    figsize=(11, 8),
    sharex=True,
    squeeze=False,
  )

  for i, name in enumerate(joint_names):
    ax = axes1[i, 0]
    ax.plot(t_motion, q_ref_motion[:, i], label="Reference", linewidth=1.5)
    ax.plot(t_motion, q_actual_motion[:, i], "--", label="Feedback", linewidth=1.1)
    ax.set_ylabel(f"{name} (rad)")
    ax.grid(True)
    ax.legend()

  axes1[-1, 0].set_xlabel("Simulation time (s)")
  fig1.suptitle("JTC Position Tracking")
  fig1.tight_layout()
  fig1.savefig(output_dir / "joint_positions.png", dpi=160)
  plt.close(fig1)

  # 图2：位置跟踪误差
  fig2, axes2 = plt.subplots(
    len(joint_names), 1,
    figsize=(11, 8),
    sharex=True,
    squeeze=False,
  )

  for i, name in enumerate(joint_names):
    ax = axes2[i, 0]
    ax.plot(t_motion, e_motion[:, i])
    ax.set_ylabel(f"{name} error (rad)")
    ax.grid(True)

  axes2[-1, 0].set_xlabel("Simulation time (s)")
  fig2.suptitle("JTC Position Tracking Error")
  fig2.tight_layout()
  fig2.savefig(output_dir / "joint_errors.png", dpi=160)
  plt.close(fig2)

  print(f"\n图像保存位置: {output_dir}")


if __name__ == "__main__":
  main()
