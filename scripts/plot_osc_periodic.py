"""绘制 OSC 周期往返轨迹的完整跟踪结果（从 rosbag2 MCAP 读取）。"""
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
import rosbag2_py
from rclpy.serialization import deserialize_message
from std_msgs.msg import Float64MultiArray
bag_path = Path.home() / "ros2_ws/osc_records/periodic_01"
reader = rosbag2_py.SequentialReader()
reader.open(
  rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="mcap"),
  rosbag2_py.ConverterOptions(
      input_serialization_format="cdr",
      output_serialization_format="cdr",
  ),
)

trajectory_times = []
positions_desired = []
positions_actual = []
errors = []
torques = []
joint_velocities = []

while reader.has_next():
  topic, data, timestamp = reader.read_next()
  if topic != "/osc_tracking":
    continue
  msg = deserialize_message(data, Float64MultiArray)
  if len(msg.data) < 16 or msg.data[0] <= 0.0:
    continue
  # /osc_tracking: [t, x_d(2), x(2), error(2), q(3), q_dot(3), tau(3)]
  trajectory_times.append(msg.data[0])
  positions_desired.append(msg.data[1:3])
  positions_actual.append(msg.data[3:5])
  errors.append(msg.data[5:7])
  joint_velocities.append(msg.data[10:13])
  torques.append(msg.data[13:16])
if not trajectory_times:
  raise RuntimeError("没有找到 /osc_start 后的 /osc_tracking 数据")
t = np.asarray(trajectory_times, dtype=float)
x_d = np.asarray(positions_desired, dtype=float)
x = np.asarray(positions_actual, dtype=float)
e_mm = np.asarray(errors, dtype=float) * 1000.0
q_dot = np.asarray(joint_velocities, dtype=float)
tau = np.asarray(torques, dtype=float)

print(f"有效采样数: {len(t)}")
print(f"轨迹时间: {t[0]:.3f} ~ {t[-1]:.3f} s")
print("全程 RMS 误差(mm):", np.sqrt(np.mean(e_mm ** 2, axis=0)))
print("最大绝对误差(mm):", np.max(np.abs(e_mm), axis=0))
print("最大绝对力矩(Nm):", np.max(np.abs(tau), axis=0))


# 图 1：完整周期轨迹的期望位置与实际位置
plt.figure(figsize=(12, 5))
plt.plot(t, x_d[:, 0], label="X desired")
plt.plot(t, x[:, 0], "--", label="X actual")
plt.plot(t, x_d[:, 1], label="Y desired")
plt.plot(t, x[:, 1], "--", label="Y actual")
plt.xlabel("Trajectory time (s)")
plt.ylabel("End-effector position (m)")
plt.title("OSC Periodic Trajectory: Desired vs Actual")
plt.legend()
plt.grid(True)
plt.tight_layout()
output_path = bag_path.parent / "periodic_01_tracking.png"
plt.savefig(output_path, dpi=180)
plt.close()
print("期望/实际位置曲线:", output_path)


# 图 2：完整运动过程中的末端位置误差
plt.figure(figsize=(12, 5))
plt.plot(t, e_mm[:, 0], label="X error")
plt.plot(t, e_mm[:, 1], label="Y error")
plt.axhline(0, linestyle="--", linewidth=0.8)
plt.xlabel("Trajectory time (s)")
plt.ylabel("Position error (mm)")
plt.title("OSC Periodic Trajectory: Tracking Error")
plt.legend()
plt.grid(True)
plt.tight_layout()
output_path = bag_path.parent / "periodic_01_error.png"
plt.savefig(output_path, dpi=180)
plt.close()
print("误差曲线:", output_path)


# 图 3：三个关节的 OSC 力矩指令（已限幅）
plt.figure(figsize=(12, 5))
for i in range(3):
  plt.plot(t, tau[:, i], label=f"Joint {i+1} torque")
plt.xlabel("Trajectory time (s)")
plt.ylabel("Commanded torque (Nm)")
plt.title("OSC Periodic Trajectory: Joint Torque Commands")
plt.legend()
plt.grid(True)
plt.tight_layout()
output_path = bag_path.parent / "periodic_01_torque.png"
plt.savefig(output_path, dpi=180)
plt.close()
print("力矩曲线:", output_path)


# 图 4：三个关节的速度
plt.figure(figsize=(12, 5))
for i in range(3):
  plt.plot(t, q_dot[:, i], label=f"Joint {i+1} velocity")
plt.axhline(0, linestyle="--", linewidth=0.8)
plt.xlabel("Trajectory time (s)")
plt.ylabel("Joint velocity (rad/s)")
plt.title("OSC Periodic Trajectory: Joint Velocity")
plt.legend()
plt.grid(True)
plt.tight_layout()
output_path = bag_path.parent / "periodic_01_velocity.png"
plt.savefig(output_path, dpi=180)
plt.close()
print("速度曲线:", output_path)
