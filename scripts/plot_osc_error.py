from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
import rosbag2_py
from rclpy.serialization import deserialize_message
from std_msgs.msg import Float64MultiArray
bag_path = Path.home() / "ros2_ws/osc_records/baseline_01"
reader = rosbag2_py.SequentialReader()

reader.open(
    rosbag2_py.StorageOptions( uri=str(bag_path),storage_id="mcap",),
    rosbag2_py.ConverterOptions( input_serialization_format="cdr", output_serialization_format="cdr",),
)

timestamps = []
trajectory_times = []
errors_x = []
errors_y = []
torques = []
joint_velocities = []
while reader.has_next():
    topic, data, timestamp = reader.read_next()
    if topic != "/osc_tracking":
        continue
    msg = deserialize_message(data, Float64MultiArray)
    if len(msg.data) < 16:
        continue
    timestamps.append(timestamp * 1e-9)
    trajectory_times.append(msg.data[0])
    errors_x.append(msg.data[5] * 1000)
    errors_y.append(msg.data[6] * 1000)
    # data[13:16] 是 OSC 的三个关节力矩指令
    torques.append(msg.data[13:16])
    # data[10:13] 对应三个关节速度，单位为 rad/s。
    joint_velocities.append(msg.data[10:13])
timestamps = np.array(timestamps)
trajectory_times = np.array(trajectory_times)
errors_x = np.array(errors_x)
errors_y = np.array(errors_y)
if len(timestamps) == 0:
    raise RuntimeError("没有读取到 OSC 跟踪数据")
# 将录制时间转换为相对秒数
t = timestamps - timestamps[0]
# 找到轨迹真正开始的时刻
active = np.flatnonzero(trajectory_times > 0)
if len(active) == 0:
    raise RuntimeError("没有检测到轨迹开始，请确认录制期间调用过 /osc_start")
start_time = t[active[0]]
# 显示开始前 0.5 秒以及开始后 5 秒
mask = (t >= start_time - 0.5) & (t <= start_time + 5.0)
t_plot = t[mask] - start_time
plt.figure(figsize=(10, 5))
plt.plot(t_plot, errors_x[mask], label="X error")
plt.plot(t_plot, errors_y[mask], label="Y error")
plt.axhline(0, color="gray", linestyle="--", linewidth=0.8)
plt.axvline(0, color="gray", linestyle=":", linewidth=0.8)
plt.xlabel("Time relative to trajectory start (s)")
plt.ylabel("Position error (mm)")
plt.title("OSC End-Effector Tracking Error")
plt.legend()
plt.grid(True)
plt.tight_layout()
output_path = bag_path.parent / "baseline_01_error.png"
plt.savefig(output_path, dpi=180)
print("读取 OSC 消息数：", len(timestamps))
print("轨迹起始录制时间：", start_time)
print("绘图保存位置：", output_path)

torques = np.array(torques)
plt.figure(figsize=(10, 5))
for i in range(3):
    plt.plot(
        t_plot,
        torques[mask, i],
        label=f"Joint {i+1} torque"
    )
plt.xlabel("Time relative to trajectory start (s)")
plt.ylabel("Torque (Nm)")
plt.title("OSC Joint Torque")
plt.legend()
plt.grid(True)
plt.tight_layout()
output_path = bag_path.parent / "baseline_01_torque.png"
plt.savefig(output_path, dpi=180)
print("力矩曲线保存位置：", output_path)

joint_velocities = np.array(joint_velocities)
plt.figure(figsize=(10, 5))
for i in range(3):
    plt.plot(
        t_plot,
        joint_velocities[mask, i],
        label=f"Joint {i+1} velocity"
    )
plt.axhline(0, color="gray", linestyle="--", linewidth=0.8)
plt.axvline(0, color="gray", linestyle=":", linewidth=0.8)
plt.xlabel("Time relative to trajectory start (s)")
plt.ylabel("Joint velocity (rad/s)")
plt.title("OSC Joint Velocity")
plt.legend()
plt.grid(True)
plt.tight_layout()
output_path = bag_path.parent / "baseline_01_velocity.png"
plt.savefig(output_path, dpi=180)
print("关节速度曲线保存位置：", output_path)