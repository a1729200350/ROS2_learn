from datetime import datetime
from pathlib import Path
import time
import numpy as np
import rclpy
from moveit_msgs.msg import DisplayTrajectory
from rclpy.qos import (QoSProfile,ReliabilityPolicy, DurabilityPolicy,)
TOPIC = "/display_planned_path"
OUTPUT_DIR = ( Path.home() / "ros2_ws" / "moveit_trajectory_records")
def main():
  rclpy.init()
  node = rclpy.create_node("capture_moveit_trajectory")
  received = []
  def callback(msg):
    if received or not msg.trajectory:
      return
    jt = msg.trajectory[0].joint_trajectory
    points = jt.points
    names = list(jt.joint_names)
    n = len(names)
    if not points or n == 0:
      return
    if any(
      len(p.positions) != n
      or len(p.velocities) != n
      or len(p.accelerations) != n
      for p in points
    ):
      print("轨迹存在缺失字段，跳过")
      return
    t = np.array([
      p.time_from_start.sec
      + p.time_from_start.nanosec * 1e-9
      for p in points
    ])
    q = np.array([p.positions for p in points])
    dq = np.array([p.velocities for p in points])
    ddq = np.array([p.accelerations for p in points])
    if not np.all(np.isfinite(np.concatenate([t[:, None], q, dq, ddq], axis=1))):
      print("轨迹存在非有限数值，跳过")
      return
    if np.any(np.diff(t) <= 0):
      print("轨迹时间没有严格递增，跳过")
      return

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    filename = (
      "moveit_trajectory_"
      + datetime.now().strftime("%Y%m%d_%H%M%S")
      + ".npz"
    )

    path = OUTPUT_DIR / filename
    np.savez_compressed(
      path,
      t=t,
      q=q,
      dq=dq,
      ddq=ddq,
      joint_names=np.array(names),
    )

    print("\n========== 轨迹保存成功 ==========")
    print("关节:", names)
    print("轨迹点数:", len(points))
    print("轨迹时长:", t[-1], "s")
    print("q shape:", q.shape)
    print("dq shape:", dq.shape)
    print("ddq shape:", ddq.shape)
    print("保存位置:", path)

    received.append(True)

  qos = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
  )

  subscription = node.create_subscription(DisplayTrajectory, TOPIC, callback, qos)
  print("等待 MoveIt 发布新规划轨迹...")
  print("请在 RViz 点击 Plan,不要点击 Execute")
  deadline = time.monotonic() + 120.0
  try:
    while rclpy.ok() and not received:
      if time.monotonic() > deadline:
        print("等待超时")
        break
      rclpy.spin_once(node, timeout_sec=0.5)
  except KeyboardInterrupt:
    pass
  finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
  main()
