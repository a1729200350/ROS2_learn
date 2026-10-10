import rclpy
import subprocess
import threading
import time
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from controller_manager_msgs.msg import ControllerManagerActivity
class ControllerStateWatch(Node):
  def __init__(self):
    super().__init__("osc_controller_state_watch")
    self.seen_active = False
    self.fault = False
    self.last_state = None
    # OSC 控制循环活跃监控
    self.last_tracking_time = None
    self.active_since = None
    self.tracking_timeout = 0.5
    self.startup_grace = 2.0
    # 订阅 OSC 输出
    self.create_subscription(Float64MultiArray,"/osc_tracking",self.tracking_callback, 10,)
    # 每 50ms 检查一次
    self.create_timer( 0.05,self.check_tracking_timeout,)
    qos = QoSProfile(
      depth=1,
      reliability=ReliabilityPolicy.RELIABLE,
      durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    self.create_subscription(
      ControllerManagerActivity,
      "/controller_manager/activity",
      self.activity_callback,
      qos,
    )
    self.get_logger().info("WAITING:等待控制器状态")
  
  def activity_callback(self, msg):
    controller = next(
      (c for c in msg.controllers
        if c.name == "effort_controller"),
      None,
    )
    state = (controller.state.label if controller is not None else "missing")
    if state == self.last_state:
      return
    self.last_state = state
    if self.fault:
      return
    if not self.seen_active:
      if state == "active":
        self.seen_active = True
        self.active_since = time.monotonic()
        self.get_logger().info("ACTIVE: effort_controller 已成功接管")
      else:
        self.get_logger().info(f"WAITING: 当前状态={state}")
      return

      # 只有曾经 active，后来失去 active，才判定故障
    if state != "active":
      self.trigger_fault(f"控制器接管后失效，当前状态={state}")

  def pause_gazebo(self):
    """故障后请求暂停 Gazebo,并验证暂停状态。"""
    self.get_logger().warn("正在请求暂停 Gazebo...")
    try:
      result = subprocess.run(
        [
          "gz", "service",
          "-s", "/world/empty/control",
          "--reqtype", "gz.msgs.WorldControl",
          "--reptype", "gz.msgs.Boolean",
          "--timeout", "3000",
          "--req", "pause: true",
        ],
        capture_output=True,
        text=True,
        timeout=6.0,
      )
      if result.returncode != 0 or "data: true" not in result.stdout:
        self.get_logger().error(
          f"Gazebo 暂停请求失败: {result.stdout} {result.stderr}"
        )
        return
      self.get_logger().info("Gazebo 已接受暂停请求")
      # 再检查 Gazebo 的真实暂停状态
      check = subprocess.run(
        [
          "gz", "topic",
          "-e",
          "-t", "/world/empty/stats",
          "-n", "1",
        ],
        capture_output=True,
        text=True,
        timeout=5.0,
      )
      if check.returncode == 0 and "paused: true" in check.stdout:
        self.get_logger().warn("保护动作完成: Gazebo 已暂停")
      else:
        self.get_logger().error("暂停请求已接受，但未能确认 Gazebo 暂停状态")
    except (subprocess.TimeoutExpired, OSError) as exc:
      self.get_logger().error(f"Gazebo 暂停操作异常: {exc}")

  def tracking_callback(self, msg):
    """记录最近收到 OSC 跟踪消息的时间。"""
    self.last_tracking_time = time.monotonic()

  def trigger_fault(self, reason):
    """统一的故障锁定和暂停入口。"""
    if self.fault:
      return
    self.fault = True
    self.get_logger().error(f"FAULT: {reason}")
    threading.Thread(target=self.pause_gazebo,daemon=True,).start()

  def check_tracking_timeout(self):
    """只在控制器接管后监控 OSC 输出超时。"""
    if self.fault or not self.seen_active:
      return
    now = time.monotonic()

    # 尚未收到第一条 OSC 消息
    if self.last_tracking_time is None:
      if now - self.active_since > self.startup_grace:
        self.trigger_fault("控制器已激活，但 OSC 未开始发布 tracking")
      return
    # 已收到过消息，检查是否中断
    age = now - self.last_tracking_time
    if age > self.tracking_timeout:
      self.trigger_fault(f"OSC 控制输出超时: {age * 1000:.1f} ms")

def main():
  rclpy.init()
  node = ControllerStateWatch()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.destroy_node()
    rclpy.try_shutdown()

if __name__ == "__main__":
  main()
