import time
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_srvs.srv import Trigger
from controller_manager_msgs.srv import SwitchController
class OSCAutoActivator(Node):
  def __init__(self):
    super().__init__("osc_auto_activator")
    self.ready_client = self.create_client(Trigger,"/osc_ready",)
    self.switch_client = self.create_client(SwitchController,"/controller_manager/switch_controller",)
  def call_service(self, client, request, timeout):
    future = client.call_async(request)
    rclpy.spin_until_future_complete( self,future,timeout_sec=timeout,)
    if not future.done():
      future.cancel()
      return None
    try:
      return future.result()
    except Exception as exc:
      self.get_logger().error(str(exc))
      return None

  def run(self):
    self.get_logger().info("等待 OSC 就绪...")
    deadline = time.monotonic() + 30.0
    last_reason = None
    while rclpy.ok() and time.monotonic() < deadline:
      # 先确认 controller_manager 服务可用
      if not self.switch_client.wait_for_service(timeout_sec=0.5):
        continue
      if not self.ready_client.wait_for_service(timeout_sec=0.5):
        continue
      response = self.call_service( self.ready_client, Trigger.Request(), timeout=1.0,)
      if response is None:
        continue
      if response.success:
        self.get_logger().info("OSC 就绪检查通过")
        break
      if response.message != last_reason:
        self.get_logger().info(f"尚未就绪：{response.message}")
        last_reason = response.message
      time.sleep(0.3)
    else:
      self.get_logger().error( "OSC 就绪超时，保持 effort_controller inactive")
      return 1
    # 就绪后激活 effort_controller
    request = SwitchController.Request()
    request.activate_controllers = ["effort_controller"]
    request.deactivate_controllers = []
    request.strictness = SwitchController.Request.STRICT
    request.activate_asap = True
    request.timeout = Duration(seconds=3.0).to_msg()
    response = self.call_service(self.switch_client,request,timeout=5.0,)
    if response is None or not response.ok:
      self.get_logger().error("effort_controller 自动激活失败")
      return 1
    self.get_logger().info( "effort_controller 自动激活成功")
    return 0

def main(args=None):
  rclpy.init(args=args)
  node = OSCAutoActivator()
  try:
    return node.run()
  finally:
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
  raise SystemExit(main())