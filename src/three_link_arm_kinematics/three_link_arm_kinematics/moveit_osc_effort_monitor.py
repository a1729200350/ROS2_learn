
"""通过已有严格 OSC 执行 MoveIt 保存的 NPZ 轨迹。"""
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import ( QoSProfile, ReliabilityPolicy, DurabilityPolicy,)
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger
from controller_manager_msgs.msg import ControllerManagerActivity
from three_link_arm_kinematics.operational_space_controller import ( OperationalSpaceController,)
from three_link_arm_kinematics.operational_space_dynamics import ( OperationalSpaceDynamics,)
class MoveItOSCEffortMonitor(Node):
  def __init__(self):
    super().__init__("moveit_osc_effort_monitor")
    filename = self.declare_parameter( "trajectory_file", "").value
    if not filename:
      raise ValueError("必须指定 trajectory_file 参数")
    path = Path(filename).expanduser().resolve()
    with np.load(path, allow_pickle=False) as data:
      self.t_ref = data["t"].astype(float)
      self.q_ref = data["q"].astype(float)
      self.dq_ref = data["dq"].astype(float)
      self.ddq_ref = data["ddq"].astype(float)
      names = data["joint_names"].tolist()
      frame = str(data["task_frame"])
    n = len(self.t_ref)
    arrays = ( self.t_ref, self.q_ref, self.dq_ref, self.ddq_ref,)
    if (
      names != ["joint1", "joint2", "joint3"]
      or frame != "arm_base"
      or n < 2
      or self.t_ref.shape != (n,)
      or any(a.shape != (n, 3) for a in arrays[1:])
      or not all(np.all(np.isfinite(a)) for a in arrays)
      or abs(self.t_ref[0]) > 1e-9
      or np.any(np.diff(self.t_ref) <= 0)
    ):
      raise ValueError("轨迹数据结构或时间无效")

    # 与已经验证的离线闭环实验一致
    kp = self.declare_parameter("task_kp", [10.0, 10.0]).value
    kd = self.declare_parameter("task_kd", [4.43, 4.43]).value
    gravity = float( self.declare_parameter("gravity", 9.8).value)
    posture_kp = float( self.declare_parameter("posture_kp", 1.0).value)
    posture_kd = float( self.declare_parameter("posture_kd", 0.5).value)
    limits = self.declare_parameter( "torque_limit", [10.0, 10.0, 10.0]).value
    self.torque_limit = np.asarray(limits, dtype=float)
    if (
      self.torque_limit.shape != (3,)
      or not np.all(np.isfinite(self.torque_limit))
      or np.any(self.torque_limit <= 0)
    ):
      raise ValueError("torque_limit 配置无效")

    self.controller = OperationalSpaceController( kp=tuple(kp), kd=tuple(kd),)
    self.osc = OperationalSpaceDynamics()
    self.osc.dynamics.g = gravity
    self.Kp0 = np.eye(3) * posture_kp
    self.Kd0 = np.eye(3) * posture_kd
    self.q = None
    self.dq = None
    self.last_joint_wall = None
    self.last_effort_wall = None
    self.last_raw_effort = None
    self.effort_active = False
    self.start_sim = None
    self.failed = False
    self.last_warning_wall = -float("inf")
    self.last_progress_sec = -1
    self.create_subscription( JointState,"/joint_states",self.on_joint_state, 10,)
    activity_qos = QoSProfile( depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,)
    self.create_subscription( ControllerManagerActivity,"/controller_manager/activity", self.on_controller_activity, activity_qos,)
    self.tau_pub = self.create_publisher( Float64MultiArray, "/joint_effort_command", 10,)
    self.tracking_pub = self.create_publisher( Float64MultiArray,"/osc_tracking", 10,)
    self.create_service(Trigger, "/osc_ready", self.on_ready)
    self.create_service(Trigger, "/osc_start", self.on_start)
    self.create_timer(0.01, self.control_loop)
    self.get_logger().info( f"MoveIt轨迹加载成功: {path}, "f"{n}点, {self.t_ref[-1]:.3f}s")

  def on_controller_activity(self, msg):
    self.effort_active = any( c.name == "effort_controller" and c.state.label == "active" for c in msg.controllers)

  def on_joint_state(self, msg):
    names = ("joint1", "joint2", "joint3")
    if not all(name in msg.name for name in names):
      return
    ids = [msg.name.index(name) for name in names]
    if ( max(ids) >= len(msg.position) or max(ids) >= len(msg.velocity)):
      return
    q = np.asarray([msg.position[i] for i in ids], dtype=float)
    dq = np.asarray([msg.velocity[i] for i in ids], dtype=float)
    if ( np.all(np.isfinite(q)) and np.all(np.isfinite(dq))):
      self.q = q
      self.dq = dq
      self.last_joint_wall = time.monotonic()

  def sample_reference(self, t):
    # 未启动轨迹：保持轨迹起点
    if self.start_sim is None:
      return ( self.q_ref[0].copy(), np.zeros(3), np.zeros(3),)
    # 轨迹结束：保持终点
    if t >= self.t_ref[-1]:
      return ( self.q_ref[-1].copy(), np.zeros(3),np.zeros(3),)
    t = max(0.0, t)
    def sample(a):
      return np.array([ np.interp(t, self.t_ref, a[:, i]) for i in range(3)])
    return (sample(self.q_ref),sample(self.dq_ref),sample(self.ddq_ref),)

  def check_ready(self):
    now = time.monotonic()
    if self.failed:
      return "OSC已故障"
    if ( self.q is None or self.dq is None or self.last_joint_wall is None):
      return "尚无完整关节反馈"
    if now - self.last_joint_wall > 0.2:
      return "关节反馈超时"
    if ( self.last_effort_wall is None or now - self.last_effort_wall > 0.1):
      return "尚无新鲜有效力矩"
    if self.start_sim is not None:
      return "轨迹已经启动"
    if np.max(np.abs(self.dq)) > 0.1:
      return "关节速度过大"
    if np.max(np.abs(self.q - self.q_ref[0])) > 0.02:
      return "当前姿态没有对准轨迹起点"

    if (
      self.last_raw_effort is None
      or np.any(np.abs(self.last_raw_effort) > self.torque_limit)
    ):
      return "初始需求力矩超限"
    return None

  def on_ready(self, request, response):
    reason = self.check_ready()
    response.success = reason is None
    response.message = reason or "MoveIt OSC已就绪"
    return response

  def on_start(self, request, response):
    reason = self.check_ready()
    if reason is None and not self.effort_active:
      reason = "effort_controller尚未active"
    now_sim = (self.get_clock().now().nanoseconds * 1e-9)
    if reason is None and now_sim <= 0:
      reason = "Gazebo仿真时钟尚未开始"
    response.success = reason is None
    response.message = reason or "MoveIt OSC轨迹开始"
    if reason is None:
      self.start_sim = now_sim
      self.get_logger().info("MoveIt轨迹开始执行")
    return response

  def publish_torque(self, tau):
    msg = Float64MultiArray()
    msg.data = np.asarray( tau, dtype=float).tolist()
    self.tau_pub.publish(msg)

  def fault(self, reason):
    if not self.failed:
      self.failed = True
      self.get_logger().error( f"OSC FAULT: {reason}" )
    self.publish_torque(np.zeros(3))
    # 故障时停止发布 tracking
    # 原 watchdog 会在 active 后检测超时并请求暂停 Gazebo

  def control_loop(self):
    if self.failed:
      self.publish_torque(np.zeros(3))
      return
    if self.q is None:
      return
    now_wall = time.monotonic()
    if ( self.last_joint_wall is None or now_wall - self.last_joint_wall > 0.2):
      self.fault("关节状态反馈超时")
      return
    now_sim = ( self.get_clock().now().nanoseconds * 1e-9)
    if now_sim <= 0:
      return
    if ( self.start_sim is not None and now_sim < self.start_sim):
      self.fault("仿真时间倒退")
      return
    elapsed = ( 0.0 if self.start_sim is None else max(0.0, now_sim - self.start_sim))
    q = self.q.copy()
    dq = self.dq.copy()
    try:
      qd, dqd, ddqd = self.sample_reference(elapsed)
      kin = self.controller.kinematics
      J = kin.jacobian(q)
      sigma_min = np.linalg.svd( J, compute_uv=False)[-1]
      if (
        sigma_min < 0.005
        or np.max(np.abs(q)) > 3.14
        or np.max(np.abs(dq)) > 10.0
        or np.linalg.norm(q - qd) > 1.0
      ):
        self.fault( f"安全边界触发: " f"sigma={sigma_min:.5g}, " f"q={q}, dq={dq}")
        return

      # MoveIt关节轨迹 -> 任务空间参考
      Jd = kin.jacobian(qd)
      xd = kin.forward_kinematics(qd)
      dxd = Jd @ dqd
      ddxd = ( Jd @ ddqd + kin.jacobian_dot(qd, dqd) @ dqd )

      # 任务空间加速度指令
      task_cmd = ( self.controller .calculate_task_acceleration_command( q, dq, xd, dxd, ddxd ))
      # 原严格OSC计算
      dyn = self.osc.calculate_task_force( q, dq, task_cmd["x_ddot_command"],)
      # 动态一致零空间
      N_T = ( self.osc.calculate_null_space_projector(q)["N_T"])
      # MoveIt移动姿态的逆动力学前馈
      tau_id = ( dyn["M"] @ ddqd + dyn["V"] + dyn["G"])
      tau_0 = ( tau_id + self.Kp0 @ (qd - q) + self.Kd0 @ (dqd - dq))
      raw = dyn["tau"] + N_T @ tau_0
    except ( np.linalg.LinAlgError, ValueError, FloatingPointError,) as exc:
      self.fault(f"OSC计算失败: {exc}")
      return
    if (not np.all(np.isfinite(raw))or np.max(np.abs(raw)) > 100.0):
      self.fault("力矩非有限或超过100Nm异常阈值")
      return
    torque = np.clip( raw, -self.torque_limit, self.torque_limit,)
    if (
      np.any(np.abs(raw) > self.torque_limit)
      and now_wall - self.last_warning_wall > 1.0
    ):
      self.get_logger().warn(f"力矩限幅: raw={raw}, applied={torque}")
      self.last_warning_wall = now_wall
    self.publish_torque(torque)
    self.last_raw_effort = raw.copy()
    self.last_effort_wall = time.monotonic()

    # 与原有OSC日志/看门狗格式保持一致
    x = task_cmd["x"]
    tracking = np.concatenate([ [elapsed], xd, x, xd - x, q, dq, torque,])
    msg = Float64MultiArray()
    msg.data = tracking.tolist()
    self.tracking_pub.publish(msg)
    if self.start_sim is not None:
      second = int(elapsed)
      if second > self.last_progress_sec:
        self.last_progress_sec = second
        self.get_logger().info( f"t={elapsed:.1f}s, " f"error_mm={1000 * (xd - x)}, " f"sigma={sigma_min:.4f}" )

def main(args=None):
    rclpy.init(args=args)
    node = MoveItOSCEffortMonitor()
    try:
      rclpy.spin(node)
    except KeyboardInterrupt:
      pass
    finally:
      node.destroy_node()
      rclpy.try_shutdown()
if __name__ == "__main__":
  main()
