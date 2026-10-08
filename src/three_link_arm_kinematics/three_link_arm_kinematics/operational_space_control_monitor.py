"""ROS2 严格操作空间控制节点:OSC + Posture。"""
import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from three_link_arm_kinematics.operational_space_controller import (OperationalSpaceController,)
from three_link_arm_kinematics.operational_space_dynamics import (OperationalSpaceDynamics,)
from three_link_arm_kinematics.task_space_trajectory import (sample_task_trajectory,)

class OperationalSpaceControlMonitor(Node):
  def __init__(self):
    super().__init__("operational_space_control_monitor")
    self.controller = OperationalSpaceController( kp=(10.0, 10.0), kd=(4.43, 4.43),)
    self.osc = OperationalSpaceDynamics()

    # 姿态二级任务
    self.q_posture_desired = np.zeros(3)
    self.Kp_posture = np.eye(3)
    self.Kd_posture = 0.5 * np.eye(3)

    # 当前实际关节状态
    self.q = None
    self.q_dot = None
    self.last_state_time = None

    # 轨迹状态
    self.start_time = None
    self.x_start = None
    self.x_goal = None
    self.move_duration = 2.0

    # 故障标记：故障后不再输出控制力矩
    self.failed = False

    # 数值边界保护
    self.min_sigma_threshold = 1e-6

    # 仅作为仿真实验异常值保护
    # 不是实际电机的额定力矩
    self.max_abs_tau = 100.0

    # 4 秒实验统计
    self.sample_count = 0
    self.error_square_sum = np.zeros(2)
    self.max_error = np.zeros(2)
    self.max_torque = np.zeros(3)
    self.summary_printed = False

    self.create_subscription(
      JointState,
      "/joint_states",
      self.joint_state_callback,
      10,
    )
    self.tau_pub = self.create_publisher(
      Float64MultiArray,
      "/joint_effort_command",
      10,
    )
    self.tracking_pub = self.create_publisher(
      Float64MultiArray,
      "/osc_tracking",
      10,
    )
    self.timer = self.create_timer(
      0.01,
      self.control_loop,
    )
    self.get_logger().info(
      "OSC + Posture ROS2 控制节点已启动"
    )

  def joint_state_callback(self, msg):
    names = ["joint1", "joint2", "joint3"]
    if not all(name in msg.name for name in names):
      return
    indices = [msg.name.index(name) for name in names]
    if (
      len(msg.position) <= max(indices)
      or len(msg.velocity) <= max(indices)
    ):
      return
    q = np.asarray([msg.position[i] for i in indices],dtype=float,)
    q_dot = np.asarray( [msg.velocity[i] for i in indices], dtype=float,)

    if not (np.all(np.isfinite(q)) and np.all(np.isfinite(q_dot))):
      return
    self.q = q
    self.q_dot = q_dot
    self.last_state_time = time.monotonic()

  def stop_on_error(self, reason):

    if not self.failed:
      self.failed = True
      self.get_logger().error(reason)

  def control_loop(self):

      if self.failed or self.q is None:
        return
      now = time.monotonic()

      # 防止使用过期关节状态继续计算力矩
      if now - self.last_state_time > 0.05:
        self.stop_on_error("关节状态反馈超时，停止输出力矩")
        return
      q = self.q.copy()
      q_dot = self.q_dot.copy()
      kin = self.controller.kinematics
      try:

        J = kin.jacobian(q)
        sigma_min = np.linalg.svd(J, compute_uv=False)[-1]

        if sigma_min < self.min_sigma_threshold:
          self.stop_on_error(f"Jacobian 接近奇异,sigma_min={sigma_min:.3e}")
          return

        # 首次获得有效关节状态后建立轨迹起点
        if self.start_time is None:
          self.x_start = kin.forward_kinematics(q)
          self.x_goal = ( self.x_start + np.array([0.05, -0.03]))
          self.start_time = now
          self.get_logger().info( f"任务起点：{self.x_start}" f"；目标：{self.x_goal}")

        t = now - self.start_time
        x_d, x_dot_d, x_ddot_d = (sample_task_trajectory(t,self.move_duration, self.x_start,self.x_goal,))

        # 一级任务：末端加速度指令
        command = (self.controller .calculate_task_acceleration_command( q, q_dot, x_d, x_dot_d, x_ddot_d,))

        # 严格操作空间动力学
        task_result = self.osc.calculate_task_force( q, q_dot, command["x_ddot_command"],)
        tau_task = task_result["tau"]

        # 动态一致零空间
        null_result = (self.osc.calculate_null_space_projector(q))
        N_T = null_result["N_T"]
        tau_0 = ( self.Kp_posture @ (self.q_posture_desired - q) - self.Kd_posture @ q_dot)
        tau_posture = N_T @ tau_0
        tau_total = tau_task + tau_posture

      except (np.linalg.LinAlgError, ValueError) as error:
        self.stop_on_error(f"OSC 计算失败：{error}")
        return

      if not np.all(np.isfinite(tau_total)):
        self.stop_on_error("计算出现非有限力矩")
        return

      if np.max(np.abs(tau_total)) > self.max_abs_tau:
        self.stop_on_error("仿真力矩超过异常值保护阈值")
        return

      # 发布关节力矩
      tau_msg = Float64MultiArray()
      tau_msg.data = tau_total.tolist()
      self.tau_pub.publish(tau_msg)

      # 当前末端误差
      x = command["x"]
      error = x_d - x

      # 记录：
      # t, xd(2), x(2), error(2),
      # q(3), q_dot(3), tau(3)
      data = np.concatenate([[t],x_d, x, error,q,q_dot,tau_total,])
      tracking_msg = Float64MultiArray()
      tracking_msg.data = data.tolist()
      self.tracking_pub.publish(tracking_msg)

      # 统计前 4 秒
      if not self.summary_printed:

        self.sample_count += 1
        self.error_square_sum += error**2
        self.max_error = np.maximum( self.max_error, np.abs(error),)
        self.max_torque = np.maximum( self.max_torque, np.abs(tau_total),)

        if t >= 4.0:
          rms = np.sqrt( self.error_square_sum / self.sample_count)
          self.get_logger().info(
            "\n========== OSC ROS2 结果 ==========\n"
            f"RMS error (mm): {rms * 1000}\n"
            f"Max error (mm): {self.max_error * 1000}\n"
            f"Final error (mm): {error * 1000}\n"
            f"Max torque (Nm): {self.max_torque}\n"
            f"Final q: {q}\n"
            f"Final q_dot: {q_dot}"
          )

          self.summary_printed = True


def main(args=None):

  rclpy.init(args=args)

  node = OperationalSpaceControlMonitor()

  try:
    rclpy.spin(node)
  finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
  main()
