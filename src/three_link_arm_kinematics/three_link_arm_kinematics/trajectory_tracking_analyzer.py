import rclpy
from rclpy.node import Node
import csv
import numpy as np
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Float64MultiArray
class TrajectoryTrackingAnalyzer(Node):
  """ 
  第一版 self.q_d = msg.points[-1].positions 只能记录最终目标
  不能严格得到q_d 

  第二版改成 根据time_from_start 插值得到 q_d

  现在增加轨迹缓存  接收/joint_trajectory_controller/joint_trajectory
  
  保存trajectory_points
  """
  def __init__(self):
    super().__init__(
      "trajectory_tracking_analyzer"
    )
    self.q = np.zeros(3)
    self.q_d = np.zeros(3)
    self.tau = np.zeros(3)

    # 轨迹缓存
    self.trajectory_times = []
    self.trajectory_positions = []

    # 是否收到轨迹
    self.has_trajectory = False

    self.start_time = None

    #创建CSV文件
    self.csv_file = open(
      "tracking_data.csv",
      "w",
      newline=""
    )
    #写入器
    self.writer = csv.writer(
      self.csv_file
    )
    #写表头
    self.writer.writerow(
      [
        "time",
        "q1",
        "q2",
        "q3",
        "qd1",
        "qd2",
        "qd3",
        "e1",
        "e2",
        "e3",
        "tau1",
        "tau2",
        "tau3"
      ]
    )
    #订阅实际状态 用于接收q
    self.create_subscription(
      JointState,
      "/joint_states",
      self.joint_state_callback,
      10
    )
    #订阅期望轨迹 用于接收q_d
    self.create_subscription(
      JointTrajectory,
      "/joint_trajectory_controller/joint_trajectory",
      self.trajectory_callback,
      10
    )
    #订阅力矩 接收τ
    self.create_subscription(
      Float64MultiArray,
      "/joint_effort_command",
      self.torque_callback,
      10
    )
    #定时记录
    self.timer = self.create_timer(
      0.01,
      self.record
    )
    self.get_logger().info(
      "轨迹跟踪数据记录器"
    )

  def joint_state_callback(self,msg):
    """
    实际状态回调
    
    从/joint_states 接收q ,检查长度 大于三  
    
    保证三个关节都被保存  
    """
    if len(msg.position)>=3:
      self.q = np.array(
        msg.position[:3]
      )

  def trajectory_callback(self,msg):
    """
    轨迹回调 

    第一版取 q_d = q_goal

    第二版 保存251个点
    """
    if len(msg.points)==0:
      return
    # # 保存最后一个目标点
    # self.q_d = np.array(
    #   msg.points[-1].positions
    # )

    self.trajectory_times.clear()
    self.trajectory_positions.clear()

    for point in msg.points:
      t = (
        point.time_from_start.sec
        +
        point.time_from_start.nanosec
        *1e-9
      )

      self.trajectory_times.append(t)
      self.trajectory_positions.append(
        np.array(
          point.positions[:3]
        )
      )

    self.has_trajectory = True

    self.get_logger().info(
      f"收到轨迹点: {len(self.trajectory_times)}"
    )

  def torque_callback(self,msg):
    """
    力矩回调 保存τ
    """
    if len(msg.data)>=3:
      self.tau=np.array(
        msg.data[:3]
      )

  
  #增加插值函数
  def interpolate_qd(self,t):
    if not self.has_trajectory:
      return np.zeros(3)

    # 超过轨迹结束
    if t >= self.trajectory_times[-1]:
      return self.trajectory_positions[-1]

    # 找区间
    for i in range(len(self.trajectory_times)-1):
      t1 = self.trajectory_times[i]
      t2 = self.trajectory_times[i+1]
      if t1 <= t <= t2:
        q1 = self.trajectory_positions[i]
        q2 = self.trajectory_positions[i+1]
        alpha = (t-t1)/(t2-t1)
        return ((1-alpha)*q1+alpha*q2)

    return self.trajectory_positions[-1]

  def record(self):
    if self.start_time is None:
      self.start_time = (
        #记录开始时间
        self.get_clock().now().nanoseconds
      )
    now = (
      self.get_clock().now().nanoseconds
    )
    #换算单位为s
    t = (
      now-self.start_time
    )/1e9
    # # 计算误差
    # e = self.q_d-self.q
    # 每10ms 重新计算q_d
    self.q_d = self.interpolate_qd(t)
    e = self.q_d-self.q
    # 写入 csv
    self.writer.writerow(
      [
        t,
        *self.q,
        *self.q_d,
        *e,
        *self.tau
      ]
    )
    self.csv_file.flush()

def main(args=None):
  rclpy.init(args=args)
  node=TrajectoryTrackingAnalyzer()
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()

if __name__=="__main__":
  main()