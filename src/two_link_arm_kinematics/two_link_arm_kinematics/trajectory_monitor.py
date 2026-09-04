import rclpy
from rclpy.node import Node
# 轨迹消息
# 用于接收期望关节轨迹：
# joint_names + trajectory points
from trajectory_msgs.msg import JointTrajectory
# 关节状态消息
# 用于接收机器人实际反馈：
# joint name + position + velocity
from sensor_msgs.msg import JointState
# from builtin_interfaces.msg import Duration
from rclpy.time import Time
import numpy as np

class TrajectoryMonitor(Node):
  """
  轨迹跟踪监控节点
  功能：
  1. 订阅期望轨迹:
      /joint_trajectory_controller/joint_trajectory
  2. 订阅实际关节状态:
      /joint_states
  3. 计算:
      tracking error
      e = q_des - q_actual
  比较最终目标点
  """
  def __init__(self):
      super().__init__('trajectory_monitor')
      # 保存期望轨迹
      self.trajectory_points = None
      self.trajectory_times = None
      self.start_time = None
      self.desired_names = None
      # 保存实际状态
      self.actual_positions = None
      self.actual_names = None
      self.actual_time = None
      # 创建轨迹订阅者
      # 接收控制器发送的目标轨迹
      # 消息类型: trajectory_msgs/msg/JointTrajectory
      # 数据: joint_names 、 points[]
      self.create_subscription(
          JointTrajectory,
          '/joint_trajectory_controller/joint_trajectory',
          self.trajectory_callback,
          10
      )
      # 创建关节状态订阅者
      # 接收机器人实际反馈
      # 来源: ros2_control 、 joint_state_broadcaster
      self.create_subscription(
          JointState,
          '/joint_states',
          self.joint_state_callback,
          10
      )

      # 每 0.2 秒计算并打印一次轨迹误差  0.2 s = 5 Hz
      self.error_timer = self.create_timer(
          0.2,
          self.calculate_error
      )

      self.get_logger().info(
          "轨迹监视器已启动"
      )

      # # 公共路径参数 s(t) 实验
      # self.experiment_common_progress()

  def get_desired_position(self,elapsed):
    "寻找当前期望位置函数"
    # 没有收到轨迹
    if self.trajectory_points is None:
      return None
    # 没有轨迹时间
    if self.start_time is None:
      return None
    # 轨迹为空
    if len(self.trajectory_points) == 0:
      return None
    # 1.当前时间在轨迹开始之前
    if elapsed <= self.trajectory_times[0]:
        return np.array(
          self.trajectory_points[0].positions
        )
    # 2.轨迹已经结束
    if elapsed >= self.trajectory_times[-1]:
        return np.array(
          self.trajectory_points[-1].positions
        )
    # 3.处于两个轨迹点之间
      #找到 elapsed 右边的轨迹点
    index_right = np.searchsorted(
        self.trajectory_times,
        elapsed
    )
    index_left = index_right - 1
    # 左右两个轨迹点的时间
    t0 = self.trajectory_times[index_left]
    t1 = self.trajectory_times[index_right]
    # 左右两个轨迹点的位置
    q0 = np.array(
      self.trajectory_points[
        index_left
      ].positions
    )
    q1 = np.array(
      self.trajectory_points[
        index_right
      ].positions
    )
    # 计算插值比例 alpha
      #               elapsed - t0
      # alpha = -----------------------
      #                  t1 - t0
    # t0 <= elapsed <= t1  0 <= alpha <= 1
    alpha = ((elapsed - t0)/(t1 - t0))
    # 线性插值
    desired = ((1.0 - alpha) * q0+alpha * q1)
    return desired
    # index_left = index_right - 1
    # current_time = self.get_clock().now()
    # elapsed = (
    #   current_time
    #   -
    #   self.start_time
    # ).nanoseconds * 1e-9
    # index = np.searchsorted(
    #   self.trajectory_times,
    #   elapsed
    # )
    # if index >= len(self.trajectory_points):
    #   index = len(self.trajectory_points)-1
    # desired = np.array(
    #   self.trajectory_points[index].positions
    # )
    # return desired,elapsed

  # def calculate_common_progress_profile(self,q_start,q_goal,v_max,a_max):
  #   "公共路径参数 s(t) 的速度、加速度限制"
  #   q_start = np.array(q_start, dtype=float)
  #   q_goal = np.array(q_goal, dtype=float)
  #   v_max = np.array(v_max, dtype=float)
  #   a_max = np.array(a_max, dtype=float)
  #   # 总关节位移
  #   delta_q = q_goal - q_start
  #   # 只考虑真正发生运动的关节
  #   moving = np.abs(delta_q) > 1e-12
  #   # 所有关节都不动
  #   if not np.any(moving):
  #       return {
  #           "delta_q": delta_q,
  #           "s_dot_max": 0.0,
  #           "s_ddot_max": 0.0,
  #           "v_peak": 0.0,
  #           "t_acc": 0.0,
  #           "t_cruise": 0.0,
  #           "duration": 0.0,
  #           "profile_type": "stationary"
  #       }
  #   # 公共路径速度限制
  #   # q_dot_i = delta_q_i * s_dot
  #   s_velocity_limits = (v_max[moving] / np.abs(delta_q[moving]))
  #   s_dot_max = np.min(s_velocity_limits)
  #   # 公共路径加速度限制
  #   # q_ddot_i = delta_q_i * s_ddot
  #   s_acceleration_limits = (
  #       a_max[moving]
  #       /
  #       np.abs(delta_q[moving])
  #   )
  #   s_ddot_max = np.min(s_acceleration_limits)
  #   # s 从 0 运动到 1
  #   D = 1.0
  #   # 梯形 / 三角形判断
  #   D_switch = (
  #       s_dot_max ** 2
  #       /
  #       s_ddot_max
  #   )
  #   # 梯形速度轨迹
  #   if D >= D_switch:
  #       v_peak = s_dot_max
  #       t_acc = (
  #           v_peak
  #           /
  #           s_ddot_max
  #       )
  #       t_cruise = (
  #           D
  #           -
  #           v_peak ** 2 / s_ddot_max
  #       ) / v_peak
  #       duration = (
  #           2.0 * t_acc
  #           +
  #           t_cruise
  #       )
  #       profile_type = "梯形速度轨迹"
  #   # 三角形速度轨迹
  #   else:
  #       v_peak = np.sqrt(
  #           s_ddot_max * D
  #       )
  #       t_acc = (
  #           v_peak
  #           /
  #           s_ddot_max
  #       )
  #       t_cruise = 0.0
  #       duration = (
  #           2.0 * t_acc
  #       )
  #       profile_type = "三角速度轨迹"
  #   return {
  #       "delta_q": delta_q,
  #       "s_dot_max": s_dot_max,
  #       "s_ddot_max": s_ddot_max,
  #       "v_peak": v_peak,
  #       "t_acc": t_acc,
  #       "t_cruise": t_cruise,
  #       "duration": duration,
  #       "profile_type": profile_type
  #   }

  # def sample_common_progress(self,profile, t):
  #   "根据时间计算 s s_dot s_ddot"
  #   a = profile["s_ddot_max"]
  #   v_peak = profile["v_peak"]
  #   t_acc = profile["t_acc"]
  #   t_cruise = profile["t_cruise"]
  #   T = profile["duration"]
  #   # 完全静止
  #   if T <= 1e-12:
  #       return 1.0, 0.0, 0.0
  #   # 轨迹开始之前
  #   if t <= 0.0:
  #       return 0.0, 0.0, 0.0
  #   # 轨迹结束之后
  #   if t >= T:
  #       return 1.0, 0.0, 0.0
  #   # 第一段：加速
  #   if t < t_acc:
  #     s = (0.5 * a * t ** 2)
  #     s_dot = (a * t)
  #     s_ddot = a
  #   # 第二段：匀速
  #   elif t < t_acc + t_cruise:
  #     s_acc = (0.5*a*t_acc ** 2)
  #     s = (s_acc + v_peak * (t - t_acc))
  #     s_dot = v_peak
  #     s_ddot = 0.0
  #   # 第三段：减速
  #   else:
  #     remaining_time = (T - t)
  #     # 从终点倒着计算
  #     s = (1.0-0.5 * a * remaining_time ** 2)
  #     s_dot = (a * remaining_time)
  #     s_ddot = -a
  #   return ( s, s_dot, s_ddot)

  # def sample_common_joint_trajectory(self,q_start,q_goal,profile,dt):
  #   "由 s(t) 得到所有关节轨迹"
  #   q_start = np.array(q_start,dtype=float)
  #   q_goal = np.array(q_goal,dtype=float)
  #   delta_q = (q_goal-q_start)
  #   T = profile["duration"]
  #   # 构造时间序列
  #   # 保证最后一个点严格等于 T
  #   trajectory = []
  #   if T <= 1e-12:
  #     times = np.array([0.0])
  #   else:
  #     times = np.arange(0.0,T,dt)
  #     if (len(times) == 0 or abs(times[-1] - T) > 1e-12):
  #       times = np.append(times,T)
  #   for t in times:
  #     (s,s_dot,s_ddot) = self.sample_common_progress(profile,t)
  #     # q(t)
  #     q = (q_start + s * delta_q )
  #     # q_dot(t)
  #     q_dot = (s_dot * delta_q)
  #     # q_ddot(t)
  #     q_ddot = (s_ddot* delta_q)
  #     trajectory.append(
  #       {
  #         "time": float(t),
  #         "position": q,
  #         "velocity": q_dot,
  #         "acceleration": q_ddot,
  #         "s": s,
  #         "s_dot": s_dot,
  #         "s_ddot": s_ddot
  #       }
  #     )
  #   return trajectory

  def calculate_error(self):
    """
    计算轨迹跟踪误差
    数学形式:
    e = q_des - q_actual
    误差大小:
    ||e||
    """
    # result = self.get_desired_position()
    # if result is None:
    #     return
    # desired,elapsed = result
      # 还没收到轨迹
    if self.trajectory_points is None:
      return
    # 还没有轨迹开始时间
    if self.start_time is None:
      return
      # 还没有实际位置
    if self.actual_positions is None:
      return
    # 还没有实际状态时间
    if self.actual_time is None:
      return
    
    # if self.desired_names is not None and self.actual_names is not None:
    #     idx = [self.actual_names.index(n) for n in self.desired_names]
    #     actual = self.actual_positions[idx]  
    # else:
    #     actual = self.actual_positions

    # 计算这个 JointState 对应的轨迹时间
    #absolute state time -  trajectory start time = elapsed trajectory time
    elapsed = (self.actual_time - self.start_time).nanoseconds * 1e-9
    # 插值计算这个时刻的期望位置
    desired = self.get_desired_position(elapsed)
    if desired is None:
      return
    # 根据关节名称重新排列 actual
    if self.desired_names is not None and self.actual_names is not None:
        idx = [self.actual_names.index(name) for name in self.desired_names]
        actual = self.actual_positions[idx]  
    else:
        actual = self.actual_positions
    # 位置误差
    error = (desired - actual)
    self.get_logger().info(
        "\n===== 轨迹误差 =====\n"
        f"time: {elapsed:.3f} s\n"
        f"desired: {desired}\n"
        f"actual : {actual}\n"
        f"error  : {error}\n"
        f"norm   : {np.linalg.norm(error)}"
    )
    
  # =========================================
  # 公共路径参数 s(t) 实验
  # def experiment_common_progress(self):
  #     """
  #     测试公共路径参数 s(t)
  #     测试案例：
  #     q_start = [0.0,  0.0, 0.0]
  #     q_goal  = [1.0, -0.5, 0.8]
  #     v_max = [0.5, 0.5, 0.5]
  #     a_max = [1.0, 1.0, 1.0]
  #     """
  #     # 初始关节位置
  #     q_start = np.array([0.0, 0.0, 0.0])
  #     # 目标关节位置
  #     q_goal = np.array([1.0, -0.5, 0.8])
  #     # 各关节最大速度
  #     v_max = np.array([0.5, 0.5, 0.5])
  #     # 各关节最大加速度
  #     a_max = np.array([1.0, 1.0, 1.0])
  #     # 计算公共路径参数 s(t) 的速度轨迹
  #     profile = self.calculate_common_progress_profile(
  #         q_start,
  #         q_goal,
  #         v_max,
  #         a_max
  #     )
  #     print(
  #       "\n===== 公共路径参数s(t) 下配置 ====="
  #       f"delta_q     : {profile['delta_q']}\n"
  #       f"s_dot_max   : {profile['s_dot_max']}\n"
  #       f"s_ddot_max  : {profile['s_ddot_max']}\n"
  #       f"profile      : {profile['profile_type']}\n"
  #       f"t_acc        : {profile['t_acc']}\n"
  #       f"t_cruise     : {profile['t_cruise']}\n"
  #       f"duration     : {profile['duration']}"
  #     )
      
  #     # 根据 s(t) 生成多关节轨迹
  #     trajectory = self.sample_common_joint_trajectory(
  #         q_start,
  #         q_goal,
  #         profile,
  #         dt=0.01
  #     )
  #     # 检查轨迹点数量
  #     print("轨迹点数量:",len(trajectory))
  #     # 测试几个关键时刻
  #     for t_test in [0.0,0.5,1.25,2.5]:
  #         (s,s_dot,s_ddot) = self.sample_common_progress(
  #             profile,
  #             t_test
  #         )
  #         q = (q_start + s *(q_goal - q_start))
  #         print(
  #             "\n===== 测试点 =====\n"
  #             f"time   : {t_test}\n"
  #             f"s      : {s}\n"
  #             f"s_dot  : {s_dot}\n"
  #             f"s_ddot : {s_ddot}\n"
  #             f"q      : {q}"
  #         )

  # ========================================

  def trajectory_callback(self, msg):
      """
      期望轨迹回调函数
      输入:
      JointTrajectory
      """
      if len(msg.points) == 0:
          return
      # 保存最后目标点
      # 后面升级为实时插值
      self.desired_names = msg.joint_names
      self.trajectory_points = msg.points
      self.trajectory_times = np.array(
          [
              point.time_from_start.sec
              +
              point.time_from_start.nanosec*1e-9
              for point in msg.points
          ]
      )
      # 使用 JointTrajectory 自己的时间戳作为轨迹零点
      self.start_time = Time.from_msg(msg.header.stamp)
      self.get_logger().info("接收到轨迹")

  def joint_state_callback(self,msg):
      """
      实际关节状态回调
      输入:
      sensor_msgs/msg/JointState
      保存:
      q_actual
      """
      # 保存实际关节名称  
      self.actual_names = msg.name
      # 保存实际关节位置
      self.actual_positions = np.array(msg.position)
      # 保存实际状态的采样时间
      self.actual_time = Time.from_msg(msg.header.stamp)

 
def main(args=None):
    # 初始化ROS2
    rclpy.init(args=args)
    # 创建节点
    node = TrajectoryMonitor()
    # 进入循环 持续处理: topic消息 callback函数
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()