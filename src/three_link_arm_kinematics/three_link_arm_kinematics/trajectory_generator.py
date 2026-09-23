import numpy as np
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import (JointTrajectory,JointTrajectoryPoint)
class TrajectoryGenerator(Node):
  def __init__(self):
    super().__init__('trajectory_generator')
    # 轨迹发布器
    self.trajectory_pub = self.create_publisher(JointTrajectory,'/joint_trajectory_controller/joint_trajectory',10)
    # 当前实验模式
    # "common" 公共路径参数 s(t)
    # "independent" 各关节独立规划 + 同时到达
    self.test_mode = "common"
    # 延迟 1 秒，只执行一次 给 controller 时间完成订阅发现
    self.test_timer = self.create_timer(
        1.0,
        self.run_test_once
    )
    self.get_logger().info('轨迹生成器启动.')
  # 单关节梯形 / 三角形速度轨迹
  def calculate_trapezoidal_profile(self,q_start,q_goal,max_velocity,max_acceleration):
      """计算单关节梯形/三角形速度轨迹参数。"""
      delta_q = (q_goal- q_start)
      # 运动距离
      distance = abs(delta_q)
      # 运动方向
      direction = float(np.sign(delta_q))
      # 特殊情况： 起点和终点相同
      if distance <= 1e-12:
          return {
              'profile_type': 'stationary',
              'q_start': q_start,
              'q_goal': q_goal,
              'delta_q': delta_q,
              'distance': 0.0,
              'direction': 0.0,
              'max_velocity': max_velocity,
              'max_acceleration': max_acceleration,
              'peak_velocity': 0.0,
              'acceleration_time': 0.0,
              'cruise_time': 0.0,
              'duration': 0.0
          }
      if max_velocity <= 0.0:
          raise ValueError('max_velocity 必须大于 0')
      if max_acceleration <= 0.0:
          raise ValueError('max_acceleration 必须大于 0')
      # 判断达到 vmax 所需要的最小距离
      # D_switch = vmax^2 / amax
      switching_distance = (
          max_velocity ** 2
          / max_acceleration
      )
      # 情况 1：标准梯形
      if distance >= switching_distance:
          profile_type = '标准梯形'
          peak_velocity = (max_velocity)  #峰值速度大小
          acceleration_time = (
              peak_velocity
              / max_acceleration
          )
          #匀速距离
          cruise_distance = (
              distance
              - peak_velocity ** 2
              / max_acceleration
          )
          #匀速时间
          cruise_time = (
              cruise_distance
              / peak_velocity
          )
      # 情况 2：三角形
      else:
          profile_type = '三角形'
          peak_velocity = np.sqrt(
              max_acceleration
              * distance
          )
          acceleration_time = (
              peak_velocity
              / max_acceleration
          )
          cruise_time = 0.0
      duration = (
          2.0 * acceleration_time
          + cruise_time
      )
      return {
          'profile_type': profile_type,
          'q_start': q_start,
          'q_goal': q_goal,
          'delta_q': delta_q,
          'distance': distance,
          'direction': direction,
          'max_velocity': max_velocity,
          'max_acceleration': max_acceleration,
          'peak_velocity': peak_velocity,
          'acceleration_time':
              acceleration_time,
          'cruise_time':
              cruise_time,
          'duration':
              duration
      }

  def sample_trapezoidal_trajectory(self,profile,t):
      """采样单关节梯形/三角形速度轨迹。"""
      q_start = profile['q_start']
      q_goal = profile['q_goal']
      direction = profile['direction']
      peak_velocity = profile['peak_velocity']
      max_acceleration = profile['max_acceleration']
      acceleration_time = (
          profile['acceleration_time']
      )
      cruise_time = (
          profile['cruise_time']
      )
      duration = (
          profile['duration']
      )
      # 静止情况 
      # q_goal - q_start = 0 
      if profile['profile_type'] == 'stationary':
          return {
              't': 0.0,
              'q': q_start,
              'q_dot': 0.0,
              'q_ddot': 0.0,
              'phase': 'stationary'
          }
      # 限制查询时间在 [0, T]
      t_clamped = float(
          np.clip(
              t,
              0.0,
              duration
          )
      )
      # 关键时间  加速结束时间
      t_acc_end = (acceleration_time)
      # 减速开始时间
      t_dec_start = (
          acceleration_time
          + cruise_time
      )
      # 关键位置 加速结束位置
      q_acc_end = (
          q_start
          + direction
          * 0.5
          * max_acceleration
          * acceleration_time ** 2
      )
      # 减速开始位置
      q_dec_start = (
          q_acc_end
          + direction
          * peak_velocity
          * cruise_time
      )
      # 第一段：加速
      if t_clamped < t_acc_end:
          q = (
              q_start
              + direction
              * 0.5
              * max_acceleration
              * t_clamped ** 2
          )
          q_dot = (
              direction
              * max_acceleration
              * t_clamped
          )
          q_ddot = (
              direction
              * max_acceleration
          )
          phase = '加速阶段'
      # 第二段：匀速
      elif t_clamped < t_dec_start:
          local_time = (
              t_clamped
              - t_acc_end
          )
          q = (
              q_acc_end
              + direction
              * peak_velocity
              * local_time
          )
          q_dot = (
              direction
              * peak_velocity
          )
          q_ddot = 0.0
          phase = '匀速阶段'
      # 第三段：减速
      else:

          local_time = (
              t_clamped
              - t_dec_start
          )
          q = (
              q_dec_start
              + direction
              * (
                  peak_velocity
                  * local_time
                  - 0.5
                  * max_acceleration
                  * local_time ** 2
              )
          )
          q_dot = (
              direction
              * (
                  peak_velocity
                  - max_acceleration
                  * local_time
              )
          )
          q_ddot = (
              -direction
              * max_acceleration
          )
          phase = '减速阶段'
      # 防止浮点误差导致终点出现极小偏差
      if t_clamped >= duration:
          q = q_goal
          q_dot = 0.0
          q_ddot = 0.0
      return {
          't': t_clamped,
          'q': q,
          'q_dot': q_dot,
          'q_ddot': q_ddot,
          'phase': phase
      }

 # 独立关节 + 同时到达：时间缩放函数
  def scale_trapezoidal_profile(self,profile,target_duration):
      """
      时间缩放梯形速度轨迹。
      保持： 起点 终点 轨迹类型
      只改变： 执行时间 速度 加速度
      """
      original_duration = (
          profile['duration']
      )
      # 静止关节
      if original_duration <= 1e-12:
        scaled = profile.copy()
        scaled['duration'] = (
            target_duration
        )
        return scaled
      if target_duration <= 1e-12:
          raise ValueError(
              'target_duration 必须大于 0'
          )

      # 时间比例
      r = (
          original_duration
          /
          target_duration
      )
      # 复制原字典
      scaled_profile = profile.copy()
      # 时间缩放
      # v' = r v
      # a' = r^2 a
      scaled_profile['peak_velocity'] = (
          r
          *
          profile['peak_velocity']
      )
      scaled_profile['max_velocity'] = (
          r
          *
          profile['max_velocity']
      )
      scaled_profile['max_acceleration'] = (
          r**2
          *
          profile['max_acceleration']
      )
      # 时间参数同步
      scaled_profile['acceleration_time'] = (
          profile['acceleration_time']
          /
          r
      )
      scaled_profile['cruise_time'] = (
          profile['cruise_time']
          /
          r
      )
      scaled_profile['duration'] = (target_duration)
      return scaled_profile

  def calculate_multi_joint_profile(self,q_start,q_goal,max_velocity,max_acceleration):
      "计算多关节配置，用于时间缩放"
      #计算原始时间
      #假设三个关节  分别计算 各个关节的时间
      profiles = []       #python空列表初始化
      durations = []
      for i in range(len(q_start)):
          profile = self.calculate_trapezoidal_profile(
              q_start[i],
              q_goal[i],
              max_velocity[i],
              max_acceleration[i]
          )
          profiles.append(profile)
          durations.append(
              profile['duration']
          )
      #同步时间
      T_sync = max(durations)
      # self.get_logger().info(
      #     f"original durations: {durations}"
      # )

      # self.get_logger().info(
      #     f"T_sync: {T_sync}"
      # )
      # 所有关节全部静止
      if T_sync <= 1e-12:
          return {
              'profiles': profiles,
              'duration': 0.0
          }
      #重新缩放
      sync_profiles=[]
      for i in range(len(q_start)):
          # Ti = durations[i]
          # r = Ti / T_sync
          # v_new = (r * max_velocity[i])
          # a_new = ( r**2 * max_acceleration[i])
          #会重新触发轨迹结构判断 造成 之前的矩阵变为三角形 
          #   新定义了一个最大速度 最大加速度 然后Dswitch 变大
          #   然后 它实际的距离小于Dswitch  还没达到最大速度就要减速 然后 被判断为三角形
          #   浮点误差 边界判断 三角形/梯形临界情况 重新计算阶段时间

          # #重新规划  调用之前的 calculate_trapezoidal_profile() 函数
          # profile_sync = (
          #     self.calculate_trapezoidal_profile(
          #         q_start[i],
          #         q_goal[i],
          #         v_new,
          #         a_new
          #     )
          # )

          #直接时间缩放原profile
          profile_sync = (
              self.scale_trapezoidal_profile(
                  profiles[i],
                  T_sync
              )
          )
          sync_profiles.append(
              profile_sync
          )
      return {
      "profiles": sync_profiles,
      "duration": T_sync
      }

  def build_time_array(self,duration,dt):
      if dt <= 0.0:
          raise ValueError('dt 必须大于 0')
      if duration <= 1e-12:
          return np.array([0.0])
      # 不允许生成 > T 的点
      times = np.arange(0.0,duration,dt)
      # 强制最后一个点严格等于 T
      if (
          len(times) == 0
          or
          abs(times[-1] - duration) > 1e-12
      ):
          times = np.append(times,duration)
      return times

  def sample_multi_joint_trajectory(self,sync_profiles,dt):
    "多关节轨迹采样器"
    #输入 multi_profile dt
    #输出 trajectory_points
    T = sync_profiles[0]['duration']  #获取总时间
    times = self.build_time_array(                #生成时间序列
        T, 
        dt
    )
    #遍历采样
    trajectory=[]
    for t in times:
        q=[]
        q_dot=[]
        q_ddot=[]
        #遍历各个关节
        for profile in sync_profiles:
            state = self.sample_trapezoidal_trajectory(
                profile,
                t
            )
            q.append(
                state['q']
            )
            q_dot.append(
                state['q_dot']
            )
            q_ddot.append(
                state['q_ddot']
            )
        #保存完整轨迹节点
        trajectory.append(
            {
            "time":t,
            "position":np.array(q),
            "velocity":np.array(q_dot),
            "acceleration":np.array(q_ddot)
            }
        )
    return trajectory   

  # 公共路径参数 s(t)
  def calculate_common_progress_profile(self,q_start,q_goal,v_max,a_max):
    "公共路径参数 s(t) 的速度、加速度限制"
    q_start = np.array(q_start, dtype=float)
    q_goal = np.array(q_goal, dtype=float)
    v_max = np.array(v_max, dtype=float)
    a_max = np.array(a_max, dtype=float)
    # 总关节位移
    delta_q = q_goal - q_start
    # 只考虑真正发生运动的关节
    moving = np.abs(delta_q) > 1e-12
    # 所有关节都不动
    if not np.any(moving):
        return {
            "delta_q": delta_q,
            "s_dot_max": 0.0,
            "s_ddot_max": 0.0,
            "v_peak": 0.0,
            "t_acc": 0.0,
            "t_cruise": 0.0,
            "duration": 0.0,
            "profile_type": "stationary"
        }
    # 公共路径速度限制
    # q_dot_i = delta_q_i * s_dot
    s_velocity_limits = (v_max[moving] / np.abs(delta_q[moving]))
    s_dot_max = np.min(s_velocity_limits)
    # 公共路径加速度限制
    # q_ddot_i = delta_q_i * s_ddot
    s_acceleration_limits = (
        a_max[moving]
        /
        np.abs(delta_q[moving])
    )
    s_ddot_max = np.min(s_acceleration_limits)
    # s 从 0 运动到 1
    D = 1.0
    # 梯形 / 三角形判断
    D_switch = (
        s_dot_max ** 2
        /
        s_ddot_max
    )
    # 梯形速度轨迹
    if D >= D_switch:
        v_peak = s_dot_max
        t_acc = (
            v_peak
            /
            s_ddot_max
        )
        t_cruise = (
            D
            -
            v_peak ** 2 / s_ddot_max
        ) / v_peak
        duration = (
            2.0 * t_acc
            +
            t_cruise
        )
        profile_type = "梯形速度轨迹"
    # 三角形速度轨迹
    else:
        v_peak = np.sqrt(
            s_ddot_max * D
        )
        t_acc = (
            v_peak
            /
            s_ddot_max
        )
        t_cruise = 0.0
        duration = (
            2.0 * t_acc
        )
        profile_type = "三角速度轨迹"
    return {
        "delta_q": delta_q,
        "s_dot_max": s_dot_max,
        "s_ddot_max": s_ddot_max,
        "v_peak": v_peak,
        "t_acc": t_acc,
        "t_cruise": t_cruise,
        "duration": duration,
        "profile_type": profile_type
    }

  def sample_common_progress(self,profile, t):
    "根据时间计算 s s_dot s_ddot"
    a = profile["s_ddot_max"]
    v_peak = profile["v_peak"]
    t_acc = profile["t_acc"]
    t_cruise = profile["t_cruise"]
    T = profile["duration"]
    # 完全静止
    if T <= 1e-12:
        return 1.0, 0.0, 0.0
    # 轨迹开始之前
    if t <= 0.0:
        return 0.0, 0.0, 0.0
    # 轨迹结束之后
    if t >= T:
        return 1.0, 0.0, 0.0
    # 第一段：加速
    if t < t_acc:
      s = (0.5 * a * t ** 2)
      s_dot = (a * t)
      s_ddot = a
    # 第二段：匀速
    elif t < t_acc + t_cruise:
      s_acc = (0.5*a*t_acc ** 2)
      s = (s_acc + v_peak * (t - t_acc))
      s_dot = v_peak
      s_ddot = 0.0
    # 第三段：减速
    else:
      remaining_time = (T - t)
      # 从终点倒着计算
      s = (1.0-0.5 * a * remaining_time ** 2)
      s_dot = (a * remaining_time)
      s_ddot = -a
    return ( s, s_dot, s_ddot)

  def sample_common_joint_trajectory(self,q_start,q_goal,profile,dt):
      "由 s(t) 得到所有关节轨迹"
      q_start = np.array(q_start,dtype=float)
      q_goal = np.array(q_goal,dtype=float)
      delta_q = (q_goal-q_start)
      T = profile["duration"]
      # 构造时间序列
      # 保证最后一个点严格等于 T
      trajectory = []
      times = self.build_time_array(T,dt)
      for t in times:
        (s,s_dot,s_ddot) = self.sample_common_progress(profile,t)
        # q(t)
        q = (q_start + s * delta_q )
        # q_dot(t)
        q_dot = (s_dot * delta_q)
        # q_ddot(t)
        q_ddot = (s_ddot* delta_q)
        trajectory.append(
          {
            "time": float(t),
            "position": q,
            "velocity": q_dot,
            "acceleration": q_ddot,
            "s": s,
            "s_dot": s_dot,
            "s_ddot": s_ddot
          }
        )
      return trajectory

  #Python trajectory -> ROS2 JointTrajectory
  def create_joint_trajectory_msg(self,trajectory_points,joint_names):
      "ROS2 关节轨迹函数 负责将采样点转换成ROS2消息"
      #创建消息
      msg = JointTrajectory()
      #增加 header 时间戳
      msg.header.stamp = (
          self.get_clock()
          .now()
          .to_msg()
      )
      #填充关节名称
      msg.joint_names = joint_names
      #遍历轨迹点   100 hz  251个
      for point in trajectory_points:
          traj_point = JointTrajectoryPoint()
          traj_point.positions = (
              point["position"].tolist()
          )
          traj_point.velocities = (
              point["velocity"].tolist()
          )
          traj_point.accelerations = (
              point["acceleration"].tolist()
          )
          #设置时间 ROS2使用 Duration 而非 float 1.25
          # ROS2 Duration 要求 nanosec < 1e9
          # 1纳秒 = 1*10^-9 秒
          t = point["time"]
          # traj_point.time_from_start.sec = int(t)
          # traj_point.time_from_start.nanosec = int(round((t-int(t))*1e9))
          total_ns = int(round(t * 1_000_000_000))
          traj_point.time_from_start.sec = (
            total_ns // 1_000_000_000)
          traj_point.time_from_start.nanosec = (
            total_ns % 1_000_000_000)
          msg.points.append(traj_point)
      return msg

  # 公共 s(t) 实验
  def experiment_common_progress(self):
      """
      测试公共路径参数 s(t)
      测试案例：
      q_start = [0.0,  0.0, 0.0]
      q_goal  = [1.0, -0.5, 0.8]
      v_max = [0.5, 0.5, 0.5]
      a_max = [1.0, 1.0, 1.0]
      """
      # 初始关节位置
      q_start = np.array([0.0, 0.0, 0.0])
      # 目标关节位置
      q_goal = np.array([1.0, -0.5, 0.8])
      # 各关节最大速度
      v_max = np.array([0.5, 0.5, 0.5])
      # 各关节最大加速度
      a_max = np.array([1.0, 1.0, 1.0])
      # 计算公共路径参数 s(t) 的速度轨迹
      profile = self.calculate_common_progress_profile(
          q_start,
          q_goal,
          v_max,
          a_max
      )
      print(
        "\n===== 公共路径参数s(t) 下配置 =====\n"
        f"delta_q     : {profile['delta_q']}\n"
        f"s_dot_max   : {profile['s_dot_max']}\n"
        f"s_ddot_max  : {profile['s_ddot_max']}\n"
        f"profile      : {profile['profile_type']}\n"
        f"t_acc        : {profile['t_acc']}\n"
        f"t_cruise     : {profile['t_cruise']}\n"
        f"duration     : {profile['duration']}"
      )
      
      # 根据 s(t) 生成多关节轨迹
      trajectory = self.sample_common_joint_trajectory(
          q_start,
          q_goal,
          profile,
          dt=0.01
      )
      # 检查轨迹点数量
      print("轨迹点数量:",len(trajectory))
      # 测试几个关键时刻
      for t_test in [0.0,0.5,1.25,2.5]:
          (s,s_dot,s_ddot) = self.sample_common_progress(
              profile,
              t_test
          )
          q = (q_start + s *(q_goal - q_start))
          print(
              "\n===== 测试点 =====\n"
              f"time   : {t_test}\n"
              f"s      : {s}\n"
              f"s_dot  : {s_dot}\n"
              f"s_ddot : {s_ddot}\n"
              f"q      : {q}"
          )
      msg = (self.create_joint_trajectory_msg(
              trajectory,
              [
                  'joint1',
                  'joint2',
                  'joint3'
              ]))
      self.trajectory_pub.publish(msg)
      self.get_logger().info(
            '\n'
            '===== 公共 s(t) 轨迹已发布 =====\n'
            f'points: {len(msg.points)}'
        )

  def experiment_independent_sync(self):
      "独立关节 + 同时到达实验"
      q_start = np.array([0.0,0.0,0.0])
      q_goal = np.array([1.0,-0.5,0.8])
      max_velocity = np.array([0.5,0.5,0.5])
      max_acceleration = np.array([1.0,1.0,1.0])
      result = (
        self.calculate_multi_joint_profile(
          q_start,
          q_goal,
          max_velocity,
          max_acceleration
        )
      )
      sync_profiles = (result['profiles'])
      T_sync = (result['duration'])
      trajectory = (
        self.sample_multi_joint_trajectory(
          sync_profiles,
          dt=0.01
        )
      )
      self.get_logger().info(
          '\n'
          '===== 独立关节同步轨迹 =====\n'
          f'duration: {T_sync}\n'
          f'points: {len(trajectory)}'
      )
      msg = (
          self.create_joint_trajectory_msg(
            trajectory,
            [
              'joint1',
              'joint2',
              'joint3'
            ]
          )
      )
      self.trajectory_pub.publish(msg)
      self.get_logger().info('独立轨迹已发布.')


  def run_test_once(self):
         # 只执行一次
        self.test_timer.cancel()
        if self.test_mode == 'common':
          self.experiment_common_progress()
        elif self.test_mode == 'independent':
          self.experiment_independent_sync()
        else:
          self.get_logger().warning(
            f'未知 test_mode: {self.test_mode}'
          )

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()

          