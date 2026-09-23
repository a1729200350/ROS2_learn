""""监测3R平面关节机械臂动力特性。"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from three_link_arm_kinematics.dynamics_model import (DynamicsModel)
class DynamicsMonitor(Node):

  def __init__(self):
    super().__init__('dynamics_monitor')

    # # ---------- 机器人参数 ----------
    # # 连杆长度 [m]
    # self.L1 = 0.5
    # self.L2 = 0.4
    # self.L3 = 0.4

    # # 各个关节到连杆质心的距离 [m]
    # self.r1 = 0.25
    # self.r2 = 0.20
    # self.r3 = 0.20

    # # 连杆质量 [kg]
    # self.m1 = 1.0
    # self.m2 = 0.8
    # self.m3 = 0.8

    # # 绕各质心z轴的转动惯量 [kg*m^2]
    # self.I1 = 0.02104
    # self.I2 = 0.01083
    # self.I3 = 0.01083

    self.model = DynamicsModel()
    self.joint_state_subscription = self.create_subscription(
      JointState,
      '/joint_states',
      self.joint_state_callback,
      10
    )
    #增加轨迹订阅器
    self.trajectory_subscription = self.create_subscription(
      JointTrajectory,
      '/joint_trajectory_controller/joint_trajectory',
      self.trajectory_callback,
      10
    )
    # ---------- 最新实际关节状态缓存 ----------
    self.latest_q = None
    self.latest_q_dot = None
    # ---------- 动力学监测定时器 ---------
    # 每 0.5 s 执行一次，即 2 Hz
    self.monitor_timer = self.create_timer(
      0.5,
      self.monitor_dynamics
  )

    self.get_logger().info('动力学监视器启动.')

  # def calculate_mass_matrix(self, q):
  #   """计算3x3空间关节质量矩阵 M(q)."""
  #   q1, q2, q3 = q
  #   q12 = q1 + q2
  #   q123 = q12 + q3

  #   s1 = math.sin(q1)
  #   c1 = math.cos(q1)

  #   s12 = math.sin(q12)
  #   c12 = math.cos(q12)

  #   s123 = math.sin(q123)
  #   c123 = math.cos(q123)

  #   # 连杆1 的质心雅可比矩阵
  #   Jv1 = np.array([
  #       [-self.r1 * s1, 0.0, 0.0],
  #       [ self.r1 * c1, 0.0, 0.0]
  #   ])

  #   Jw1 = np.array([
  #       [1.0, 0.0, 0.0]
  #   ])

  #   # 连杆2 的质心雅克比矩阵
  #   Jv2 = np.array([
  #     [
  #       -self.L1 * s1 - self.r2 * s12,
  #       -self.r2 * s12,
  #       0.0
  #     ],
  #     [
  #       self.L1 * c1 + self.r2 * c12,
  #       self.r2 * c12,
  #       0.0
  #     ]
  #   ])

  #   Jw2 = np.array([
  #       [1.0, 1.0, 0.0]
  #   ])

  #   # 连杆3 的质心雅克比矩阵
  #   Jv3 = np.array([
  #     [
  #       -self.L1 * s1
  #       - self.L2 * s12
  #       - self.r3 * s123,

  #       -self.L2 * s12
  #       - self.r3 * s123,

  #       -self.r3 * s123
  #     ],
  #     [
  #       self.L1 * c1
  #       + self.L2 * c12
  #       + self.r3 * c123,

  #       self.L2 * c12
  #       + self.r3 * c123,

  #       self.r3 * c123
  #     ]
  #   ])

  #   Jw3 = np.array([
  #     [1.0, 1.0, 1.0]
  #   ])

  #   # 单独连杆的贡献
  #   # M_i = m_i Jv_i^T Jv_i + I_i Jw_i^T Jw_i
  #   M1 = (
  #       self.m1 * Jv1.T @ Jv1
  #       + self.I1 * Jw1.T @ Jw1
  #   )

  #   M2 = (
  #       self.m2 * Jv2.T @ Jv2
  #       + self.I2 * Jw2.T @ Jw2
  #   )

  #   M3 = (
  #       self.m3 * Jv3.T @ Jv3
  #       + self.I3 * Jw3.T @ Jw3
  #   )

  #   M = M1 + M2 + M3
  #   return M

  # def calculate_gravity(self, q):
  #   """
  #   计算重力项 G(q)
  #   """
  #   q1, q2, q3 = q
  #   q12 = q1 + q2
  #   q123 = q12 + q3
  #   s1 = math.sin(q1)
  #   c1 = math.cos(q1)
  #   s12 = math.sin(q12)
  #   c12 = math.cos(q12)
  #   s123 = math.sin(q123)
  #   c123 = math.cos(q123)

  #   # 重力方向
  #   g = np.array([
  #     -9.81,
  #     0.0,
  #   ])

  #   # 连杆1 的质心雅克比矩阵
  #   Jv1 = np.array([
  #     [-self.r1*s1,0,0],
  #     [ self.r1*c1,0,0]
  #   ])

  #   # 连杆2 的质心雅克比矩阵
  #   Jv2 = np.array([
  #     [
  #     -self.L1*s1-self.r2*s12,
  #     -self.r2*s12,
  #     0
  #     ],
  #     [
  #     self.L1*c1+self.r2*c12,
  #     self.r2*c12,
  #     0
  #     ]
  #   ])

  #   # 连杆3 的质心雅克比矩阵
  #   Jv3=np.array([
  #     [
  #     -self.L1*s1-self.L2*s12-self.r3*s123,
  #     -self.L2*s12-self.r3*s123,
  #     -self.r3*s123
  #     ],
  #     [
  #     self.L1*c1+self.L2*c12+self.r3*c123,
  #     self.L2*c12+self.r3*c123,
  #     self.r3*c123
  #     ]
  #   ])

  #   Q_g = (
  #     self.m1*Jv1.T@g
  #     +
  #     self.m2*Jv2.T@g
  #     +
  #     self.m3*Jv3.T@g
  #   )
  #   G=-Q_g

  #   return G

  # def calculate_velocity_term(self, q, q_dot, epsilon=1e-6):
  #   """
  #   计算速度相关项 V(q, q_dot)。

  #   V_i = sum_j sum_k gamma_ijk * q_dot_j * q_dot_k
  #   """
  #   # gamma_ijk =1/2 * (dM_ij/dq_k + dM_ik/dq_j - dM_jk/dq_i )
  #   #epsolon=1e-6  用于计算数值导数的微小增量
  #   #dtype=float  确保输入是浮点数数组  进行浮点运算
  #   q = np.asarray(q, dtype=float)
  #   q_dot = np.asarray(q_dot, dtype=float)
  #   #共有 三个关节
  #   n = 3

  #   # dM_dq[k, i, j] 表示：
  #   #
  #   #       ∂M_ij
  #   #       -----
  #   #        ∂q_k
  #   # 第一个索引 k：对哪个关节变量求偏导
  #   # 第二个索引 i：M 的第几行
  #   # 第三个索引 j：M 的第几列
  #   #数学编号是 1,2,3，Python 编号是 0,1,2
  #   dM_dq = np.zeros((n, n, n))
  #   # 使用中心差分计算 ∂M/∂q_k
  #   for k in range(n):
  #     dq = np.zeros(n)
  #     #每个关节只扰动一次
  #     dq[k] = epsilon
  #     M_plus = self.model.calculate_mass_matrix(q + dq)
  #     M_minus = self.model.calculate_mass_matrix(q - dq)
  #     dM_dq[k] = ( M_plus - M_minus ) / (2.0 * epsilon)
  #   # 根据 gamma_ijk 计算 V_i
  #   V = np.zeros(n)
  #   for i in range(n):
  #     for j in range(n):
  #       for k in range(n):
  #         gamma_ijk = 0.5 * (dM_dq[k, i, j] + dM_dq[j, i, k] - dM_dq[i, j, k])
  #         V[i] += ( gamma_ijk * q_dot[j] * q_dot[k] )

  #   return V

  # def calculate_inverse_dynamics(self, q, q_dot, q_ddot):
  #   """
  #   计算逆动力学关节力矩：

  #       tau = M(q) q_ddot + V(q, q_dot) + G(q)
  #   """
  #   q = np.asarray(q, dtype=float)
  #   q_dot = np.asarray(q_dot, dtype=float)
  #   q_ddot = np.asarray(q_ddot, dtype=float)

  #   M = self.model.calculate_mass_matrix(q)
  #   V = self.calculate_velocity_term(q, q_dot)
  #   G = self.calculate_gravity(q)

  #   tau_inertia = M @ q_ddot

  #   tau = tau_inertia + V + G

  #   return tau

  def joint_state_callback(self, msg):

    """    
    从 /joint_states 读取实际关节位置 q 和速度 q_dot，

    计算当前动力学量 M(q)、V(q,q_dot)、G(q)。
    """
    #/joint_states 不直接提供 q_ddot， 所以暂时不计算完整的逆动力学力矩
    joint_names = ('joint1', 'joint2', 'joint3')

    if not all(name in msg.name for name in joint_names):
      return
    #将关节名称 与 joint_state 中的索引对应起来 并通过for name in joint_names 进行排序
    indices = [
      msg.name.index(name)
      for name in joint_names
    ]

    max_index = max(indices)
    if len(msg.position) <= max_index:
      return

    # q = np.array([
    #   msg.position[index]
    #   for index in indices
    # ], dtype=float)
    # # JointState 的速度有时可能为空
    # if len(msg.velocity) > max_index:
    #   q_dot = np.array([
    #     msg.velocity[index]
    #     for index in indices
    #   ], dtype=float)
    # else:
    #   q_dot = np.zeros(3)
    # # 动力学
    # M = self.calculate_mass_matrix(q)
    # V = self.calculate_velocity_term(
    #   q,
    #   q_dot
    # )
    # G = self.calculate_gravity(q)
    # #当前没有关节加速度 q_ddot ,所以计算基础力矩
    # tau_bias = V + G

    # #矩阵校验
    #   #对称误差  求范数=0  验证对称矩阵
    # symmetry_error = np.linalg.norm(M - M.T)
    # #计算特征值 λi>0  验证矩阵正定
    # #eigvalsh专门用于实对称矩阵 会直接返回实数特征值  数值计算更稳定
    # #eigvals 用于计算任意方阵 会带来 特征值可能负数  无序 计算效率低 数值稳定性一般  等问题
    # eigenvalues = np.linalg.eigvalsh(M)

    # self.get_logger().info(
    #     '\n'
    #     '========== 动力学 ==========\n'
    #     f'q = {q} rad\n'
    #     f'q_dot = {q_dot} rad/s\n\n'
    #     f'M(q) =\n{M}\n'
    #     f'V(q, q_dot) = {V}\n'
    #     f'G(q) = {G}\n'
    #     f'tau_bias = V + G = {tau_bias}\n\n'
    #     f'对称误差 ||M-M^T|| = {symmetry_error:.3e}\n'
    #     f'特征值 = {eigenvalues}'
    # )
    # 保存最新关节位置
    self.latest_q = np.array([
      msg.position[index]
      for index in indices
    ], dtype=float)
    # 保存最新关节速度
    if len(msg.velocity) > max_index:
      self.latest_q_dot = np.array([
        msg.velocity[index]
        for index in indices
      ], dtype=float)
    else:
      self.latest_q_dot = None

  def trajectory_callback(self, msg):

    """
    接收期望 JointTrajectory。

    对轨迹中的每个点计算：

        tau_ff =
            M(q_d) q_ddot_d + V(q_d, q_dot_d) + G(q_d)

    这里的 tau_ff 是逆动力学前馈力矩，
    暂时只计算，不发送给控制器。
    """

    joint_names = (
        'joint1',
        'joint2',
        'joint3'
    )
    # 检查 JointTrajectory 是否包含三个关节
    if not all(
      name in msg.joint_names
      for name in joint_names
    ):
      self.get_logger().warning(
        '轨迹中缺少 joint1/joint2/joint3'
      )
      return

    # 根据名称确定三个关节在消息中的顺序
    indices = [
      msg.joint_names.index(name)
      for name in joint_names
    ]

    if len(msg.points) == 0:
      self.get_logger().warning(
        '收到空 JointTrajectory'
      )
      return

    torque_trajectory = []

    # 遍历所有轨迹点
    for point_index, point in enumerate(msg.points):

      max_index = max(indices)
      # 完整逆动力学必须同时有 q、q_dot、q_ddot
      if len(point.positions) <= max_index:
        continue
      if len(point.velocities) <= max_index:
        continue
      if len(point.accelerations) <= max_index:
        continue

      q_d = np.array([
        point.positions[index]
        for index in indices
      ], dtype=float)

      q_dot_d = np.array([
        point.velocities[index]
        for index in indices
      ], dtype=float)

      q_ddot_d = np.array([
        point.accelerations[index]
        for index in indices
      ], dtype=float)

      tau_ff = self.model.calculate_inverse_dynamics(
        q_d,
        q_dot_d,
        q_ddot_d
      )

      time_from_start = (
        point.time_from_start.sec
        +
        point.time_from_start.nanosec
        * 1e-9
      )

      torque_trajectory.append(
        {
          'time': time_from_start,
          'q_d': q_d,
          'q_dot_d': q_dot_d,
          'q_ddot_d': q_ddot_d,
          'tau_ff': tau_ff
        }
      )

    if len(torque_trajectory) == 0:
      self.get_logger().warning(
        '轨迹中没有可用于完整逆动力学的有效点'
      )
      return

    # 整条轨迹的前馈力矩分析
    times = np.array([
      state['time']
      for state in torque_trajectory
    ])
    tau_array = np.vstack([
      state['tau_ff']
      for state in torque_trajectory
    ])
    # 每个关节最大绝对力矩所在的轨迹点
    max_abs_indices = np.argmax(np.abs(tau_array),axis=0)
    max_torque_log = (
      '\n'
      '========== 前馈力矩最大值 ==========\n'
    )

    for joint_index in range(3):
      point_index = max_abs_indices[joint_index]
      state = torque_trajectory[point_index]
      tau_value = state['tau_ff'][joint_index]
      max_torque_log += (
        f'\n'
        f'joint{joint_index + 1}:\n'
        f'  point = {point_index}\n'
        f'  t = {state["time"]:.3f} s\n'
        f'  tau = {tau_value:.6f} N*m\n'
        f'  |tau|max = {abs(tau_value):.6f} N*m\n'
        f'  q_d = {state["q_d"]}\n'
        f'  q_dot_d = {state["q_dot_d"]}\n'
        f'  q_ddot_d = {state["q_ddot_d"]}\n'
      )
    self.get_logger().info(max_torque_log)

    # 只显示几个代表点，避免一次打印 251 个点
    sample_indices = sorted(
        set([
            0,
            len(torque_trajectory) // 2,
            len(torque_trajectory) - 1
        ])
    )

    log_text = (
      '\n'
      '========== 期望轨迹逆动力学 ==========\n'
      f'有效轨迹点数量 = {len(torque_trajectory)}\n'
    )

    for index in sample_indices:

      state = torque_trajectory[index]

      log_text += (
        '\n'
        f'----- point {index} -----\n'
        f't = {state["time"]:.3f} s\n'
        f'q_d = {state["q_d"]} rad\n'
        f'q_dot_d = {state["q_dot_d"]} rad/s\n'
        f'q_ddot_d = {state["q_ddot_d"]} rad/s^2\n'
        f'前馈力矩 = {state["tau_ff"]} N*m\n'
      )

    self.get_logger().info(log_text)

  def monitor_dynamics(self):
    """
    以较低频率计算并输出当前实际状态的动力学量。
    """

    if self.latest_q is None:
      return
    if self.latest_q_dot is None:
      return

    # copy，得到当前时刻的一份状态快照
    q = self.latest_q.copy()
    q_dot = self.latest_q_dot.copy()

    # ---------- 动力学 ----------
    M = self.model.calculate_mass_matrix(q)
    V = self.model.calculate_velocity_term( q, q_dot)
    G = self.model.calculate_gravity(q)

    # 当前没有 q_ddot，因此这里只计算偏置力矩
    tau_bias = V + G

    # ---------- 质量矩阵检查 ----------
    symmetry_error = np.linalg.norm( M - M.T)

    eigenvalues = np.linalg.eigvalsh(M)

    self.get_logger().info(
        '\n'
        '========== 动力学 ==========\n'
        f'q = {q} rad\n'
        f'q_dot = {q_dot} rad/s\n\n'
        f'M(q) =\n{M}\n'
        f'V(q, q_dot) = {V}\n'
        f'G(q) = {G}\n'
        f'tau_bias = V + G = {tau_bias}\n\n'
        f'对称误差 ||M-M^T|| = '
        f'{symmetry_error:.3e}\n'
        f'特征值 = {eigenvalues}'
    )

def main(args=None):
  rclpy.init(args=args)

  node = DynamicsMonitor()

  rclpy.spin(node)

  node.destroy_node()
  rclpy.shutdown()
if __name__ == '__main__':
  main()
