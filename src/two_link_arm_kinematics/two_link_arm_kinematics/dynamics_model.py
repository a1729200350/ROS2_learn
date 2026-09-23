"""
机器人动力学模型

负责：

M(q)
G(q)
V(q,q_dot)
tau

不包含 ROS2。

"""
import numpy as np
class DynamicsModel:
  def __init__(self):
    # 机器人参数
    self.L1 = 0.5
    self.L2 = 0.4
    self.L3 = 0.4

    self.m1 = 1.0
    self.m2 = 0.8
    self.m3 = 0.8

    self.r1 = 0.25
    self.r2 = 0.2
    self.r3 = 0.2

    self.I1 = 0.02104
    self.I2 = 0.01083
    self.I3 = 0.01083

    self.g = 9.81

  @staticmethod
  def _as_joint_vector(value, name):
    """ 将输入转换为包含三个有限数值的一维数组。 

    检查输入是不是三个有效数值，避免错误数据悄悄进入计算。
    """
    vector = np.asarray(value, dtype=float)
    if vector.shape != (3,):
      raise ValueError(
        f"{name} 必须是长度为 3 的一维数组，"
        f"当前形状为 {vector.shape}"
        )
    if not np.all(np.isfinite(vector)):
      raise ValueError(
        f"{name} 不能包含 NaN 或无穷大"
        )
    return vector

  def _center_of_mass_jacobians(self, q):
    """
    计算三个连杆质心的平移雅可比。

    每个矩阵形状为 2x3:

    两行对应局部平面的 x、y 方向；

    三列对应 joint1、joint2、joint3。
    """
    q = self._as_joint_vector(q, "q")
    q1, q2, q3 = q
    q12 = q1 + q2
    q123 = q12 + q3
    s1 = np.sin(q1)
    c1 = np.cos(q1)
    s12 = np.sin(q12)
    c12 = np.cos(q12)
    s123 = np.sin(q123)
    c123 = np.cos(q123)
    # 连杆 1 质心的平移雅可比
    Jv1 = np.array([
      [-self.r1 * s1, 0.0, 0.0],
      [ self.r1 * c1, 0.0, 0.0]
    ])
    # 连杆 2 质心的平移雅可比
    Jv2 = np.array([
      [
        -self.L1 * s1 - self.r2 * s12,
        -self.r2 * s12,
        0.0
      ],
      [
        self.L1 * c1 + self.r2 * c12,
        self.r2 * c12,
        0.0
      ]
    ])
    # 连杆 3 质心的平移雅可比
    Jv3 = np.array([
      [
        -self.L1 * s1 - self.L2 * s12 - self.r3 * s123,
        -self.L2 * s12 - self.r3 * s123,
        -self.r3 * s123
      ],
      [
        self.L1 * c1 + self.L2 * c12 + self.r3 * c123,
        self.L2 * c12 + self.r3 * c123,
        self.r3 * c123
      ]
    ])
    return Jv1, Jv2, Jv3

  def calculate_mass_matrix(self, q): 
    """计算关节空间质量矩阵 M(q),单位:kg·m²。"""
    Jv1, Jv2, Jv3 = self._center_of_mass_jacobians(q)
    # 平面机械臂各连杆的角速度雅可比
    Jw1 = np.array([[1.0, 0.0, 0.0]])
    Jw2 = np.array([[1.0, 1.0, 0.0]])
    Jw3 = np.array([[1.0, 1.0, 1.0]])
        # 每根连杆同时贡献平移动能和转动动能
    M1 = (
      self.m1 * (Jv1.T @ Jv1)
      + self.I1 * (Jw1.T @ Jw1)
    )
    M2 = (
      self.m2 * (Jv2.T @ Jv2)
      + self.I2 * (Jw2.T @ Jw2)
    )
    M3 = (
      self.m3 * (Jv3.T @ Jv3)
      + self.I3 * (Jw3.T @ Jw3)
    )
    return M1 + M2 + M3

  
  def calculate_gravity( self,q):
    """计算动力学方程右侧的重力项 G(q),单位:N·m。

    对应当前 URDF 安装姿态：
    假设基座与世界坐标对齐，世界重力沿 -Z,
    则重力在 arm_base 局部平面中沿 -X。

    静止保持时，所需补偿力矩 tau = G(q)。
    """
    Jv1, Jv2, Jv3 = self._center_of_mass_jacobians(q)
    # 局部平面中的重力加速度，单位：m/s²
    gravity = np.array([-self.g, 0.0])
    # 重力实际施加到关节上的广义力
    Q_g = (
      self.m1 * (Jv1.T @ gravity)
      + self.m2 * (Jv2.T @ gravity)
      + self.m3 * (Jv3.T @ gravity)
    )

    # 动力学方程： tau = M q_ddot + V + G 
    return -Q_g

  def calculate_velocity_term(self,q,q_dot, epsilon=1e-6):
    """
    根据质量矩阵的偏导数，计算速度相关力矩。

    包含科里奥利项和离心项。
    返回形状为 (3,) 的数组,单位:N·m。
    """
    q = self._as_joint_vector(q, "q")
    q_dot = self._as_joint_vector(q_dot, "q_dot")
    # epsilon 是计算数值偏导时使用的关节角扰动量
    if not np.isscalar(epsilon):
      raise ValueError("epsilon 必须是正的有限数值")
    try:
      epsilon = float(epsilon)
    except (TypeError, ValueError):
      raise ValueError(
        "epsilon 必须是正的有限数值"
      ) from None
    if not np.isfinite(epsilon) or epsilon <= 0.0:
      raise ValueError("epsilon 必须是正的有限数值")
    n = 3
    # dM_dq[k, i, j] = ∂M[i, j] / ∂q[k]
    dM_dq = np.zeros((n, n, n), dtype=float)
    # 对每个关节角分别做中心差分
    for k in range(n):
      delta_q = np.zeros(n, dtype=float)
      delta_q[k] = epsilon

      M_plus = self.calculate_mass_matrix(q + delta_q)
      M_minus = self.calculate_mass_matrix(q - delta_q)

      dM_dq[k] = (
        (M_plus - M_minus) / (2.0 * epsilon)
      )
    # V[i] = Σ_j Σ_k Γ[i,j,k] * q_dot[j] * q_dot[k]
    V = np.zeros(n, dtype=float)
    for i in range(n):
          for j in range(n):
            for k in range(n):
              gamma_ijk = 0.5 * (
                dM_dq[k, i, j]
                + dM_dq[j, i, k]
                - dM_dq[i, j, k]
              )
              V[i] += (gamma_ijk * q_dot[j] * q_dot[k])
    return V

  # 完整逆动力学
  def calculate_inverse_dynamics( self, q, q_dot, q_ddot):
    """
    根据关节位置、速度、加速度计算所需关节力矩。

    tau = M(q) @ q_ddot + V(q, q_dot) + G(q)

    输入单位：
      q:rad
      q_dot:rad/s
      q_ddot:rad/s²

    返回形状为 (3,) 的数组,单位：:·m。
    """
    q = self._as_joint_vector(q, "q")
    q_dot = self._as_joint_vector(q_dot, "q_dot")
    q_ddot = self._as_joint_vector(q_ddot, "q_ddot")

    M = self.calculate_mass_matrix(q)
    V = self.calculate_velocity_term(q,q_dot)
    G = self.calculate_gravity(q)
    tau = ( M @ q_ddot + V + G)
    return tau  