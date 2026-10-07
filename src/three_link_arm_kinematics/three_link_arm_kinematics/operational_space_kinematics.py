"""平面3R机械臂的操作空间运动学."""
import numpy as np
class OperationalSpaceKinematics:
  """平面 3R 任务空间运动学."""

  def __init__(self,L1=0.5,L2=0.4, L3=0.4,):
    self.L1 = float(L1)
    self.L2 = float(L2)
    self.L3 = float(L3)

  def forward_kinematics(self, q):
    """返回末端执行器位置 [x, y]."""
    q = np.asarray(q, dtype=float)
    if q.shape != (3,):
      raise ValueError(f'q must have shape (3,), got {q.shape}')
    q1, q2, q3 = q
    q12 = q1 + q2
    q123 = q12 + q3
    x = ( self.L1 * np.cos(q1) + self.L2 * np.cos(q12) + self.L3 * np.cos(q123))
    y = (self.L1 * np.sin(q1) + self.L2 * np.sin(q12) + self.L3 * np.sin(q123))
    return np.array([x, y])

  def jacobian(self, q):
    """返回平面平移雅可比矩阵 J(q)。"""
    q = np.asarray(q, dtype=float)
    if q.shape != (3,):
      raise ValueError(f'q must have shape (3,), got {q.shape}')
    q1, q2, q3 = q
    q12 = q1 + q2
    q123 = q12 + q3
    J = np.array([
      [
        -self.L1 * np.sin(q1)- self.L2 * np.sin(q12)- self.L3 * np.sin(q123),

        -self.L2 * np.sin(q12) - self.L3 * np.sin(q123),

        -self.L3 * np.sin(q123),
      ],
      [
        self.L1 * np.cos(q1) + self.L2 * np.cos(q12) + self.L3 * np.cos(q123),

        self.L2 * np.cos(q12) + self.L3 * np.cos(q123),

        self.L3 * np.cos(q123),
      ],
    ])
    return J

  def jacobian_dot(self, q, q_dot):
    """返回雅可比矩阵的时间导数 J_dot(q, q_dot)。"""
    q = np.asarray(q, dtype=float)
    q_dot = np.asarray(q_dot, dtype=float)

    if q.shape != (3,):
      raise ValueError( f'q must have shape (3,), got {q.shape}')
    
    if q_dot.shape != (3,):
      raise ValueError(f'q_dot must have shape (3,), got {q_dot.shape}')
    
    q1, q2, q3 = q
    q1_dot, q2_dot, q3_dot = q_dot
    q12 = q1 + q2
    q123 = q12 + q3
    q12_dot = (q1_dot + q2_dot)
    q123_dot = (q1_dot + q2_dot + q3_dot)
    J_dot = np.array([
      [
        -self.L1 * np.cos(q1) * q1_dot- self.L2 * np.cos(q12) * q12_dot- self.L3 * np.cos(q123) * q123_dot,

        -self.L2 * np.cos(q12) * q12_dot- self.L3 * np.cos(q123) * q123_dot,

        -self.L3 * np.cos(q123) * q123_dot,
      ],
      [
        -self.L1 * np.sin(q1) * q1_dot- self.L2 * np.sin(q12) * q12_dot- self.L3 * np.sin(q123) * q123_dot,

        -self.L2 * np.sin(q12) * q12_dot- self.L3 * np.sin(q123) * q123_dot,

        -self.L3 * np.sin(q123) * q123_dot,
      ],
    ])
    return J_dot

  def task_velocity(self, q, q_dot):
    """返回 x_dot = J(q) q_dot."""
    q_dot = np.asarray(q_dot, dtype=float)
    return (self.jacobian(q)@ q_dot)

  def task_acceleration(self,q,q_dot,q_ddot,):
    """
    返回: x_ddot = J q_ddot + J_dot q_dot
    """

    q_dot = np.asarray(q_dot, dtype=float)
    q_ddot = np.asarray(q_ddot, dtype=float)

    J = self.jacobian(q)
    J_dot = self.jacobian_dot( q, q_dot,)

    x_ddot_from_q_ddot = ( J @ q_ddot )
    x_ddot_from_jacobian = ( J_dot @ q_dot)
    x_ddot = ( x_ddot_from_q_ddot + x_ddot_from_jacobian)

    return x_ddot