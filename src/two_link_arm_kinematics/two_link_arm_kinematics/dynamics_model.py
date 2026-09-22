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

    self.I1 = 0.0208333
    self.I2 = 0.0107
    self.I3 = 0.0107

    self.g = 9.81

  # 质量矩阵 M(q)
  def calculate_mass_matrix(self, q):
    q1,q2,q3 = q
    c2 = np.cos(q2)
    c3 = np.cos(q3)
    c23 = np.cos(q2+q3)

    M = np.zeros((3,3))

    M[0,0] = (
      1.4652
      +0.5*c2
      +0.2*c23
    )

    M[0,1] = (
      0.66166
      +0.2*c2
    )

    M[0,2] = (
      0.18683
      +0.05*c23
    )

    M[1,0] = M[0,1]

    M[1,1] = (
      0.34166
      +0.1*c3
    )


    M[1,2] = (
      0.10683
      +0.05*c3
    )

    M[2,0] = M[0,2]

    M[2,1] = M[1,2]

    M[2,2] = 0.04283

    return M

  # 重力项 G(q)
  def calculate_gravity( self,q):
    q1,q2,q3 = q

    G1 = (
      -16.5789*np.cos(q1)
    )

    G2 = (
      -6.2784*np.cos(q1+q2)
    )

    G3 = (
      -1.5696*np.cos(q1+q2+q3)
    )

    return np.array(
      [
        G1,
        G2,
        G3
      ]
    )

  # 速度项 V(q,q_dot)
  def calculate_velocity_term(self,q,q_dot):
    q1,q2,q3 = q
    dq1,dq2,dq3 = q_dot
    s2 = np.sin(q2)
    s3 = np.sin(q3)
    V1 = (
      -0.8 * s2 *
      (
        2*dq1*dq2
        +
        dq2*dq2
      )
    )

    V2 = (
      0.8
      *
      s2
      *
      dq1*dq1
    )

    V3 = (
      0.3
      *
      s3
      *
      dq2*dq2
    )

    return np.array(
      [
        V1,
        V2,
        V3
      ]
    )

  # 完整逆动力学
  def calculate_inverse_dynamics( self, q, q_dot, q_ddot):
    M = self.calculate_mass_matrix(
      q
    )
    V = self.calculate_velocity_term(
      q,
      q_dot
    )
    G = self.calculate_gravity(q)
    tau = ( M @ q_ddot + V + G)
    return tau  