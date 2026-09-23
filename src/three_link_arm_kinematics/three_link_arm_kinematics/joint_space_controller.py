import numpy as np
class JointSpaceController:
  def __init__(self, kp, kd):
    # 初始化关节空间 PD 控制器
    # kp:三个关节的位置反馈增益
    # kd:三个关节的速度反馈增益
    # 将 kp、kd 转换为 3×3 对角增益矩阵：
    #  Kp = diag(kp1, kp2, kp3)
    #  Kd = diag(kd1, kd2, kd3)
      # 位置误差增益矩阵 Kp
      # np.diag() 将 [kp1, kp2, kp3] 转换成 3×3 对角矩阵
    self.Kp = np.diag(
      np.asarray(kp, dtype=float)
    )
      # 速度误差增益矩阵 Kd
    self.Kd = np.diag(
      np.asarray(kd, dtype=float)
    )

  def calculate_pd_torque(self, q, q_dot, q_d, q_dot_d):
    """
    关节空间 PD 控制：
        e     = q_d - q
        e_dot = q_dot_d - q_dot
        tau_pd = Kp e + Kd e_dot
    """
    #实际关节状态：
      # q     = 实际关节位置
      # q_dot = 实际关节速度
    #期望关节状态:
      # q_d     = 期望关节位置
      # q_dot_d = 期望关节速度
    q = np.asarray(q, dtype=float)
    q_dot = np.asarray(q_dot, dtype=float)
    q_d = np.asarray(q_d, dtype=float)
    q_dot_d = np.asarray( q_dot_d, dtype=float)
    # 位置误差
    e = q_d - q
    # 速度误差
    e_dot = q_dot_d - q_dot
    # PD 控制力矩
    tau_p = self.Kp @ e
    tau_d = self.Kd @ e_dot
    # 总 PD 反馈力矩:
    # tau_pd = Kp(q_d - q) + Kd(q_dot_d - q_dot) 
    tau_pd = tau_p + tau_d
    return tau_pd, e, e_dot

  def calculate_computed_torque(self,q,q_dot,q_d,q_dot_d,tau_ff):
    tau_pd, e, e_dot = (
      self.calculate_pd_torque(q,q_dot,q_d,q_dot_d)
    )
    tau_total = tau_ff + tau_pd
    return (tau_total,tau_ff,tau_pd,e,e_dot)

  def calculate_classical_computed_torque(self,model,q,q_dot,q_d,q_dot_d,q_ddot_d):
    e = q_d - q
    e_dot = q_dot_d - q_dot
    # 加速度修正
    q_ddot_cmd = (
      q_ddot_d
      + self.Kd @ e_dot
      + self.Kp @ e
    )
    M = model.calculate_mass_matrix(q)
    V = model.calculate_velocity_term(q, q_dot)
    G = model.calculate_gravity(q)
    tau = (M @ q_ddot_cmd+ V+ G)
    return tau, e, e_dot, q_ddot_cmd

  def calculate_control_torque(self,q,q_dot,q_d,q_dot_d,tau_ff):
    """ 逆动力学前馈 + PD 

      tau = tau_ff + Kp (q_d - q) + Kd (q_dot_d - q_dot)
    """
    tau_ff = np.asarray(tau_ff,dtype=float)
    tau_pd, e, e_dot = (self.calculate_pd_torque( q, q_dot, q_d, q_dot_d ))
    tau = tau_ff + tau_pd
    return tau, tau_ff, tau_pd, e, e_dot

  