"""平面3R机械臂的操作空间动力学."""
import numpy as np
from .operational_space_kinematics import (OperationalSpaceKinematics,)
from .dynamics_model import (DynamicsModel,)
class OperationalSpaceDynamics:
  """平面3R机械臂的操作空间动力学量."""

  def __init__(self,dynamics=None):
    self.kinematics = (OperationalSpaceKinematics())
    self.dynamics = (
      DynamicsModel()
        if dynamics is None
        else dynamics
      )
  
  def calculate_operational_inertia(self, q,):
    """
    计算: Lambda = (J M^-1 J^T)^-1
    """
    q = np.asarray( q, dtype=float,)
    J = (self.kinematics.jacobian(q))
    M = ( self.dynamics.calculate_mass_matrix(q))
    # M X = J^T
    # X = M^-1 J^T
    M_inv_JT = np.linalg.solve( M,J.T,)
    task_inverse_inertia = (J @ M_inv_JT)
    Lambda = np.linalg.solve(task_inverse_inertia,np.eye( task_inverse_inertia.shape[0]),)
    return {
      'J': J,
      'M': M,
      'M_inv_JT': M_inv_JT,
      'task_inverse_inertia':task_inverse_inertia,
      'Lambda': Lambda,
    }

  def calculate_dynamic_consistent_inverse(self,q,):
    """
    计算动态一致广义逆:
    J_bar  = M^-1 J^T Lambda
    """
    result = (self.calculate_operational_inertia(q))
    J_bar = (result['M_inv_JT'] @ result['Lambda'])
    return {**result, 'J_bar': J_bar,}

  def calculate_operational_bias(self,q,q_dot,):
    """
    计算操作空间偏差项:
      mu = Lambda J M^-1 V - Lambda J_dot q_dot
      p  = Lambda J M^-1 G
    """
    q = np.asarray(q, dtype=float,)
    q_dot = np.asarray(q_dot,dtype=float,)

    # Lambda、J、M
    result = (self.calculate_operational_inertia(q))
    J = result['J']
    M = result['M']
    Lambda = result['Lambda']

    # J_dot
    J_dot = (self.kinematics.jacobian_dot(q,q_dot,))

    # 关节空间动力学项
    V = (self.dynamics.calculate_velocity_term(q,q_dot,))
    G = (self.dynamics.calculate_gravity(q))

    # 不显式计算 M^-1
    M_inv_V = np.linalg.solve(M,V,)
    M_inv_G = np.linalg.solve(M,G,)

    # mu = Lambda J M^-1 V - Lambda J_dot q_dot
    mu_from_dynamics = ( Lambda @ J @ M_inv_V )
    mu_from_geometry = ( -Lambda @ J_dot @ q_dot )
    mu = ( mu_from_dynamics + mu_from_geometry )

    # p = Lambda J M^-1 G
    p = ( Lambda @ J @ M_inv_G )

    return {
      **result,
      'J_dot': J_dot,
      'V': V,
      'G': G,
      'M_inv_V': M_inv_V,
      'M_inv_G': M_inv_G,
      'mu_from_dynamics':mu_from_dynamics,
      'mu_from_geometry':mu_from_geometry,
      'mu': mu,
      'p': p,
    }

  def calculate_task_force(self,q,q_dot,x_ddot_command,):
    """
    Calculate:
      F = Lambda x_ddot_command + mu + p
    """

    x_ddot_command = np.asarray(x_ddot_command,dtype=float,)
    result = (self.calculate_operational_bias( q, q_dot,))
    Lambda = result['Lambda']
    mu = result['mu']
    p = result['p']
    force_inertia = ( Lambda @ x_ddot_command )
    F = ( force_inertia + mu + p )
    tau = ( result['J'].T @ F )

    return {
      **result,
      'x_ddot_command': x_ddot_command,
      'force_inertia': force_inertia,
      'F': F,
      'tau': tau,
    }

  def calculate_null_space_projector(self,q,):
    """
    进行动态一致性计算
    零空间投影器:
    N   = I - J_bar J
    N_T = N^T
        = I - J^T J_bar^T
    """
    result = (self.calculate_dynamic_consistent_inverse(q))
    J = result['J']
    J_bar = result['J_bar']
    N = ( np.eye(J.shape[1]) - J_bar @ J )
    N_T = N.T
    return {
      **result,
      'N': N,
      'N_T': N_T,
    }

  