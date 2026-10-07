"""平面3R机械臂操作空间加速度控制器."""
import numpy as np
from .operational_space_kinematics import (OperationalSpaceKinematics,)

class OperationalSpaceController:
  """二维任务空间中的解算加速度控制器."""
  def __init__(self,kp=(10.0, 10.0), kd=(4.43, 4.43),):
    self.kinematics = OperationalSpaceKinematics()
    self.Kp = np.diag( np.asarray(kp, dtype=float))
    self.Kd = np.diag( np.asarray(kd, dtype=float))

  def calculate_task_acceleration_command( self, q, q_dot, x_d, x_dot_d, x_ddot_d,):
    """
    计算: x_ddot_c = x_ddot_d + Kd * e_dot + Kp * e
    """
    q = np.asarray(q, dtype=float)
    q_dot = np.asarray(q_dot, dtype=float)
    x_d = np.asarray(x_d, dtype=float)
    x_dot_d = np.asarray( x_dot_d, dtype=float,)
    x_ddot_d = np.asarray( x_ddot_d, dtype=float, )

    x = ( self.kinematics.forward_kinematics(q))
    J = (self.kinematics.jacobian(q))
    x_dot = ( J @ q_dot )
    e_x = ( x_d - x )
    e_x_dot = ( x_dot_d - x_dot)
    x_ddot_command = ( x_ddot_d + self.Kd @ e_x_dot + self.Kp @ e_x)
    return {
      'x': x,
      'x_dot': x_dot,
      'e_x': e_x,
      'e_x_dot': e_x_dot,
      'x_ddot_command': x_ddot_command,
    }

  def calculate_joint_acceleration_command(self,q,q_dot,x_ddot_command,):
    """
    Calculate:  q_ddot_c = J^+ (x_ddot_c - J_dot q_dot)
    """
    q = np.asarray(q, dtype=float)
    q_dot = np.asarray(q_dot, dtype=float)
    x_ddot_command = np.asarray(x_ddot_command, dtype=float,)
    J = ( self.kinematics .jacobian(q))
    J_dot = (self.kinematics.jacobian_dot( q, q_dot,))
    J_pinv = np.linalg.pinv(J)
    geometric_acceleration = (J_dot @ q_dot)
    corrected_task_acceleration = ( x_ddot_command - geometric_acceleration )
    q_ddot_command = (J_pinv @ corrected_task_acceleration)

    return {
      'J': J,
      'J_dot': J_dot,
      'J_pinv': J_pinv,
      'geometric_acceleration': geometric_acceleration,
      'corrected_task_acceleration': corrected_task_acceleration,
      'q_ddot_command': q_ddot_command,
    }

  def calculate(self,q,q_dot,x_d, x_dot_d, x_ddot_d,):
    """计算完整的解耦加速度指令."""
    task_result = (self.calculate_task_acceleration_command(q, q_dot,x_d, x_dot_d, x_ddot_d,))
    joint_result = (self.calculate_joint_acceleration_command( q, q_dot, task_result[ 'x_ddot_command' ],))
    return { **task_result, **joint_result,}