import numpy as np
from two_link_arm_kinematics.joint_space_controller import (JointSpaceController)
if __name__ == '__main__':
    controller = JointSpaceController(
      kp=[10.0, 10.0, 10.0],
      kd=[2.0, 2.0, 2.0]
    )
    # 实际状态
    q = np.array([
      0.4,
      -0.2,
      0.3
    ])
    q_dot = np.array([
      0.1,
      -0.1,
      0.2
    ])
    # 期望状态
    q_d = np.array([
      0.5,
      -0.25,
      0.4
    ])
    q_dot_d = np.array([
      0.5,
      -0.25,
      0.4
    ])
    tau_pd, e, e_dot = (
      controller.calculate_pd_torque(
        q,
        q_dot,
        q_d,
        q_dot_d
      )
    )

    print('e =', e)
    print('e_dot =', e_dot)
    print('tau_pd =', tau_pd)