import numpy as np
from three_link_arm_kinematics.joint_space_controller import (JointSpaceController)
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

    # 用手算结果检查位置误差
    np.testing.assert_allclose(
      e,
      [0.1, -0.05, 0.1],
      rtol=0.0,
      atol=1e-10,
      err_msg='位置误差计算错误'
    )
    # 用手算结果检查速度误差
    np.testing.assert_allclose(
      e_dot,
      [0.4, -0.15, 0.2],
      rtol=0.0,
      atol=1e-10,
      err_msg='速度误差计算错误'
    )
    # tau_pd = Kp @ e + Kd @ e_dot
    np.testing.assert_allclose(
      tau_pd,
      [1.8, -0.8, 1.4],
      rtol=0.0,
      atol=1e-10,
      err_msg='PD 力矩计算错误'
    )

    print('PD 控制测试通过：位置误差、速度误差、力矩均符合预期。')