import math

from geometry_msgs.msg import Twist
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class KinematicsMonitor(Node):

    def __init__(self):
        super().__init__('kinematics_monitor')

        # 两根连杆长度，和当前 URDF 一致
        self.L1 = 0.5
        self.L2 = 0.4

        # 保存最近一次计算结果
        self.latest_result = None
        self.previous_theta1 = None
        self.previous_theta2 = None
        self.previous_time = None
        self.current_J = None

        # 实验用的 DLS 阻尼参数
        # self.damping_lambda = 0.05

        # 自适应 DLS 参数
        # sigma_min 大于这个值时，不使用阻尼
        self.sigma_threshold = 0.10
        # 最大阻尼
        self.lambda_max = 0.05

        # 订阅 /desired_cartesian_velocity 期望笛卡尔速度  即末端速度
        self.desired_velocity_subscription = self.create_subscription(
            Twist,
            '/desired_cartesian_velocity',
            self.desired_velocity_callback,
            10
        )

        # 订阅 /joint_states
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # 每 0.5 秒把最新计算结果打印一次   普通状态信息
        # self.timer = self.create_timer(
        #     0.5,
        #     self.print_result
        # )

        self.get_logger().info('Kinematics monitor started.')

    def joint_state_callback(self, msg):

        # 确认消息里存在我们需要的两个关节
        if 'joint1' not in msg.name or 'joint2' not in msg.name:
            return

        # 找到 joint1、joint2 在数组中的位置
        joint1_index = msg.name.index('joint1')
        joint2_index = msg.name.index('joint2')

        # 防止 position 数组长度不足
        if len(msg.position) <= max(joint1_index, joint2_index):
            return

        # -----------------------------
        # 1. 从 /joint_states 得到关节角度 q
        # -----------------------------

        theta1 = msg.position[joint1_index]
        theta2 = msg.position[joint2_index]
        # -----------------------------
        # 2. Joint velocity q_dot 关节速度 q一点
        # -----------------------------
        current_time = self.get_clock().now().nanoseconds * 1e-9
        theta1_dot = None
        theta2_dot = None
        velocity_source = None
        # 如果 /joint_states 本身带有 velocity，就直接使用
        if len(msg.velocity) > max(joint1_index, joint2_index):

            theta1_dot = msg.velocity[joint1_index]
            theta2_dot = msg.velocity[joint2_index]

            velocity_source = 'JointState velocity'
        # 如果没有 velocity，就通过 position 的变化估算
        elif self.previous_time is not None:

            dt = current_time - self.previous_time

            if dt > 0.0:
                theta1_dot = (
                    theta1 - self.previous_theta1
                ) / dt

                theta2_dot = (
                    theta2 - self.previous_theta2
                ) / dt

                velocity_source = 'finite difference'
        # 保存这一次的数据，留给下一次计算
        self.previous_theta1 = theta1
        self.previous_theta2 = theta2
        self.previous_time = current_time
        # -----------------------------
        # 3. Forward Kinematics 正运动学
        # -----------------------------

        x = (
            self.L1 * math.cos(theta1)
            + self.L2 * math.cos(theta1 + theta2)
        )

        y = (
            self.L1 * math.sin(theta1)
            + self.L2 * math.sin(theta1 + theta2)
        )

        # -----------------------------
        # 4. Jacobian 雅克比矩阵
        # -----------------------------

        J = np.array([
            [
                -self.L1 * math.sin(theta1)
                - self.L2 * math.sin(theta1 + theta2),

                -self.L2 * math.sin(theta1 + theta2)
            ],
            [
                self.L1 * math.cos(theta1)
                + self.L2 * math.cos(theta1 + theta2),

                self.L2 * math.cos(theta1 + theta2)
            ]
        ])
        # 当前姿态雅可比
        self.current_J = J
        # -----------------------------
        # End-effector velocity 末端速度
        #
        # x_dot = J(q) q_dot
        # -----------------------------
        end_effector_velocity = None

        if theta1_dot is not None and theta2_dot is not None:

            q_dot = np.array([
                theta1_dot,
                theta2_dot
            ])

            end_effector_velocity = J @ q_dot
            # 观察差分速度
            # if abs(theta1_dot) > 1e-4 or abs(theta2_dot) > 1e-4:
            #     self.get_logger().info(
            #         '\n'
            #         f'q_dot = [{theta1_dot:.4f}, {theta2_dot:.4f}] rad/s, '
            #         f'joint1 contribution = '
            #         f'[{velocity_from_joint1[0]:.4f}, '
            #         f'{velocity_from_joint1[1]:.4f}] m/s\n'
            #         f'joint2 contribution = '
            #         f'[{velocity_from_joint2[0]:.4f}, '
            #         f'{velocity_from_joint2[1]:.4f}] m/s\n'
            #         f'total end-effector velocity = '
            #         f'[{end_effector_velocity[0]:.4f}, '
            #         f'{end_effector_velocity[1]:.4f}] m/s'
            #     )

        # -----------------------------
        # 5. Singular Value Decomposition  SVD奇异值分解
        # -----------------------------

        singular_values = np.linalg.svd(
            J,
            compute_uv=False
        )

        sigma_max = np.max(singular_values)
        sigma_min = np.min(singular_values)

        # -----------------------------
        # 6. Condition Number 条件数：机械臂在最好运动方向和最差运动方向的比值
        # -----------------------------

        if sigma_min < 1e-9:
            condition_number = math.inf
        else:
            condition_number = sigma_max / sigma_min

        # -----------------------------
        # 7. Manipulability 可操作度：整体机械臂的运动能力，越大越好
        #
        # w = sqrt(det(J J^T))=σ1*σ2*....
        # -----------------------------

        JJ_T = J @ J.T

        determinant = np.linalg.det(JJ_T)

        # 防止浮点误差造成非常小的负数
        determinant = max(determinant, 0.0)

        manipulability = math.sqrt(determinant)

        # 保存本次结果
        self.latest_result = {
            'theta1': theta1,
            'theta2': theta2,
            'theta1_dot': theta1_dot,
            'theta2_dot': theta2_dot,
            'x': x,
            'y': y,
            'J': J,
            'end_effector_velocity': end_effector_velocity,
            'velocity_source': velocity_source,
            'sigma_max': sigma_max,
            'sigma_min': sigma_min,
            'condition_number': condition_number,
            'manipulability': manipulability
        }

    def desired_velocity_callback(self, msg):

        if self.current_J is None:
            self.get_logger().warn(
                'No Jacobian available yet.'
            )
            return

        J = self.current_J

        # -----------------------------
        # Desired Cartesian velocity 期望笛卡尔速度
        # -----------------------------

        x_dot_d = np.array([
            msg.linear.x,       # 线性 X  Desired linear velocity in X
            msg.linear.y        # 线性 Y
        ])

        # -----------------------------
        # 1. Moore-Penrose pseudoinverse  穆尔-彭罗斯伪逆
        # -----------------------------

        J_pinv = np.linalg.pinv(J)

        q_dot_mp = J_pinv @ x_dot_d
        mp_norm = np.linalg.norm(q_dot_mp)

        # 实际能实现出的末端速度
        x_dot_mp = J @ q_dot_mp

        # -----------------------------
        # Singular values decomposition  SVD奇异值分解
        # -----------------------------
        singular_values = np.linalg.svd(
            J,
            compute_uv=False
        )
        sigma_min = np.min(singular_values)
        # -----------------------------
        # Adaptive damping  自适应阻尼
        # -----------------------------
        if sigma_min >= self.sigma_threshold:
            damping_lambda = 0.0
        else:
            ratio = (
                1.0
                - sigma_min / self.sigma_threshold
            )
            damping_lambda = (
                self.lambda_max
                * ratio ** 2
            )

        # -----------------------------
        # Damped Least Squares  阻尼最小二乘法  阻尼伪逆
        #
        # q_dot =
        # (J^T J + lambda^2 I)^(-1)
        # J^T x_dot_d
        # -----------------------------
        lambda_sq = damping_lambda ** 2
        A = (
            J.T @ J
            + lambda_sq * np.eye(2)
        )
        b = J.T @ x_dot_d
        q_dot_dls = np.linalg.solve(A, b)
        dls_norm = np.linalg.norm(q_dot_dls)
        # 实际能实现出的末端速度
        x_dot_dls = J @ q_dot_dls

        # -----------------------------
        # Print result  打印结果
        # -----------------------------

        self.get_logger().info(
            '\n'
            f'Desired Cartesian velocity:\n'
            f'x_dot_d = '
            f'[{x_dot_d[0]:.4f}, '
            f'{x_dot_d[1]:.4f}] m/s\n'
            f'\n'
            f'MP pseudoinverse:\n'
            f'q_dot = '
            f'[{q_dot_mp[0]:.4f}, '
            f'{q_dot_mp[1]:.4f}] rad/s\n'
            f'achieved x_dot = '
            f'[{x_dot_mp[0]:.4f}, '
            f'{x_dot_mp[1]:.4f}] m/s\n'
            f'\n'
            f'Adaptive DLS:\n'
            f'sigma_threshold = {self.sigma_threshold:.6f}\n'
            f'lambda = {damping_lambda:.6f}\n'
            f'q_dot = '
            f'[{q_dot_dls[0]:.4f}, '
            f'{q_dot_dls[1]:.4f}] rad/s\n'
            f'achieved x_dot = '
            f'[{x_dot_dls[0]:.4f}, '
            f'{x_dot_dls[1]:.4f}] m/s\n'
            f'||q_dot_MP|| = {mp_norm:.6f} rad/s\n'
            f'||q_dot_DLS|| = {dls_norm:.6f} rad/s\n'
            f'\n'
            f'sigma_min = '
            f'{sigma_min:.6f}'

        )

    def print_result(self):

        if self.latest_result is None:
            return

        r = self.latest_result

        J = r['J']
        velocity_text = 'Joint velocity: waiting for data\n'
        if r['end_effector_velocity'] is not None:

            v = r['end_effector_velocity']

            velocity_text = (
                f'q_dot = '
                f"[{r['theta1_dot']:.4f}, "
                f"{r['theta2_dot']:.4f}] rad/s\n"
                f'End-effector velocity: '
                f'x_dot = {v[0]:.4f} m/s, '
                f'y_dot = {v[1]:.4f} m/s\n'
                f'Velocity source: '
                f"{r['velocity_source']}\n"
            )
        self.get_logger().info(
            '\n'
            f"q = [{r['theta1']:.4f}, {r['theta2']:.4f}] rad\n"
            f'{velocity_text}'
            f"End-effector: x = {r['x']:.4f} m, "
            f"y = {r['y']:.4f} m\n"
            f'Jacobian:\n'
            f'[{J[0, 0]: .4f}  {J[0, 1]: .4f}]\n'
            f'[{J[1, 0]: .4f}  {J[1, 1]: .4f}]\n'
            f"sigma_max = {r['sigma_max']:.6f}\n"
            f"sigma_min = {r['sigma_min']:.6f}\n"
            f"condition number = {r['condition_number']}\n"
            f"manipulability = {r['manipulability']:.6f}"
        )


def main(args=None):

    rclpy.init(args=args)

    node = KinematicsMonitor()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
