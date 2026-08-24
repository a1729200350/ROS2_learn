"""Monitor planar three-link arm kinematics and velocity mappings."""

import math

from geometry_msgs.msg import Twist
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class KinematicsMonitor(Node):
    """Monitor kinematics, redundancy, and inverse-velocity experiments."""

    def __init__(self):
        super().__init__('kinematics_monitor')

        # 三根连杆长度，和当前 URDF 一致
        self.L1 = 0.5
        self.L2 = 0.4
        self.L3 = 0.4

        self.latest_result = None
        self.previous_q = None
        self.previous_time = None
        self.current_q = None
        self.current_J = None

        # 自适应 DLS 参数
        self.sigma_threshold = 0.10
        self.lambda_max = 0.05

        self.desired_velocity_subscription = self.create_subscription(
            Twist,
            '/desired_cartesian_velocity',
            self.desired_velocity_callback,
            10
        )
        self.joint_state_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.get_logger().info('Kinematics monitor started.')

    def calculate_jacobian(self, q):
        """Return the planar translational Jacobian for joint vector q."""
        theta1, theta2, theta3 = q
        theta12 = theta1 + theta2
        theta123 = theta12 + theta3

        return np.array([
            [
                -self.L1 * math.sin(theta1)
                - self.L2 * math.sin(theta12)
                - self.L3 * math.sin(theta123),
                -self.L2 * math.sin(theta12)
                - self.L3 * math.sin(theta123),
                -self.L3 * math.sin(theta123)
            ],
            [
                self.L1 * math.cos(theta1)
                + self.L2 * math.cos(theta12)
                + self.L3 * math.cos(theta123),
                self.L2 * math.cos(theta12)
                + self.L3 * math.cos(theta123),
                self.L3 * math.cos(theta123)
            ]
        ])

    def calculate_manipulability(self, q):
        """Return Yoshikawa manipulability for joint vector q."""
        jacobian = self.calculate_jacobian(q)
        determinant = np.linalg.det(jacobian @ jacobian.T)
        return math.sqrt(max(float(determinant), 0.0))

    def manipulability_gradient(self, q):
        """Estimate the manipulability gradient with central differences."""
        epsilon = 1e-4
        gradient = np.zeros(3)

        for index in range(3):
            q_plus = q.copy()
            q_minus = q.copy()
            q_plus[index] += epsilon
            q_minus[index] -= epsilon
            gradient[index] = (
                self.calculate_manipulability(q_plus)
                - self.calculate_manipulability(q_minus)
            ) / (2.0 * epsilon)

        return gradient

    def calculate_singularity_objective(self, q):
        """Return a secondary objective that increases manipulability."""
        singularity_gain = 0.1
        return singularity_gain * self.manipulability_gradient(q)

    def calculate_joint_limit_objective(self, q):
        """Return the joint-limit objective, gain, and nearest distance."""
        q_min = np.full(3, -math.pi)
        q_max = np.full(3, math.pi)
        epsilon = 0.01

        lower_distance = np.maximum(q - q_min, epsilon)
        upper_distance = np.maximum(q_max - q, epsilon)
        nearest_distance = float(np.min(np.minimum(
            lower_distance,
            upper_distance
        )))

        joint_limit_gradient = (
            -2.0 / lower_distance ** 3
            + 2.0 / upper_distance ** 3
        )

        base_gain = 0.01
        sensitivity = 0.01
        gain = float(np.clip(
            base_gain + sensitivity / nearest_distance,
            0.01,
            0.5
        ))
        objective = -gain * joint_limit_gradient

        return objective, gain, nearest_distance

    @staticmethod
    def limit_joint_velocity(velocity, max_speed=1.0):
        """Scale a joint-velocity vector without changing its direction."""
        bounded_velocity = velocity.copy()
        max_value = float(np.max(np.abs(bounded_velocity)))
        if max_value > max_speed:
            bounded_velocity *= max_speed / max_value
        return bounded_velocity

    @staticmethod
    def calculate_dls_pseudoinverse(jacobian, damping_lambda):
        """Return the adaptive damped-least-squares pseudoinverse."""
        if damping_lambda == 0.0:
            return np.linalg.pinv(jacobian)

        lambda_sq = damping_lambda ** 2
        matrix = (
            jacobian @ jacobian.T
            + lambda_sq * np.eye(jacobian.shape[0])
        )
        return (
            jacobian.T
            @ np.linalg.solve(matrix, np.eye(jacobian.shape[0]))
        )

    @staticmethod
    def projector_diagnostics(
            jacobian,
            mp_pseudoinverse,
            dls_pseudoinverse,
            secondary_objective,
            desired_velocity):
        """Compare strict MP and soft DLS null-space operators."""
        identity = np.eye(jacobian.shape[1])
        projector_mp = identity - mp_pseudoinverse @ jacobian
        projector_dls = identity - dls_pseudoinverse @ jacobian

        null_velocity_mp = projector_mp @ secondary_objective
        null_velocity_dls = projector_dls @ secondary_objective
        achieved_null_mp = jacobian @ null_velocity_mp
        achieved_null_dls = jacobian @ null_velocity_dls

        q_dot_dls = dls_pseudoinverse @ desired_velocity
        achieved_dls = jacobian @ q_dot_dls
        achieved_total_mp = jacobian @ (q_dot_dls + null_velocity_mp)
        achieved_total_dls = jacobian @ (q_dot_dls + null_velocity_dls)

        return {
            'projector_mp': projector_mp,
            'projector_dls': projector_dls,
            'jn_mp_norm': np.linalg.norm(jacobian @ projector_mp),
            'jn_dls_norm': np.linalg.norm(jacobian @ projector_dls),
            'mp_idempotency_error': np.linalg.norm(
                projector_mp @ projector_mp - projector_mp
            ),
            'dls_idempotency_error': np.linalg.norm(
                projector_dls @ projector_dls - projector_dls
            ),
            'mp_secondary_leak': np.linalg.norm(achieved_null_mp),
            'dls_secondary_leak': np.linalg.norm(achieved_null_dls),
            'dls_task_error': np.linalg.norm(
                achieved_dls - desired_velocity
            ),
            'mp_total_task_error': np.linalg.norm(
                achieved_total_mp - desired_velocity
            ),
            'dls_total_task_error': np.linalg.norm(
                achieved_total_dls - desired_velocity
            )
        }

    @staticmethod
    def task_hierarchy_diagnostics(
            jacobian,
            mp_pseudoinverse,
            q_dot_task1,
            tertiary_objective):
        """Evaluate projected and unprojected secondary-task solutions."""
        identity = np.eye(jacobian.shape[1])
        projector1 = identity - mp_pseudoinverse @ jacobian

        # 二级任务：期望 q2_dot=0.2、q3_dot=-0.2
        jacobian2 = np.array([
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0]
        ])
        desired2 = np.array([0.2, -0.2])
        residual2 = desired2 - jacobian2 @ q_dot_task1

        # 对比实验：直接使用 J2 伪逆后再投影
        wrong_correction = (
            projector1
            @ np.linalg.pinv(jacobian2)
            @ residual2
        )
        wrong_total = q_dot_task1 + wrong_correction

        # 严格任务优先级：先构造有效雅可比 J2*N1
        effective_jacobian2 = jacobian2 @ projector1
        hierarchy_rcond = 1e-8
        effective_pseudoinverse2 = np.linalg.pinv(
            effective_jacobian2,
            rcond=hierarchy_rcond
        )
        correct_correction = (
            projector1
            @ effective_pseudoinverse2
            @ residual2
        )
        correct_total = q_dot_task1 + correct_correction
        achieved2 = jacobian2 @ correct_total
        task2_error = desired2 - achieved2

        # Task 1 和 Task 2 完成后的剩余零空间
        projector2 = (
            projector1
            - effective_pseudoinverse2 @ effective_jacobian2
        )
        task3_velocity = projector2 @ tertiary_objective

        return {
            'desired2': desired2,
            'effective_jacobian2': effective_jacobian2,
            'effective_rank2': np.linalg.matrix_rank(
                effective_jacobian2,
                tol=hierarchy_rcond
            ),
            'wrong_task1': jacobian @ wrong_total,
            'wrong_task2': jacobian2 @ wrong_total,
            'correct_task1': jacobian @ correct_total,
            'correct_task2': achieved2,
            'task2_error': task2_error,
            'task2_error_norm': np.linalg.norm(task2_error),
            'projector2_norm': np.linalg.norm(projector2),
            'task1_projector2_residual': np.linalg.norm(
                jacobian @ projector2
            ),
            'task2_projector2_residual': np.linalg.norm(
                jacobian2 @ projector2
            ),
            'task3_velocity_norm': np.linalg.norm(task3_velocity)
        }

    def joint_state_callback(self, msg):
        """Update kinematics from the latest three-joint state."""
        joint_names = ('joint1', 'joint2', 'joint3')
        if not all(name in msg.name for name in joint_names):
            return

        indices = [msg.name.index(name) for name in joint_names]
        max_index = max(indices)
        if len(msg.position) <= max_index:
            return

        q = np.array([msg.position[index] for index in indices])
        current_time = self.get_clock().now().nanoseconds * 1e-9
        q_dot = None
        velocity_source = None

        if len(msg.velocity) > max_index:
            q_dot = np.array([msg.velocity[index] for index in indices])
            velocity_source = 'JointState velocity'
        elif self.previous_time is not None:
            dt = current_time - self.previous_time
            if dt > 0.0:
                q_dot = (q - self.previous_q) / dt
                velocity_source = 'finite difference'

        self.previous_q = q.copy()
        self.previous_time = current_time
        self.current_q = q

        theta1, theta2, theta3 = q
        theta12 = theta1 + theta2
        theta123 = theta12 + theta3
        x = (
            self.L1 * math.cos(theta1)
            + self.L2 * math.cos(theta12)
            + self.L3 * math.cos(theta123)
        )
        y = (
            self.L1 * math.sin(theta1)
            + self.L2 * math.sin(theta12)
            + self.L3 * math.sin(theta123)
        )

        jacobian = self.calculate_jacobian(q)
        self.current_J = jacobian
        end_effector_velocity = None
        if q_dot is not None:
            end_effector_velocity = jacobian @ q_dot

        singular_values = np.linalg.svd(jacobian, compute_uv=False)
        sigma_max = float(np.max(singular_values))
        sigma_min = float(np.min(singular_values))
        condition_number = math.inf
        if sigma_min >= 1e-9:
            condition_number = sigma_max / sigma_min

        self.latest_result = {
            'q': q,
            'q_dot': q_dot,
            'x': x,
            'y': y,
            'J': jacobian,
            'end_effector_velocity': end_effector_velocity,
            'velocity_source': velocity_source,
            'sigma_max': sigma_max,
            'sigma_min': sigma_min,
            'condition_number': condition_number,
            'manipulability': self.calculate_manipulability(q)
        }

    def desired_velocity_callback(self, msg):
        """Run inverse-velocity and task-priority experiments."""
        if self.current_J is None or self.current_q is None:
            self.get_logger().warn('No joint state available yet.')
            return

        jacobian = self.current_J
        q = self.current_q
        desired_velocity = np.array([
            msg.linear.x,
            msg.linear.y
        ])

        mp_pseudoinverse = np.linalg.pinv(jacobian)
        q_dot_mp = mp_pseudoinverse @ desired_velocity
        achieved_mp = jacobian @ q_dot_mp

        z_limit, secondary_gain, distance_to_limit = (
            self.calculate_joint_limit_objective(q)
        )
        z_singularity = self.calculate_singularity_objective(q)
        secondary_objective = z_limit + z_singularity

        projector_mp = (
            np.eye(jacobian.shape[1])
            - mp_pseudoinverse @ jacobian
        )
        q_dot_null_raw = projector_mp @ secondary_objective
        q_dot_null = self.limit_joint_velocity(q_dot_null_raw)

        singular_values = np.linalg.svd(jacobian, compute_uv=False)
        sigma_min = float(np.min(singular_values))
        if sigma_min >= self.sigma_threshold:
            damping_lambda = 0.0
        else:
            ratio = 1.0 - sigma_min / self.sigma_threshold
            damping_lambda = self.lambda_max * ratio ** 2

        dls_pseudoinverse = self.calculate_dls_pseudoinverse(
            jacobian,
            damping_lambda
        )
        q_dot_dls = dls_pseudoinverse @ desired_velocity
        achieved_dls = jacobian @ q_dot_dls

        q_dot_total = q_dot_dls + q_dot_null
        achieved_total = jacobian @ q_dot_total
        null_space_residual = jacobian @ q_dot_null

        projector_results = self.projector_diagnostics(
            jacobian,
            mp_pseudoinverse,
            dls_pseudoinverse,
            secondary_objective,
            desired_velocity
        )
        hierarchy_results = self.task_hierarchy_diagnostics(
            jacobian,
            mp_pseudoinverse,
            q_dot_mp,
            z_singularity
        )

        self.get_logger().info(
            '\n'
            f'Desired Cartesian velocity: {desired_velocity}\n'
            f'MP joint velocity: {q_dot_mp}\n'
            f'MP achieved velocity: {achieved_mp}\n'
            f'DLS lambda: {damping_lambda:.6f}\n'
            f'DLS joint velocity: {q_dot_dls}\n'
            f'DLS achieved velocity: {achieved_dls}\n'
            f'Joint-limit objective: {z_limit}\n'
            f'Singularity objective: {z_singularity}\n'
            f'Combined secondary objective: {secondary_objective}\n'
            f'Raw null-space velocity: {q_dot_null_raw}\n'
            f'Bounded null-space velocity: {q_dot_null}\n'
            f'Null-space residual: {null_space_residual}\n'
            f'Total joint velocity: {q_dot_total}\n'
            f'Total achieved velocity: {achieved_total}\n'
            f'Distance to nearest limit: {distance_to_limit:.6f}\n'
            f'Secondary gain: {secondary_gain:.6f}\n'
            f"||J*N_MP||: {projector_results['jn_mp_norm']:.10e}\n"
            f"||J*N_DLS||: {projector_results['jn_dls_norm']:.10e}\n"
            f'MP idempotency error: '
            f"{projector_results['mp_idempotency_error']:.10e}\n"
            f'DLS idempotency error: '
            f"{projector_results['dls_idempotency_error']:.10e}\n"
            f'MP secondary leak: '
            f"{projector_results['mp_secondary_leak']:.10e}\n"
            f'DLS secondary leak: '
            f"{projector_results['dls_secondary_leak']:.10e}\n"
            f'Pure DLS task error: '
            f"{projector_results['dls_task_error']:.10e}\n"
            f'DLS + MP-null task error: '
            f"{projector_results['mp_total_task_error']:.10e}\n"
            f'DLS + DLS-null task error: '
            f"{projector_results['dls_total_task_error']:.10e}\n"
            '===== Strict task hierarchy =====\n'
            f"Desired Task 2: {hierarchy_results['desired2']}\n"
            f"Effective J2 rank: {hierarchy_results['effective_rank2']}\n"
            f"Wrong method Task 1: {hierarchy_results['wrong_task1']}\n"
            f"Wrong method Task 2: {hierarchy_results['wrong_task2']}\n"
            f'Correct method Task 1: '
            f"{hierarchy_results['correct_task1']}\n"
            f'Correct method Task 2: '
            f"{hierarchy_results['correct_task2']}\n"
            f"Task 2 residual: {hierarchy_results['task2_error']}\n"
            f'Task 2 residual norm: '
            f"{hierarchy_results['task2_error_norm']:.10e}\n"
            f"||N2||: {hierarchy_results['projector2_norm']:.10e}\n"
            f'||J1*N2||: '
            f"{hierarchy_results['task1_projector2_residual']:.10e}\n"
            f'||J2*N2||: '
            f"{hierarchy_results['task2_projector2_residual']:.10e}\n"
            f'||N2*z3||: '
            f"{hierarchy_results['task3_velocity_norm']:.10e}"
        )

    def print_result(self):
        """Print the most recently calculated kinematic state."""
        if self.latest_result is None:
            return

        result = self.latest_result
        jacobian = result['J']
        velocity_text = 'Joint velocity: waiting for data\n'
        if result['end_effector_velocity'] is not None:
            velocity_text = (
                f"q_dot = {result['q_dot']} rad/s\n"
                f'End-effector velocity = '
                f"{result['end_effector_velocity']} m/s\n"
                f"Velocity source: {result['velocity_source']}\n"
            )

        self.get_logger().info(
            '\n'
            f"q = {result['q']} rad\n"
            f'{velocity_text}'
            f"End-effector: x = {result['x']:.4f} m, "
            f"y = {result['y']:.4f} m\n"
            f'Jacobian:\n{jacobian}\n'
            f"sigma_max = {result['sigma_max']:.6f}\n"
            f"sigma_min = {result['sigma_min']:.6f}\n"
            f"condition number = {result['condition_number']}\n"
            f"manipulability = {result['manipulability']:.6f}"
        )


def main(args=None):
    """Run the kinematics monitor node."""
    rclpy.init(args=args)
    node = KinematicsMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
