"""Monitor planar three-link arm kinematics and velocity mappings."""

import math

from geometry_msgs.msg import Twist
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class KinematicsMonitor(Node):
    """Monitor forward kinematics and inverse-velocity solutions."""

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

    def calculate_joint_limit_velocity(self, q, jacobian, jacobian_pinv):
        """Return a bounded null-space velocity away from joint limits."""
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

        secondary_velocity = -gain * joint_limit_gradient
        null_space_projector = np.eye(3) - jacobian_pinv @ jacobian
        null_space_velocity = null_space_projector @ secondary_velocity

        max_speed = 1.0
        max_value = float(np.max(np.abs(null_space_velocity)))
        if max_value > max_speed:
            null_space_velocity *= max_speed / max_value

        return null_space_velocity, gain, nearest_distance

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
        """Compare MP, null-space, and adaptive-DLS velocity solutions."""
        if self.current_J is None or self.current_q is None:
            self.get_logger().warn('No joint state available yet.')
            return

        jacobian = self.current_J
        q = self.current_q
        desired_velocity = np.array([
            msg.linear.x,
            msg.linear.y
        ])

        jacobian_pinv = np.linalg.pinv(jacobian)
        q_dot_mp = jacobian_pinv @ desired_velocity
        x_dot_mp = jacobian @ q_dot_mp

        q_dot_null, secondary_gain, distance_to_limit = (
            self.calculate_joint_limit_velocity(
                q,
                jacobian,
                jacobian_pinv
            )
        )
        q_dot_total = q_dot_mp + q_dot_null
        x_dot_total = jacobian @ q_dot_total
        null_space_residual = jacobian @ q_dot_null

        singular_values = np.linalg.svd(jacobian, compute_uv=False)
        sigma_min = float(np.min(singular_values))
        if sigma_min >= self.sigma_threshold:
            damping_lambda = 0.0
        else:
            ratio = 1.0 - sigma_min / self.sigma_threshold
            damping_lambda = self.lambda_max * ratio ** 2

        if damping_lambda == 0.0:
            q_dot_dls = q_dot_mp.copy()
        else:
            lambda_sq = damping_lambda ** 2
            matrix = (
                jacobian @ jacobian.T
                + lambda_sq * np.eye(jacobian.shape[0])
            )
            q_dot_dls = (
                jacobian.T
                @ np.linalg.solve(matrix, desired_velocity)
            )
        x_dot_dls = jacobian @ q_dot_dls

        self.get_logger().info(
            '\n'
            f'Desired Cartesian velocity: {desired_velocity}\n'
            f'MP joint velocity: {q_dot_mp}\n'
            f'MP achieved velocity: {x_dot_mp}\n'
            f'Null-space joint velocity: {q_dot_null}\n'
            f'Null-space residual: {null_space_residual}\n'
            f'Total joint velocity: {q_dot_total}\n'
            f'Total achieved velocity: {x_dot_total}\n'
            f'Distance to nearest limit: {distance_to_limit:.6f}\n'
            f'Secondary gain: {secondary_gain:.6f}\n'
            f'Adaptive DLS lambda: {damping_lambda:.6f}\n'
            f'DLS joint velocity: {q_dot_dls}\n'
            f'DLS achieved velocity: {x_dot_dls}'
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
