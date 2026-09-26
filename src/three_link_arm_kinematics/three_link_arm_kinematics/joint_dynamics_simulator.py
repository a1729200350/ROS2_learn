import rclpy
import time
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from three_link_arm_kinematics.dynamics_model import DynamicsModel
class JointDynamicsSimulator(Node):
    def __init__(self):
        super().__init__("joint_dynamics_simulator")
        self.model = DynamicsModel()
        # 状态
        self.q = np.zeros(3)
        self.q_dot = np.zeros(3)

        # 力矩
        self.tau = np.zeros(3)
        # 最近一次收到有效力矩的时间
        self.last_tau_time = None
        # 力矩命令超时阈值，单位：秒
        self.tau_timeout = 0.1
        # 超时后锁定暂停，本轮不自动恢复
        self.command_timed_out = False

        # 时间步长
        self.dt = 0.01
        # 接收力矩
        self.tau_sub = self.create_subscription(
        Float64MultiArray,
        "/joint_effort_command",
        self.tau_callback,
        10
        )
        # 发布关节状态
        self.state_pub = self.create_publisher(
        JointState,
        "/joint_states",
        10
        )
        # 100Hz
        self.timer = self.create_timer(
        self.dt,
        self.update
        )
        self.get_logger().info("关节动力学仿真器已启动")

    def tau_callback(self,msg):
        # self.tau = np.array(
        # msg.data,
        # dtype=float
        # )

        # 超时后，本轮不再接受命令恢复运动
        if self.command_timed_out:
            return
        tau = np.asarray(msg.data, dtype=float)
        # 无效消息不能更新力矩，也不能刷新接收时间
        if tau.shape != (3,) or not np.all(np.isfinite(tau)):
            self.get_logger().warning(
                "忽略无效力矩命令：需要三个有限数值。"
            )
            return
        self.tau = tau
        self.last_tau_time = time.monotonic()

    # def update(self):
    #     q = self.q
    #     q_dot = self.q_dot
    #     # 动力学模型
    #     M = self.model.calculate_mass_matrix(q)
    #     V = self.model.calculate_velocity_term(q, q_dot)
    #     G = self.model.calculate_gravity(q)
    #     # M qdd = tau - V - G
    #     q_ddot = np.linalg.solve(M,self.tau - V - G)
    #     # 欧拉积分
    #     self.q_dot += q_ddot * self.dt  
    #     self.q += self.q_dot * self.dt
    #     # 发布状态
    #     msg = JointState()
    #     msg.header.stamp = (
    #         self.get_clock().now().to_msg()
    #     )
    #     msg.name = [
    #         "joint1",
    #         "joint2",
    #         "joint3"
    #     ]
    #     msg.position = (
    #         self.q.tolist()
    #     )
    #     msg.velocity = (
    #         self.q_dot.tolist()
    #     )
    #     self.state_pub.publish(
    #         msg
    #     )

    def update(self):
        # 收到第一条有效命令后，才允许积分
        can_integrate = (
            self.last_tau_time is not None
            and not self.command_timed_out
        )

        if can_integrate:
            command_age = time.monotonic() - self.last_tau_time

            if command_age > self.tau_timeout:
                self.command_timed_out = True
                can_integrate = False

                self.get_logger().error(
                    f"力矩命令已中断 {command_age:.3f} s，"
                    "动力学积分已暂停。"
                    "请停止本轮节点，重新启动实验。"
                )

        if can_integrate:
            q = self.q
            q_dot = self.q_dot

            # 动力学模型
            M = self.model.calculate_mass_matrix(q)
            V = self.model.calculate_velocity_term(q, q_dot)
            G = self.model.calculate_gravity(q)

            # M q_ddot = tau - V - G
            q_ddot = np.linalg.solve(
                M,
                self.tau - V - G
            )

            # 保留原有积分顺序
            self.q_dot += q_ddot * self.dt
            self.q += self.q_dot * self.dt

        # 等待命令或超时暂停时，也继续发布保存的状态
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["joint1", "joint2", "joint3"]
        msg.position = self.q.tolist()
        msg.velocity = self.q_dot.tolist()

        self.state_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointDynamicsSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()