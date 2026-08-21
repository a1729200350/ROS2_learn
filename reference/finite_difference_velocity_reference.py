"""差分关节速度的参考实现，不会被当前 ROS 2 节点自动调用."""

import math


class FiniteDifferenceVelocityEstimator:
    """使用消息时间、角度展开和低通滤波估算关节速度."""

    def __init__(
            self,
            minimum_dt=1e-4,
            maximum_dt=0.5,
            filter_alpha=0.25,
            maximum_speeds=(2.0, 2.0, 2.0)):
        """保存差分范围、滤波系数和三个关节的速度上限."""
        self.minimum_dt = minimum_dt
        self.maximum_dt = maximum_dt
        self.filter_alpha = filter_alpha
        self.maximum_speeds = maximum_speeds

        self.previous_angles = None
        self.previous_time = None
        self.filtered_velocities = None

    @staticmethod
    def shortest_angular_difference(current_angle, previous_angle):
        """返回位于负 pi 到正 pi 之间的最短角度差."""
        angle_difference = current_angle - previous_angle
        return math.atan2(
            math.sin(angle_difference),
            math.cos(angle_difference)
        )

    def reset(self, angles, current_time):
        """使用当前采样重新建立差分基准."""
        self.previous_angles = tuple(angles)
        self.previous_time = current_time
        self.filtered_velocities = None

    def update(self, angles, current_time):
        """返回三个关节的滤波速度；数据无效或首帧时返回 None."""
        angles = tuple(angles)

        if self.previous_time is None:
            self.reset(angles, current_time)
            return None

        dt = current_time - self.previous_time

        # 时间未前进或间隔太短时，保留旧基准，等待下一帧。
        if dt <= self.minimum_dt:
            return None

        # 通信间隔太长时放弃本帧速度，只重新建立基准。
        if dt > self.maximum_dt:
            self.reset(angles, current_time)
            return None

        raw_velocities = tuple(
            self.shortest_angular_difference(current, previous) / dt
            for current, previous in zip(angles, self.previous_angles)
        )

        # 超过 URDF 速度上限时，不输出错误速度，并从当前姿态重新开始。
        if any(
                abs(velocity) > maximum_speed
                for velocity, maximum_speed in zip(
                    raw_velocities, self.maximum_speeds)):
            self.reset(angles, current_time)
            return None

        if self.filtered_velocities is None:
            filtered_velocities = raw_velocities
        else:
            alpha = self.filter_alpha
            filtered_velocities = tuple(
                alpha * raw + (1.0 - alpha) * previous_filtered
                for raw, previous_filtered in zip(
                    raw_velocities, self.filtered_velocities)
            )

        self.previous_angles = angles
        self.previous_time = current_time
        self.filtered_velocities = filtered_velocities

        return filtered_velocities


def joint_state_time_seconds(msg, fallback_time_seconds):
    """优先读取 JointState 时间戳；时间戳为空时使用节点当前时间."""
    stamp = msg.header.stamp

    if stamp.sec == 0 and stamp.nanosec == 0:
        return fallback_time_seconds

    return stamp.sec + stamp.nanosec * 1e-9


# 以后手动集成到 kinematics_monitor.py 时：
#
# 1. 在 KinematicsMonitor.__init__ 中创建：
#
#    self.velocity_estimator = FiniteDifferenceVelocityEstimator()
#
# 2. 在 joint_state_callback 中读取 theta1、theta2、theta3 后计算时间：
#
#    fallback_time = self.get_clock().now().nanoseconds * 1e-9
#    current_time = joint_state_time_seconds(msg, fallback_time)
#
# 3. 当 msg.velocity 没有三个关节速度时调用：
#
#    estimated_velocities = self.velocity_estimator.update(
#        (theta1, theta2, theta3),
#        current_time
#    )
#
#    if estimated_velocities is not None:
#        theta1_dot, theta2_dot, theta3_dot = estimated_velocities
#        velocity_source = 'filtered finite difference'
#
# 当前文件只是参考，不会自动改变或运行 kinematics_monitor.py。
