"""Monitor planar three-link arm kinematics and velocity mappings."""
import math
import numpy as np
import rclpy
import osqp
import scipy.sparse as sparse
from scipy.optimize import lsq_linear
from geometry_msgs.msg import Twist
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
        self.current_p_ee = None

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

    def apply_priority_task(
            self,
            q_dot_previous,
            N_previous,
            J_task,
            x_dot_task):

        # 1. 有效 Jacobian
        J_bar = J_task @ N_previous

        # 2. 当前任务剩余需求
        residual = (
            x_dot_task
            - J_task @ q_dot_previous
        )

        # 3. 有效 Jacobian 的 MP 伪逆
        J_bar_pinv = np.linalg.pinv(J_bar)

        # 4. 当前任务产生的关节速度修正
        delta_q_dot = (
            N_previous
            @ J_bar_pinv
            @ residual
        )

        # 5. 更新累计关节速度
        q_dot_new = (
            q_dot_previous
            + delta_q_dot
        )

        # 6. 更新剩余零空间
        N_new = (
            N_previous
            @ (
                np.eye(N_previous.shape[0])
                - J_bar_pinv @ J_bar
            )
        )

        return (
            q_dot_new,
            N_new,
            residual,
            J_bar
        )

    def calculate_manipulability(self, q):
        """Return Yoshikawa manipulability for joint vector q."""
        jacobian = self.calculate_jacobian(q)
        determinant = np.linalg.det(jacobian @ jacobian.T)
        return math.sqrt(max(float(determinant), 0.0))

    def manipulability_gradient(self, q):
        """Estimate the manipulability gradient with central differences.
            利用中心差分估计可操作性梯度"""
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

    def calculate_singularity(self,q,
        #jacobian,
        #jacobian_pinv
        ):
        # -----------------------------------------
        # 奇异位形避免二级任务
        # 目标：最大化机械臂可操作度 manipulability
        # w(q) = sqrt(det(J J^T))
        # 梯度方向：grad_w = ∇w(q)
        # 表示可操作度随关节变化的最快方向
        grad_w = self.manipulability_gradient(q)
        # 奇异值规避增益 kw越大 规避效果越强
        k_w = 0.1
        # 奇异规避二级任务的期望关节速度方向 沿着增加可操作度方向运动
        # z_singularity = k_w ∇w(q)
        z_singularity = k_w * grad_w
        # 零空间投影
        # N = I - J^+ J  作用：将二级任务限制在不影响主任务的零空间内 
        # J * N = 0
        #N = np.eye(3)-jacobian_pinv@jacobian                       Z_singlularity
        # 零空间奇异规避角速度
        # q_dot_null = N z
        #q_dot_null_singularity = N @ z_singularity                 Z_singlularity
        #################  测试
        # self.get_logger().info(
        #     f"\nManipulability gradient:\n{grad_w}"
        #     f"\nSingularity z:\n{z_singularity}"
        #     #f"\nNull velocity:\n{q_dot_null_singularity}"          Z_singlularity
        # )
        #################
        return z_singularity
        #q_dot_null_singularity                                     Z_singlularity

    def calculate_joint_limit(self, q, 
        #jacobian,         z_limit更改
        #jacobian_pinv     z_limit更改
        ):
        """Return the joint-limit avoidance secondary objective.
            计算关节限位避免二级任务 z_limit。"""
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

        #自适应增益  k_l gain  
        base_gain = 0.01   
        sensitivity = 0.01
        gain = float(np.clip(
            base_gain + sensitivity / nearest_distance,
            0.01,
            0.5
        ))
        # -----------------------------------------
        #   简单线性激活
        # -----------------------------------------
        
        d_safe = 1.0
        d_danger = 0.3
        # #定义最简单任务激活系数  
        # #α=(d_safe-d_min)/d_safe-d_danger
        # activation = (
        #     (d_safe - nearest_distance)
        #     / (d_safe - d_danger)
        # )

        # activation = float(
        #     np.clip(activation, 0.0, 1.0)
        # )
        #控制器余弦平滑处理任务激活系数
        #s=(d_safe-d_min)/d_safe-d_danger
        #α=0.5[1-cos(πs)]  s∈[0,1]
        # d_min >= d_safe   -> α = 0
        # d_min <= d_danger -> α = 1
        # 中间区域           -> 0 < α < 1
        # -----------------------------------------
        s = (
            (d_safe - nearest_distance)
            / (d_safe - d_danger)
        )
        #将 s限制在0 ~ 1之间 防止因为余弦周期性得到错误数值 
        s = float(
            np.clip(s, 0.0, 1.0)
        )

        activation = 0.5 * (
            1.0 - math.cos(math.pi * s)
        )
        #只输出z_limit更改
        # secondary_velocity = -gain * joint_limit_gradient
        # null_space_projector = np.eye(3) - jacobian_pinv @ jacobian
        # null_space_velocity = null_space_projector @ secondary_velocity

        # max_speed = 1.0
        # max_value = float(np.max(np.abs(null_space_velocity)))
        # if max_value > max_speed:
        #     null_space_velocity *= max_speed / max_value

        z_limit = -activation*gain * joint_limit_gradient

        return z_limit,gain, nearest_distance,activation
        #null_space_velocity,       z_limit更改
        
    def calculate_joint_position_velocity_bounds(self, q, dt):
        """根据关节位置限制和关节速度限制，计算当前时刻允许的关节速度上下界。"""

        q_min = np.full(3, -math.pi)
        q_max = np.full(3, math.pi)
        q_dot_max = np.full(3, 0.05)
        # 原始关节速度限制  
        # -q_dot_max <= q_dot <= q_dot_max
        lower_velocity = -q_dot_max
        upper_velocity = q_dot_max
        # 位置限制转换成速度限制
        # q_next = q + q_dot * dt
        # q_min <= q_next <= q_max
        # 得：
        # (q_min - q)/dt <= q_dot <= (q_max - q)/dt
        lower_position = (
            q_min - q
        ) / dt
        upper_position = (
            q_max - q
        ) / dt
        # 合并位置限制和速度限制
        # 下界取更大的值
        # 上界取更小的值
        # 即取两个约束区间的交集
        lower_bound = np.maximum(
            lower_velocity,
            lower_position
        )
        upper_bound = np.minimum(
            upper_velocity,
            upper_position
        )
        return lower_bound, upper_bound,q_max,upper_position    

    def calculate_joint_velocity_damper_bounds(self, q):
        """根据距离关节限位的距离，计算速度阻尼器速度边界。"""
        q_min = np.full(3, -math.pi)
        q_max = np.full(3,  math.pi)
        #原始的最大关节速度
        q_dot_max = np.full(3, 0.05)
        # -----------------------------------------
        # Velocity Damper  速度阻尼器      参数
        #
        # d_safe:  安全距离
        # 距离限位小于该值时开始减速
        #
        # eta: η 阻尼器 最大允许速度
        # 当前设成与 q_dot_max 相同
        # -----------------------------------------
        d_safe = 0.5
        eta = 0.05
        lower_bound = -q_dot_max.copy()
        upper_bound =  q_dot_max.copy()
        for i in range(len(q)):
            # 距离上限
            d_upper = q_max[i] - q[i]
            if d_upper < d_safe:
                upper_bound[i] = min(
                    upper_bound[i],
                    eta * d_upper / d_safe
                )
            # 距离下限
            d_lower = q[i] - q_min[i]
            if d_lower < d_safe:
                lower_bound[i] = max(
                    lower_bound[i],
                    -eta * d_lower / d_safe
                )
        return lower_bound, upper_bound

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

        #末端位置
        p_ee = np.array([x,y])
        self.current_p_ee = p_ee

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
        """Compare MP, null-space, and adaptive-DLS velocity solutions.
            比较 MP 零空间 和自适应DLS速度解决方案"""
        if self.current_J is None or self.current_q is None:
            self.get_logger().warn('No joint state available yet.')
            return

        jacobian = self.current_J
        q = self.current_q
        p_ee = self.current_p_ee
        desired_velocity = np.array([
            msg.linear.x,
            msg.linear.y
        ])

        jacobian_pinv = np.linalg.pinv(jacobian)
        q_dot_mp = jacobian_pinv @ desired_velocity
        x_dot_mp = jacobian @ q_dot_mp

        # 关节限位 二级任务
        # q_dot_limit, secondary_gain, distance_to_limit = (
        #     self.calculate_joint_limit_velocity(
        #         q,
        #         jacobian,
        #         jacobian_pinv
        #     )
        # )
        # ==========================================
        # 约束逆运动学 Constrained IK 对比实验
        #
        # 实验目的：
        # 比较以下三种关节速度求解方式：
        #
        # 1. MP伪逆：
        #    q_dot_mp = J^+ * x_dot_des
        #
        # 2. MP结果直接限幅：
        #    q_dot_clip = clip(q_dot_mp)
        #
        # 3. 带边界约束的线性最小二乘：
        #
        #    min ||J*q_dot - x_dot_des||^2
        #
        #    subject to:
        #    -q_dot_max <= q_dot <= q_dot_max
        #
        # ==========================================


        # ------------------------------------------
        # 关节速度上下限
        #
        # 每个关节最大角速度：
        # |q_dot_i| <= 0.05 rad/s
        # ------------------------------------------
        q_dot_max = np.array([
            0.05,
            0.05,
            0.05
        ])
        # MP无约束解
        # 保存MP解作为实验基准
        q_dot_mp_test = q_dot_mp.copy()
        # MP解实际产生的末端速度
        x_dot_mp_test = (
            jacobian @ q_dot_mp_test
        )
        # MP任务空间速度误差
        error_mp = np.linalg.norm(
            x_dot_mp_test - desired_velocity
        )
        #MP解直接限幅
        # clip只是对已经得到的MP解逐元素截断，并不会重新优化任务空间误差。
        q_dot_clip = np.clip(
            q_dot_mp_test,
            -q_dot_max,
            q_dot_max
        )
        # 限幅后的实际末端速度
        x_dot_clip = (
            jacobian @ q_dot_clip
        )
        error_clip = np.linalg.norm(
            x_dot_clip - desired_velocity
        )
        # ------------------------------------------
        # 边界最小二乘法
        # 首次约束逆运动学实验
        # ------------------------------------------
        # 求解：
        #
        # min ||J*q_dot - x_dot_des||^2
        #
        # subject to（受限于）:
        #
        # -q_dot_max <= q_dot <= q_dot_max
        #
        # 与直接clip不同：
        # lsq_linear会在速度限制范围内重新寻找
        # 任务空间误差尽可能小的关节速度。.


        ##取交集后的上下界
        # ==========================================
        #  位置硬约束 + 原始速度约束
        # ==========================================
        lower_hard, upper_hard,q_max, upper_position = (
            self.calculate_joint_position_velocity_bounds(
                q,
                0.01
            )
        )
        # ==========================================
        #  Velocity Damper 速度阻尼器约束
        # ==========================================
        lower_damper, upper_damper = (
            self.calculate_joint_velocity_damper_bounds(
                q
            )
        )
        # ==========================================
        # 两套约束取交集
        # 下界取较大值
        # 上界取较小值
        # ==========================================
        lower_bound = np.maximum(
            lower_hard,
            lower_damper
        )
        upper_bound = np.minimum(
            upper_hard,
            upper_damper
        )

        # ==========================================
        # 测试：位置限制是否真正生效
        # ==========================================

        # 不修改真实关节位置 q
        q_test = q.copy()

        # 人为让第三关节非常接近上限
        # q3 = pi - 0.0001 rad
        q_test[2] = math.pi - 0.0001

        lower_test, upper_test, q_max_test, upper_position_test = (
            self.calculate_joint_position_velocity_bounds(
                q_test,
                0.01
            )
        )
        # ==========================================
        # 当前 constrained IK
        # ==========================================
        result = lsq_linear(
            jacobian,
            desired_velocity,
            bounds=(
                lower_bound,
                upper_bound
            )
        )   
        # 约束最小二乘得到的关节速度
        q_dot_constrained = result.x
        # 实际产生的末端速度
        x_dot_constrained = (
            jacobian @ q_dot_constrained
        )
        #约束误差
        error_constrained = np.linalg.norm(
            x_dot_constrained
            - desired_velocity
        )
        # 不等式一般形式:
        # A @ q_dot <= b
        n = jacobian.shape[1]
        I_n = np.eye(n)
        #拼接上限  下限 到一个矩阵A
        A = np.vstack([
            I_n,
            -I_n
        ])
        #将单个速度限制更新为位置限制和速度限制的交集
        # b = np.concatenate([
        #     q_dot_max,
        #     q_dot_max
        # ])

        b = np.concatenate([
            upper_bound,
            -lower_bound
        ])

        Aq = A @ q_dot_constrained
        # ==========================================
        # 约束松弛量/约束余量  slack
        #
        # slack = b - A*q_dot
        #
        # slack > 0  -> 约束未到边界
        # slack = 0  -> 约束处于激活状态
        # ==========================================
        constraint_slack = (
            b - Aq
        )
        # 判断约束是否接近激活
        active_tolerance = 1e-8

        active_constraints = (
            np.abs(constraint_slack)
            < active_tolerance
        )

        # ==========================================
        # 构造标准 QP 的 H 和 f
        # ==========================================

        H = jacobian.T @ jacobian

        f = (
            -jacobian.T
            @ desired_velocity
        )
        # ==========================================
        # 验证最小二乘目标与 QP 目标等价
        # ==========================================

        # 原始最小二乘目标：
        #
        # objective_ls= 1/2 ||J*q_dot - x_dot_des||^2
        objective_ls = (
            0.5
            * np.linalg.norm(
                jacobian @ q_dot_constrained
                - desired_velocity
            ) ** 2
        )

        # 标准 QP 中与 q_dot 有关的部分：
        # objective_qp = 1/2 q_dot^T H q_dot + f^T q_dot
        objective_qp = (
            0.5
            * q_dot_constrained.T
            @ H
            @ q_dot_constrained
            + f.T @ q_dot_constrained
        )

        # 展开最小二乘时被省略的常数项：
        # constant = 1/2 x_dot_des^T x_dot_des
        constant_term = (
            0.5
            * desired_velocity.T
            @ desired_velocity
        )

        # 把常数项加回 QP 目标
        objective_qp_with_constant = (
            objective_qp
            + constant_term
        )
    
        #避障约束

        d_safe = 0.05 #安全距离
        eta = 1.0     #增益
        d_influence = 0.20 #影响距离
        #人造障碍点
        obstacle_position = np.array([
            0.95,
            0.40
        ])
        #计算相对量
        delta_p = (
            p_ee
            - obstacle_position
        )
        distance = np.linalg.norm(
            delta_p
        )
        #防止除零
        if distance > 1e-8:
            n_obs = (
                delta_p / distance
            )
            J_distance = (
                n_obs.reshape(1, 2)
                @ jacobian
            )
        A_obstacle = (-J_distance)
        b_obstacle = np.array([eta * (distance - d_safe)])
        distance_rate_before = (J_distance @ q_dot_constrained)
        obstacle_lhs_before = (A_obstacle @ q_dot_constrained)

        #一般线性约束 一个约束对应多个关节
        #与6个约束拼接起来
        # A_extra = np.array([[1.0, 1.0, 0.0]])
        # b_extra = np.array([0.02])
        # A_total = np.vstack([A,A_extra])
        # b_total = np.concatenate([b,b_extra])
        # A_total = np.vstack([A,A_obstacle])
        # b_total = np.concatenate([b,b_obstacle])
        #增加距离影响
        A_list = [A]
        b_list = [b]
        if distance <= d_influence:
                A_list.append(A_obstacle)   #列表末尾追加一个元素
                b_list.append(b_obstacle)
        A_total = np.vstack(A_list)         #矩阵按行向下拼接，竖直拼接
        b_total = np.concatenate(b_list)    #一维数组首尾拼接，数组首尾拼接


        # ==========================================
        # 标准 QP with OSQP
        # ==========================================
        #OSQP 要求 H A是稀疏矩阵
        P = sparse.csc_matrix(H)

        # A_qp = sparse.csc_matrix(A)
        #更改为7*3
        A_qp = sparse.csc_matrix(A_total)
        # # 原约束：
        # # A*q_dot <= b
        # # 转成 OSQP：
        # # -inf <= A*q_dot <= b 
        # qp_lower = np.full(b.shape,-np.inf)
        # qp_upper = b.copy()         #上界
        qp_lower = np.full(b_total.shape,-np.inf)
        qp_upper = b_total.copy() 
      
        
        # 创建 OSQP 求解器
        solver = osqp.OSQP()
        solver.setup(
            P=P,
            q=f,
            A=A_qp,
            l=qp_lower,
            u=qp_upper,
            verbose=False,
            eps_abs=1e-8,
            eps_rel=1e-8,
            max_iter=10000,
            polish=True
        )
        # 求解 QP
        qp_result = solver.solve()
        q_dot_qp = qp_result.x          #取结果
        # 验证 OSQP 解
        x_dot_qp = (jacobian @ q_dot_qp)
        error_qp = np.linalg.norm(x_dot_qp - desired_velocity)
        # 验证 OSQP 解是否违反约束
        Aq_qp = (
            A_total@ q_dot_qp
        )
        constraint_violation = np.maximum(
            Aq_qp - b_total,
            0.0
        )
        max_constraint_violation = np.max(
            constraint_violation
        )

        distance_rate_qp = (J_distance @ q_dot_qp)        
        obstacle_slack = (b_obstacle- A_obstacle @ q_dot_qp)  



        z_limit,secondary_gain,distance_to_limit, activation = (
            self.calculate_joint_limit(q)
        )
        

        # 奇异规避二级任务
        # q_dot_singularity = (
        #     self.calculate_singularity_velocity(
        #         q,
        #         jacobian,
        #         jacobian_pinv
        #     )
        # )

        # z_sing = k_s ∇w(q)
        z_sing = self.calculate_singularity(q)

        #q_dot_null=(q_dot_limit+q_dot_singularity)
        ###############################################
        #测试注释行
        ###############################################
        #q_dot_null =q_dot_singularity

        ############   二级任务融合    ###############
        #z_total=z_limit+z_sing 
        z_total = (z_limit+z_sing)
        # 零空间投影矩阵
        # N = I - J+J    
        
        #N = np.eye(3) - jacobian_pinv @ jacobian
        #代码模块化 
        N = (
            np.eye(jacobian.shape[1])   # jacobian.shape[1] 为关节数 n，用于自动构造 n×n 单位矩阵
            - jacobian_pinv @ jacobian
        )
        # N_zlimit = N @ z_limit
        # N_zsing = N @ z_sing
        # 零空间速度
        #为了区分 数学计算得到的零空间速度  和工程限速后的零空间速度
        q_dot_null_raw = N @ z_total  #原始数据
        q_dot_null = q_dot_null_raw.copy()  #复制一份零空间角速度副本供后面限速
        # 限制零空间最大关节速度
        max_speed = 1.0
        max_value = np.max(
            np.abs(q_dot_null)
        )

        if max_value > max_speed:
            q_dot_null *= (
                max_speed /
                max_value
            )

        # 验证两个二级任务合成后的运动是否让 w 增加。
        # q_next = q + q_dot_null * dt
        # dt = 0.01
        # q_test = q + q_dot_null * dt
        # w_current = self.calculate_manipulability(q)
        # w_next = self.calculate_manipulability(q_test)
        # self.get_logger().info(
        #     f"\nw current = {w_current:.6f}"
        #     f"\nw next = {w_next:.6f}"
        # )    



        singular_values = np.linalg.svd(jacobian, compute_uv=False)
        sigma_min = float(np.min(singular_values))
        if sigma_min >= self.sigma_threshold:
            damping_lambda = 0.0
        else:
            ratio = 1.0 - sigma_min / self.sigma_threshold
            damping_lambda = self.lambda_max * ratio ** 2

        if damping_lambda == 0.0:
            # 非奇异区域：
            # lambda = 0
            # DLS退化为Moore-Penrose伪逆
            jacobian_dls_pinv = jacobian_pinv.copy()
        else:
            # lambda^2
            lambda_sq = damping_lambda ** 2
            # J J^T + lambda^2 I
            matrix = (
                jacobian @ jacobian.T
                + lambda_sq * np.eye(jacobian.shape[0])
            )
            # DLS伪逆
            # J_dls^# = J^T (J J^T + lambda^2 I)^-1
            jacobian_dls_pinv = (
                jacobian.T
                @ np.linalg.solve(matrix, np.eye(jacobian.shape[0]))
            )
        # DLS关节速度    
        # q_dot_dls = J_dls^+ x_dot_des
        q_dot_dls= (
            jacobian_dls_pinv
            @ desired_velocity
        )
        # DLS实际末端速度
        x_dot_dls = jacobian @ q_dot_dls

        #把mp负责的主空间升级为DLS负责主空间
        #q_dot_total 更改定义为DLS的主空间 + 零空间
        #MP以后主要用于：
        #1.构造严格零空间投影 N；
        #2.作为实验基准，与DLS比较
        q_dot_total = q_dot_dls + q_dot_null
        x_dot_total = jacobian @ q_dot_total
        null_space_residual = jacobian @ q_dot_null

        # -----------------------------------------
        # 比较 MP 和 DLS 零空间投影性质
        # -----------------------------------------
        # DLS零空间投影矩阵
        # N_dls = I - J_dls^# J
        N_dls = (
            np.eye(jacobian.shape[1])
            - jacobian_dls_pinv @ jacobian
        )
        # 验证 JN 是否接近0
        JN_mp = jacobian @ N
        JN_dls = jacobian @ N_dls
        # 计算矩阵大小
        JN_mp_norm = np.linalg.norm(JN_mp)
        JN_dls_norm = np.linalg.norm(JN_dls)

        #验证投影矩阵的幂等性  N^2 = N
        N_mp_error = np.linalg.norm(
            N @ N - N
        )

        N_dls_error = np.linalg.norm(
            N_dls @ N_dls - N_dls
        )

        # ==========================================
        # DLS + Null Space 对比实验
        # ==========================================

        # 方案 A：
        # DLS 主任务 + MP 严格零空间
        q_dot_null_A = N @ z_total
        q_dot_total_A = (
            q_dot_dls
            + q_dot_null_A
        )
        x_dot_null_A = (
            jacobian @ q_dot_null_A
        )
        x_dot_total_A = (
            jacobian @ q_dot_total_A
        )
        secondary_leak_A = np.linalg.norm(      #二级任务干扰程度   期望：||J*N_mp*z||≈0        利用欧式范数 把数组里的数压缩成一个数
            x_dot_null_A
        )

        # 方案 B：
        # DLS 主任务 + DLS 软零空间
        q_dot_null_B = N_dls @ z_total

        q_dot_total_B = (
            q_dot_dls
            + q_dot_null_B
        )
        x_dot_null_B = (
            jacobian @ q_dot_null_B
        )
        x_dot_total_B = (
            jacobian @ q_dot_total_B
        )
        secondary_leak_B = np.linalg.norm(      #二级任务干扰程度       期望：||J*N_dls*z||>0       
            x_dot_null_B
        )

        # ==========================================
        # 最终末端误差
        # error = ||x_dot_actual - x_dot_desired||
        # ==========================================
        # 只有DLS主任务时的任务误差         #e_DLS=||x_dot_DLS-x_dot_d||
        task_error_dls = np.linalg.norm(
            x_dot_dls - desired_velocity
        )

        # 方案A：
        # DLS主任务 + MP严格零空间          #e_A=||x_dot_total_A-x_dot_d||
        #预期 ：e_A=e_DLS
        task_error_A = np.linalg.norm(
            x_dot_total_A - desired_velocity
        )

        # 方案B：
        # DLS主任务 + DLS软零空间           #e_B=||x_dot_total_B-x_dot_d||
        #预期 ：e_B！=e_DLS
        task_error_B = np.linalg.norm(
            x_dot_total_B - desired_velocity
        )

        # # ==========================================
        # # 严格任务优先级实验
        # # ==========================================

        # #一级任务 
        # q_dot_task1 = q_dot_mp
        # N1 = (
        #     np.eye(jacobian.shape[1])
        #     - jacobian_pinv @ jacobian
        # )
        # #二级任务：控制第三关节速度
        # J2 = np.array([
        #     [0.0, 1.0, 0.0],
        #     [0.0, 0.0, 1.0]
        # ])
        # x_dot_2_desired = np.array([
        #     0.2,    # q2_dot
        #     -0.2     # q3_dot
        # ])
        # #计算残差 r    r2 = x_dot_2-J_2*q_dot_1
        # secondary_residual = (
        #     x_dot_2_desired
        #     - J2 @ q_dot_task1
        # )
        # #错误实验   直接使用 J2 的伪逆求解二级任务，再通过 N1 投影
        # #预测 结论  一级任务不被破坏 二级任务未达成   
        # y_wrong = (
        #     np.linalg.pinv(J2)
        #     @ secondary_residual
        # )
        # q_dot_task2_wrong = (
        #     N1 @ y_wrong
        # )
        # q_dot_total_wrong = (
        #     q_dot_task1
        #     + q_dot_task2_wrong
        # )
        # task1_wrong = (
        #     jacobian @ q_dot_total_wrong
        # )
        # task2_wrong = (
        #     J2 @ q_dot_total_wrong
        # )

        # #正确方法
        # #一级任务后真正可以使用雅可比
        # J2_projected = J2 @ N1
        # #解J_2*N_1*y=r_2
        # y_task2 = (
        #     np.linalg.pinv(J2_projected)
        #     @ secondary_residual
        # )
        # #二级修正
        # q_dot_task2_correct = (
        #     N1 @ y_task2
        # )
        # #正确的关节总速度
        # q_dot_total_task2 = (
        #     q_dot_task1
        #     + q_dot_task2_correct
        # )
        # task1_correct = (
        #     jacobian @ q_dot_total_task2
        # )
        # task2_achieved = (
        #     J2 @ q_dot_total_task2
        # )
        # task2_error = (
        #     x_dot_2_desired
        #     - task2_achieved
        # )
        # task2_error_norm = np.linalg.norm(
        #     task2_error
        # )

        # # ==========================================
        # # Task 1 + Task 2 完成后的剩余零空间
        # # ==========================================
        # #J2^——伪逆
        # J2_projected_pinv = np.linalg.pinv(
        #     J2_projected
        # )
        # #N2 =N1-J2^——伪逆*J2^——  剩余部分N2=一级剩余空间-二级占用空间  同时不影响 Task 1 和 Task 2 的所有关节运动。
        # N2 = (
        #     N1
        #     - J2_projected_pinv @ J2_projected
        # )
        # N2_norm = np.linalg.norm(N2)  #对N2 求范数  证明已经无自由度

        # task1_N2_residual = np.linalg.norm(    #J1N2  不会破坏一级任务
        #     jacobian @ N2
        # )
        # task2_N2_residual = np.linalg.norm(    #J2N2  不会破坏二级任务
        #     J2 @ N2
        # )
        # #增加三级任务 验证任务存在但机器人没有自由度执行任务  
        # q_dot_task3 = N2 @ z_sing

        # ==========================================
        # 严格任务优先级递归实验
        # ==========================================
        # 初始化
        # q_dot_0 = 0     N0 = I
        q_dot_0 = np.zeros(
            jacobian.shape[1]
        )
        N0 = np.eye(
            jacobian.shape[1]
        )
        # 一级任务
        # J1 * q_dot = x_dot_1
        J1 = jacobian
        x_dot_1 = desired_velocity
        q_dot_1, N1, r1, J1_bar = (
            self.apply_priority_task(
                q_dot_0,
                N0,
                J1,
                x_dot_1
            )
        )
        # 二级任务
        # 控制第三关节角速度  q_dot_3 = -0.2 rad/s
        J2 = np.array([
            [0.0, 0.0, 1.0]
        ])
        x_dot_2 = np.array([-0.2])
        q_dot_2, N2, r2, J2_bar = (
            self.apply_priority_task(
                q_dot_1,
                N1,
                J2,
                x_dot_2
            )
        )

        self.get_logger().info(
            '\n'
             f'Desired Cartesian velocity: {desired_velocity}\n'
            # f'MP joint velocity: {q_dot_mp}\n'
            # f'MP achieved velocity: {x_dot_mp}\n'
            # f'Null-space joint velocity: {q_dot_null}\n'
            # f'Null-space 残差: {null_space_residual}\n'
            # f'Total joint velocity: {q_dot_total}\n'
            # f'Total achieved velocity: {x_dot_total}\n'
            #f'Distance to nearest limit: {distance_to_limit:.6f}\n'    测试
            #f'Secondary gain: {secondary_gain:.6f}\n'                  测试
            #f'Singularity avoidance velocity: {q_dot_null}\n'          测试
            f'z_sing: {z_sing}\n'
            f'z_limit: {z_limit}\n'
            f'z_total: {z_total}\n'
            # region
            #f'Nz_limit:{N_zlimit}\n'
            #f'Nz_sing:{N_zsing}\n'
            #f'\nJ*N_mp:\n{JN_mp}\n'
            #f'||J*N_mp|| = {JN_mp_norm:.8f}\n'
            #f'J*N_dls:\n{JN_dls}\n'
            #f'||J*N_DLS|| = {JN_dls_norm:.8f}\n'
            #f'||N_MP^2 - N_MP|| = {N_mp_error:.10e}\n'
            #f'||N_DLS^2 - N_DLS|| = {N_dls_error:.10e}'
            # f'Adaptive DLS lambda: {damping_lambda:.6f}\n'
            # f'DLS joint velocity: {q_dot_dls}\n'
            # f'DLS achieved velocity: {x_dot_dls}'
            # f'方案 A - DLS + MP null space:\n'
            # f'Null velocity A: {q_dot_null_A}\n'
            # f'Null Cartesian leakage A: {x_dot_null_A}\n'
            # f'Total achieved velocity A: {x_dot_total_A}\n'
            # f'Secondary leakage norm A: {secondary_leak_A:.10e}\n'
            # f'Task error A: {task_error_A:.10e}\n'
            # f'方案 B - DLS + DLS null operator:\n'
            # f'Null velocity B: {q_dot_null_B}\n'
            # f'Null Cartesian leakage B: {x_dot_null_B}\n'
            # f'Total achieved velocity B: {x_dot_total_B}\n'
            # f'Secondary leakage norm B: {secondary_leak_B:.10e}\n'
            # f'Task error B: {task_error_B:.10e}\n'
            # f'Pure DLS task error: {task_error_dls:.10e}'
            # f'===== 严格任务分级实验 =====\n'
            # f'二级任务 期望关节三角速度: {x_dot_2_desired}\n'
            # f'一级任务 基础关节速度: {q_dot_task1}\n'
            # f'二级任务残差: {secondary_residual}\n'    
            # f'投影后的二级雅克比: {J2_projected}\n'
            # '--- 错误实验: 先计算J2伪逆 ---\n'
            # f'总错误关节速度: {q_dot_total_wrong}\n'
            # f'一级任务 错误实现的: {task1_wrong}\n'
            # f'二级任务 错误实现的: {task2_wrong}\n'
            # '--- 正确实验：计算投影后的二级雅克比伪逆 ---\n'
            # f'总正确关节速度: {q_dot_total_correct}\n'
            # f'一级任务 实现的: {task1_correct}\n'
            # f'二级任务 实现的: {task2_correct}'       
            # '===== 二级任务后剩余零空间 =====\n'
            # f'||N2|| = {N2_norm:.10e}\n'
            # f'||J1*N2|| = {task1_N2_residual:.10e}\n'
            # f'||J2*N2|| = {task2_N2_residual:.10e}\n' 
            # f'Nz_sing = {np.linalg.norm(q_dot_task3):.10e}\n'
            # '===== 二级不可行任务实验 =====\n'
            # f'期望二级任务关节速度: {x_dot_2_desired}\n'
            # f'第二个有效雅可比:\n{J2_projected}\n'
            # f'第二个有效雅可比的秩: '
            # f'{np.linalg.matrix_rank(J2_projected)}\n'
            # f'总关节速度: {q_dot_total_task2}\n'
            # f'一级任务达成的: '
            # f'{jacobian @ q_dot_total_task2}\n'
            # f'二级任务达成的: {task2_achieved}\n'
            # f'二级任务残差: {task2_error}\n'
            # f'||二级任务残差||: '
            # f'{task2_error_norm:.10e}'
            # '===== 严格任务优先级递归实验 =====\n'
            # f'q_dot_0: {q_dot_0}\n'
            # f'N0:\n{N0}\n'
            # '\n--- 一级任务 ---\n'
            # f'r1: {r1}\n'
            # f'J1_bar:\n{J1_bar}\n'
            # f'q_dot_1: {q_dot_1}\n'
            # f'||q_dot_1 - q_dot_mp||: '
            # f'{np.linalg.norm(q_dot_1 - q_dot_mp):.10e}\n'
            # '\n--- 二级任务 ---\n'
            # f'J2_bar:\n{J2_bar}\n'
            # f'r2: {r2}\n'
            # f'q_dot_2: {q_dot_2}\n'
            # f'Task 1 achieved: {J1 @ q_dot_2}\n'
            # f'Task 2 achieved: {J2 @ q_dot_2}\n'
            # '\n--- 剩余零空间 ---\n'
            # f'N2:\n{N2}\n'
            # f'||J1*N2||: '
            # f'{np.linalg.norm(J1 @ N2):.10e}\n'
            # f'||J2*N2||: '
            # f'{np.linalg.norm(J2 @ N2):.10e}'
            # f'Distance: {distance_to_limit}\n'
            # f'Joint limit activation: {activation}\n'
            # '===== 约束逆运动学实验 =====\n'
            # f'MP q_dot: {q_dot_mp_test}\n'
            # f'MP achieved: {x_dot_mp_test}\n'
            # f'MP error: {error_mp:.10e}\n'
            # f'Clip q_dot: {q_dot_clip}\n'
            # f'Clip achieved: {x_dot_clip}\n'
            # f'Clip error: {error_clip:.10e}\n'
            # endregion
             f'Constrained q_dot: {q_dot_constrained}\n'
             f'Constrained achieved: {x_dot_constrained}\n'
             f'Constrained error: {error_constrained:.10e}\n'
            # '===== 二次规划约束不等式 =====\n'
            f'A:\n{A}\n'
            f'b: {b}\n'
            f'A @ q_dot_constrained: '
            f'{Aq}\n'
            # f'Aq <= b: {Aq <= b}'
            # region '===== 约束激活检测 =====\n'
            f'Constraint slack: {constraint_slack}\n'   
            f'Active constraints: {active_constraints}\n'
            # '===== 合并位置限制和速度限制 =====\n'
            # f'下边界: {lower_bound}\n'
            # f'上边界: {upper_bound}\n'
            # f'距离上限 q_max-q: {q_max - q}\n'
            # f'位置产生的上界: {upper_position}\n'  
            # f'q_test: {q_test}\n'
            # f'q_max - q_test: {math.pi - q_test[2]}\n'
            # f'测试上界: {upper_test}\n'    
            # f'测试下界: {lower_test}\n'    
            # endregion   
            # f'速度阻尼器上界{upper_damper}\n'
            # f'速度阻尼器下界{lower_damper}\n'
            # '===== 位置速度限制+电机速度限制+关节动态上下界 =====\n'
            # f'下边界: {lower_bound}\n'
            # f'上边界: {upper_bound}\n'  
            # f'H:\n{H}\n'
            # f'f: {f}\n'
            # f'H shape: {H.shape}\n'
            # f'f shape: {f.shape}\n'
            # '===== LS 与 QP 目标验证 =====\n'
            # f'最小二乘目标：{objective_ls:.12e}\n'
            # f'QP 目标：{objective_qp:.12e}\n'
            # f'常数项：{constant_term:.12e}\n'
            # f'QP 目标 + 常数项：{objective_qp_with_constant:.12e}\n'
            # f'QP 目标 - 最小二乘目标：'
            # f'{objective_qp_with_constant - objective_ls:.12e}\n'
            # '===== OSQP QP 求解 =====\n'
            # f'QP状态信息: {qp_result.info.status}\n'
            # f'OSQP QP关节速度q_dot: {q_dot_qp}\n'
            # f'q1_dot + q2_dot:{q_dot_qp[0] + q_dot_qp[1]:.10f}\n'
            # f'OSQP QP末端速度: {x_dot_qp}\n'
            # f'OSQP QP误差: {error_qp:.10e}\n'
            # f'OSQP status: {qp_result.info.status}\n'
            # f'||QP - LSQ||: {np.linalg.norm(q_dot_qp - q_dot_constrained):.10e}'
            # f'约束左端A @ q_dot_qp: {Aq_qp}\n'
            # f'约束违反量Constraint violation: {constraint_violation}\n'
            # f'最大约束违反量Max constraint violation:{max_constraint_violation:.10e}\n'
            '===== 障碍物距离约束 =====\n'
            f'End-effector position: {p_ee}\n'
            f'Obstacle position: {obstacle_position}\n'
            f'Distance: {distance:.6f}\n'
            f'n_obs: {n_obs}\n'
            f'J_distance: {J_distance}\n'
            f'A_obstacle: {A_obstacle}\n'
            f'b_obstacle: {b_obstacle}\n'
            f'QP避障距离变化率: {distance_rate_before}\n'
            f'最小允许距离变化率：{-b_obstacle}\n'
            f'避障约束左端实际值: {obstacle_lhs_before}\n'
            f'b_obs: {b_obstacle}\n'
            f'QP避障后关节速度 q_dot: {q_dot_qp}\n'
            f'QP避障后距离变化率 d_dot: {distance_rate_qp}\n'
            f'避障松弛量 obstacle slack: {obstacle_slack}'
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
