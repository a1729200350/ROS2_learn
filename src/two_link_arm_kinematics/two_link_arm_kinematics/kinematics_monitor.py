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

        # ==========================================
        # QP 控制参数
        # ==========================================
        # 控制周期
        # 当前节点仍由 desired_velocity_callback() 触发，
        # 尚未建立真正的 100 Hz timer 控制循环。
        self.control_dt = 0.01
        # 障碍物
        self.obstacle_position = np.array([
            0.95,
            0.40
        ])
        # 避障参数
        self.obstacle_d_safe = 0.05
        self.obstacle_d_influence = 0.20
        self.obstacle_eta = 1.0
        # 障碍物 Slack
        self.rho_obstacle = 0.01
        self.slack_obstacle_max = 0.005
        # 约束激活判断容差
        self.active_tolerance = 1e-8

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

    def calculate_dynamic_bounds(self, q):
        """计算最终关节速度动态上下界，并转换成 A*q_dot <= b。"""
        # 位置硬约束 + 原始速度约束
        lower_hard, upper_hard, _, _ = (self.calculate_joint_position_velocity_bounds(q,self.control_dt))
        # 速度阻尼器 Velocity Damper
        lower_damper, upper_damper = (self.calculate_joint_velocity_damper_bounds(q))
        # 两组约束取交集
        lower_bound = np.maximum(
            lower_hard,
            lower_damper
        )
        upper_bound = np.minimum(
            upper_hard,
            upper_damper
        )
        # 转换成 A*q_dot <= b
        n = len(q)
        I_n = np.eye(n)
        A = np.vstack([
            I_n,
            -I_n
        ])
        b = np.concatenate([
            upper_bound,
            -lower_bound
        ])
        return {
            'lower_bound': lower_bound,
            'upper_bound': upper_bound,
            'A': A,
            'b': b
        }

    def calculate_obstacle_constraint(self,p_ee,jacobian):
        """计算障碍物距离约束。"""
        # 相对位置
        delta_p = (p_ee- self.obstacle_position)
        distance = float(np.linalg.norm(delta_p))
        # 2. 距离 Jacobian     d_dot = J_distance * q_dot
        if distance > 1e-8:
            n_obs = (delta_p / distance)
            J_distance = (n_obs.reshape(1, 2)@ jacobian)
        else:
            n_obs = np.zeros(2)
            J_distance = np.zeros((1, jacobian.shape[1]))
        # 原始硬避障约束 d_dot >= -eta(d-d_safe)
        # 转换： -J_distance*q_dot <= eta(d-d_safe)
        A_obstacle = (-J_distance)
        b_obstacle = np.array([self.obstacle_eta * (distance- self.obstacle_d_safe)])
        # 是否进入影响区域
        active = (distance<= self.obstacle_d_influence)
        return {
            'active': active,
            'distance': distance,
            'n_obs': n_obs,
            'J_distance': J_distance,
            'A': A_obstacle,
            'b': b_obstacle
        }

    def build_qp(self,jacobian,desired_velocity,dynamic_bounds,obstacle):
        """构造带障碍物 Slack 的完整 QP。"""
        n = jacobian.shape[1]
        # 原始任务目标
        # min 1/2 ||J*q_dot - x_dot_d||^2
        # H = J^T J
        # f = -J^T x_dot_d
        H = (jacobian.T@ jacobian)
        f = (-jacobian.T@ desired_velocity)
        # 扩展优化变量
        # x = [q_dot1, q_dot2, q_dot3, s_obs]^T
        variable_count = n + 1
        H_aug = np.zeros((variable_count, variable_count))
        H_aug[:n, :n] = H
        H_aug[n, n] = (self.rho_obstacle)
        f_aug = np.zeros(variable_count)
        f_aug[:n] = f
        # 3. 原来的关节硬约束
        # A*q_dot <= b
        # 扩展：
        # [A  0] [q_dot] <= b
        #        [s_obs]
        # Slack 不允许破坏这些硬约束
        A_base = dynamic_bounds['A']
        b_base = dynamic_bounds['b']
        A_hard_base = np.hstack([A_base,np.zeros((A_base.shape[0], 1))])
        A_list = [A_hard_base]
        b_list = [b_base]
        # 软避障约束
        # A_obs*q_dot - s_obs <= b_obs
        if obstacle['active']:
            A_obstacle_soft = np.hstack([obstacle['A'],np.array([[-1.0]])])
            A_list.append(A_obstacle_soft)
            b_list.append(obstacle['b'])
        # Slack 下界
        # s_obs >= 0
        # -s_obs <= 0
        A_slack_lower = np.zeros((1, variable_count))
        A_slack_lower[0, n] = -1.0
        b_slack_lower = np.array([0.0])
        A_list.append(A_slack_lower)
        b_list.append(b_slack_lower)
        # Slack 上界
        # s_obs <= s_max
        A_slack_upper = np.zeros((1, variable_count))
        A_slack_upper[0, n] = 1.0
        b_slack_upper = np.array([self.slack_obstacle_max])
        A_list.append(A_slack_upper)
        b_list.append(b_slack_upper)
        # 拼接完整约束
        A_aug = np.vstack(A_list)
        b_aug = np.concatenate(b_list)  
        return {
            'H': H,
            'f': f,
            'H_aug': H_aug,
            'f_aug': f_aug,
            'A_aug': A_aug,
            'b_aug': b_aug,
            'num_joints': n
        }

    def solve_qp(self, qp):
        """使用 OSQP 求解 QP。"""
        H_aug = qp['H_aug']
        f_aug = qp['f_aug']
        A_aug = qp['A_aug']
        b_aug = qp['b_aug']
        # 转换成 OSQP 稀疏矩阵
        P = sparse.csc_matrix(H_aug)
        A_qp = sparse.csc_matrix(A_aug)
        # 原约束：
        # A_aug*x <= b_aug
        # OSQP：
        # -inf <= A_aug*x <= b_aug
        qp_lower = np.full(b_aug.shape,-np.inf)
        qp_upper = (b_aug.copy())
        # 求解
        solver = osqp.OSQP()
        solver.setup(
            P=P,
            q=f_aug,
            A=A_qp,
            l=qp_lower,
            u=qp_upper,
            verbose=False,
            eps_abs=1e-8,
            eps_rel=1e-8,
            max_iter=10000,
            polish=True
        )
        result = solver.solve()
        status = result.info.status
        success = status in (
            'solved',
            'solved inaccurate'
        )
        if not success:
            return {
                'success': False,
                'status': status,
                'result': result
            }
        x_solution = (result.x)
        n = qp['num_joints']
        q_dot = (x_solution[:n])
        slack_obstacle = (x_solution[n])
        return {
            'success': True,
            'status': status,
            'result': result,
            'x_solution': x_solution,
            'q_dot': q_dot,
            'slack_obstacle': slack_obstacle,
            'dual_variables': result.y
        }

    def check_qp_solution(self,jacobian,desired_velocity,qp,solution,obstacle):
        """检查 QP 解、约束和 TTK 条件。"""
        x_solution = (solution['x_solution'])
        q_dot = (solution['q_dot'])
        slack_obstacle = (solution['slack_obstacle'])
        dual_variables = (solution['dual_variables'])
        H_aug = qp['H_aug']
        f_aug = qp['f_aug']
        A_aug = qp['A_aug']
        b_aug = qp['b_aug']
        # 任务空间误差
        achieved_velocity = (jacobian @ q_dot)
        task_error = np.linalg.norm(
            achieved_velocity
            - desired_velocity
        )
        # 完整约束余量
        # margin = b - A*x
        # margin > 0 ：约束未激活
        # margin = 0 ：约束激活
        # margin < 0 ：约束被违反
        constraint_margin = (b_aug- A_aug @ x_solution)
        active_constraints = (np.abs(constraint_margin)< self.active_tolerance )
        # KKT - Stationarity 驻点条件
        # H*x + f + A^T*y = 0
        stationarity_residual = (
            H_aug @ x_solution
            + f_aug
            + A_aug.T @ dual_variables
        )
        stationarity_error = np.linalg.norm(
            stationarity_residual
        )
        # KKT - Complementarity 互补松弛
        # y_i * (b_i - A_i*x) = 0
        complementarity = (
            dual_variables
            * constraint_margin
        )
        complementarity_error = np.linalg.norm(
            complementarity
        )
        # KKT - Primal Feasibility
        # 要求：
        # A*x <= b
        # A*x - b > 0 的部分才是真正的约束违反量
        #原始可行性违反量
        primal_violation = np.maximum(
            A_aug @ x_solution - b_aug,
            0.0
        )
        #违反量范数
        primal_feasibility_error = np.linalg.norm(
            primal_violation
        )
        # KKT - Dual Feasibility 对偶可行性条件
        # 因此对应拉格朗日乘数要求：y >= 0
        #如果 y < 0，则负的部分属于对偶可行性违反量
        dual_violation = np.maximum(
            -dual_variables,
            0.0
        )
        # 对偶可行性误差
        dual_feasibility_error = np.linalg.norm(
            dual_violation
        )
        # Slack 上界状态
        slack_upper_margin = (
            self.slack_obstacle_max
            - slack_obstacle
        )
        slack_upper_active = (
            abs(slack_upper_margin)
            < self.active_tolerance
        )
        # 障碍物指标
        distance_rate = float((obstacle['J_distance']@ q_dot)[0])
        minimum_distance_rate = (-obstacle['b'][0])
        obstacle_violation = max(
            minimum_distance_rate
            - distance_rate,
            0.0
        )
        guaranteed_min_distance_rate = (
            minimum_distance_rate
            - self.slack_obstacle_max
        )
        return {
            'achieved_velocity': achieved_velocity,
            'task_error': task_error,
            'constraint_margin':
                constraint_margin,
            'active_constraints':
                active_constraints,
            'stationarity_residual':
                stationarity_residual,
            'stationarity_error':
                stationarity_error,
            'complementarity':
                complementarity,
            'complementarity_error':
                complementarity_error,
            'primal_violation':
                primal_violation,
            'primal_feasibility_error':
                primal_feasibility_error,
            'dual_violation':
                dual_violation,
            'dual_feasibility_error':
                dual_feasibility_error,
            'slack_upper_margin':
                slack_upper_margin,
            'slack_upper_active':
                slack_upper_active,
            'distance_rate':
                distance_rate,
            'minimum_distance_rate':
                minimum_distance_rate,
            'guaranteed_min_distance_rate':
                guaranteed_min_distance_rate,
            'obstacle_violation':
                obstacle_violation
        }

    def print_qp_result(self,solution,check,obstacle):
        self.get_logger().info(
            '\n'
            '===== QP Result =====\n'
            f"QP status: "
            f"{solution['status']}\n"
            f"q_dot: "
            f"{solution['q_dot']}\n"
            f"obstacle distance: "
            f"{obstacle['distance']:.10f}\n"
            f"obstacle active: "
            f"{obstacle['active']}\n"
            f"obstacle slack: "
            f"{solution['slack_obstacle']:.10f}\n"
            f"distance rate: "
            f"{check['distance_rate']:.10f}\n"
            f"task error: "
            f"{check['task_error']:.10e}\n"
            f"stationarity error: "
            f"{check['stationarity_error']:.10e}\n"
            f"complementarity error: "
            f"{check['complementarity_error']:.10e}"
        )

    # ==================================================
    # 已完成学习实验
    # 以下函数默认不参与当前正式 QP 控制流程。
    # 需要复现实验时，在 desired_velocity_callback()
    # 中临时调用。
  
    def experiment_constrained_ik(self,q,jacobian,desired_velocity):
        """MP、直接限幅、约束最小二乘对比实验。"""
        # ==========================================
        # MP 伪逆
        # ========================================== 
        jacobian_pinv = np.linalg.pinv(jacobian)
        q_dot_mp = (jacobian_pinv@ desired_velocity)
        x_dot_mp = (jacobian@ q_dot_mp)
        error_mp = np.linalg.norm(x_dot_mp- desired_velocity)
        # ==========================================
        # MP 后直接限幅
        # ==========================================
        q_dot_max = np.full(jacobian.shape[1],0.05)
        q_dot_clip = np.clip(
            q_dot_mp,
            -q_dot_max,
            q_dot_max
        )
        x_dot_clip = (jacobian@ q_dot_clip)
        error_clip = np.linalg.norm(x_dot_clip- desired_velocity)
        # ==========================================
        # 约束最小二乘 Constrained Least Squares
        # ==========================================
        dynamic_bounds = (self.calculate_dynamic_bounds(q))
        result = lsq_linear(
            jacobian,
            desired_velocity,
            bounds=(
                dynamic_bounds['lower_bound'],
                dynamic_bounds['upper_bound']
            )
        )
        q_dot_constrained = (result.x)
        x_dot_constrained = (jacobian@ q_dot_constrained)
        error_constrained = np.linalg.norm(x_dot_constrained- desired_velocity)
        self.get_logger().info(
            '\n'
            '===== 约束逆运动学实验 =====\n'
            f'MP q_dot: {q_dot_mp}\n'
            f'MP error: {error_mp:.10e}\n'
            f'Clip q_dot: {q_dot_clip}\n'
            f'Clip error: {error_clip:.10e}\n'
            f'Constrained q_dot: '
            f'{q_dot_constrained}\n'
            f'Constrained error: '
            f'{error_constrained:.10e}'
        )

    def experiment_dls_null_space(self,q,jacobian,desired_velocity):
        """DLS、MP零空间与DLS软零空间对比实验。"""
        # ==========================================
        # MP 伪逆
        # ==========================================
        jacobian_pinv = np.linalg.pinv(jacobian)
        # ==========================================
        # 二级任务
        # ==========================================
        z_limit, gain, distance_to_limit, activation = (self.calculate_joint_limit(q))
        z_sing = (self.calculate_singularity(q))
        z_total = (z_limit + z_sing)
        # ==========================================
        # MP 严格零空间
        # ==========================================
        N = (np.eye(jacobian.shape[1])- jacobian_pinv @ jacobian)
        # ==========================================
        # 自适应 DLS
        # ==========================================
        singular_values = np.linalg.svd(jacobian,compute_uv=False)
        sigma_min = float(np.min(singular_values))
        if sigma_min >= self.sigma_threshold:
            damping_lambda = 0.0
        else:
            ratio = (
                1.0
                - sigma_min
                / self.sigma_threshold
            )
            damping_lambda = (
                self.lambda_max
                * ratio ** 2
            )
        if damping_lambda == 0.0:
            jacobian_dls_pinv = (
                jacobian_pinv.copy()
            )
        else:
            matrix = (
                jacobian @ jacobian.T
                + damping_lambda ** 2
                * np.eye(jacobian.shape[0])
            )
            jacobian_dls_pinv = (
                jacobian.T
                @ np.linalg.solve(
                    matrix,
                    np.eye(jacobian.shape[0])
                )
            )
        q_dot_dls = (
            jacobian_dls_pinv
            @ desired_velocity
        )
        # ==========================================
        # DLS 软零空间
        # ==========================================
        N_dls = (
            np.eye(jacobian.shape[1])
            - jacobian_dls_pinv @ jacobian
        )
        # ==========================================
        # 方案 A
        # DLS + MP 严格零空间
        # ==========================================
        q_dot_null_A = (
            N @ z_total
        )
        q_dot_total_A = (
            q_dot_dls
            + q_dot_null_A
        )
        # ==========================================
        # 方案 B
        # DLS + DLS 软零空间
        # ==========================================
        q_dot_null_B = (N_dls @ z_total)
        q_dot_total_B = (
            q_dot_dls
            + q_dot_null_B
        )
        self.get_logger().info(
            '\n'
            '===== DLS + 零空间 实验 =====\n'
            f'lambda: {damping_lambda:.10f}\n'
            f'DLS q_dot: {q_dot_dls}\n'
            f'方案 A q_dot: {q_dot_total_A}\n'
            f'方案 B q_dot: {q_dot_total_B}\n'
            f'||J*N_MP||: '
            f'{np.linalg.norm(jacobian @ N):.10e}\n'
            f'||J*N_DLS||: '
            f'{np.linalg.norm(jacobian @ N_dls):.10e}'
        )

    def experiment_strict_priority(self,jacobian,desired_velocity):
        """严格任务优先级递归实验。"""
        q_dot_0 = np.zeros(jacobian.shape[1])
        N0 = np.eye(jacobian.shape[1])
        # 一级任务
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
        J2 = np.array([[0.0, 0.0, 1.0]])
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
            '===== 严格任务优先级实验 =====\n'
            f'q_dot_1: {q_dot_1}\n'
            f'q_dot_2: {q_dot_2}\n'
            f'Task 1 achieved: '
            f'{J1 @ q_dot_2}\n'
            f'Task 2 achieved: '
            f'{J2 @ q_dot_2}\n'
            f'||J1*N2||: '
            f'{np.linalg.norm(J1 @ N2):.10e}\n'
            f'||J2*N2||: '
            f'{np.linalg.norm(J2 @ N2):.10e}'
        )

    def solve_experiment_qp(self,H,f,A,b):
        """实验专用通用 OSQP 求解函数。"""
        # ==========================================
        # 转换成 OSQP 稀疏矩阵
        # ==========================================
        P = sparse.csc_matrix(H)
        A_qp = sparse.csc_matrix(A)
        # A*x <= b
        # 转成 OSQP：
        # -inf <= A*x <= b
        qp_lower = np.full(b.shape,-np.inf)
        qp_upper = (b.copy())
        # ==========================================
        # 求解
        # ==========================================
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
        return solver.solve()

    def experiment_qp_infeasible(self,q,jacobian,desired_velocity):
        """人为制造互相冲突的硬约束，观察 QP 不可行。"""
        #  原始任务目标
        # min 1/2 ||J*q_dot - x_dot_d||^2
        H = (jacobian.T@ jacobian)
        f = (-jacobian.T@ desired_velocity)
        # 原来的动态关节约束
        dynamic_bounds = (self.calculate_dynamic_bounds(q))
        A = dynamic_bounds['A']
        b = dynamic_bounds['b']
        
        # 人工制造互相冲突的两个硬约束
        # q1_dot <= -0.04
        # -q1_dot <= -0.04   ->   q1_dot >= 0.04
        # 二者无法同时成立
        
        A_conflict = np.array([
            [ 1.0, 0.0, 0.0],
            [-1.0, 0.0, 0.0]
        ])

        b_conflict = np.array([
            -0.04,
            -0.04
        ])
        # 拼接
        A_total = np.vstack([
            A,
            A_conflict
        ])
        b_total = np.concatenate([
            b,
            b_conflict
        ])
        # 求解
        result = self.solve_experiment_qp(
            H,
            f,
            A_total,
            b_total
        )
        self.get_logger().info(
            '\n'
            '===== 人为制造的QP不可行冲突实验 =====\n'
            f'QP status: {result.info.status}'
        )

    def experiment_single_slack(self,q,jacobian,desired_velocity,rho=1000.0):
        """单 Slack 松弛变量实验。"""
        n = jacobian.shape[1]
        # 原始任务目标
        H = (
            jacobian.T
            @ jacobian
        )
        f = (
            -jacobian.T
            @ desired_velocity
        )
        # 扩展变量
        # x = [q1_dot, q2_dot, q3_dot, s]^T
        H_aug = np.zeros(
            (n + 1, n + 1)
        )
        H_aug[:n, :n] = H
        # Slack 惩罚：rho/2 * s^2
        H_aug[n, n] = rho
        f_aug = np.zeros(n + 1)
        f_aug[:n] = f
        # 原来的关节硬约束
        dynamic_bounds = (self.calculate_dynamic_bounds(q))
        A_base = (dynamic_bounds['A'])
        b_base = (dynamic_bounds['b'])
        # 原来 6×3 扩展成 6×4
        # Slack 不允许破坏关节硬约束
        A_hard_base = np.hstack([
            A_base,
            np.zeros(
                (A_base.shape[0], 1)
            )
        ])
        # 人工硬约束
        # q1_dot <= -0.04
        A_conflict_hard = np.array([
            [1.0, 0.0, 0.0, 0.0]
        ])
        b_conflict_hard = np.array([
            -0.04
        ])
        #人工软约束
        # 原来希望：q1_dot >= 0.04
        # 即：
        # -q1_dot <= -0.04
        # 加 Slack：-q1_dot - s <= -0.04
        # 即：
        # q1_dot + s >= 0.04
        A_conflict_soft = np.array([
            [-1.0, 0.0, 0.0, -1.0]
        ])
        b_conflict_soft = np.array([
            -0.04
        ])
        # Slack 非负 
        # s >= 0
        # -s <= 0
        A_slack_nonnegative = np.array([
            [0.0, 0.0, 0.0, -1.0]
        ])

        b_slack_nonnegative = np.array([
            0.0
        ])
        # 拼接完整约束
        A_aug = np.vstack([
            A_hard_base,
            A_conflict_hard,
            A_conflict_soft,
            A_slack_nonnegative
        ])

        b_aug = np.concatenate([
            b_base,
            b_conflict_hard,
            b_conflict_soft,
            b_slack_nonnegative
        ])
        # 求解
        result = self.solve_experiment_qp(
            H_aug,
            f_aug,
            A_aug,
            b_aug
        )
        status = (result.info.status)
        if status not in (
            'solved',
            'solved inaccurate'
        ):
            self.get_logger().info(
                '\n'
                '===== Single Slack Experiment =====\n'
                f'QP status: {status}'
            )
            return
        x_solution = (result.x)
        q_dot = (x_solution[:n])
        slack = (x_solution[n])
        self.get_logger().info(
            '\n'
            '===== 单松弛变量实验 =====\n'
            f'rho: {rho}\n'
            f'QP status: {status}\n'
            f'q_dot: {q_dot}\n'
            f'slack s: {slack:.10f}'
        )

    def experiment_multiple_slack(self,q,jacobian,desired_velocity,rho_1=100.0,rho_2=100.0):
        """两个独立 Slack 变量实验。"""
        n = jacobian.shape[1]
        # 优化变量：
        # x =   [q1_dot,
        #       q2_dot,
        #       q3_dot,
        #       s1,
        #       s2]^T
        variable_count = (n + 2)
        # 原始任务目标
        H = (jacobian.T@ jacobian)
        f = (-jacobian.T@ desired_velocity)
        # 扩展目标函数
        H_aug = np.zeros(
            (
                variable_count,
                variable_count
            )
        )
        H_aug[:n, :n] = H
        H_aug[n, n] = (rho_1)
        H_aug[n + 1, n + 1] = (rho_2)
        f_aug = np.zeros(variable_count)
        f_aug[:n] = f
        # 原始关节硬约束
        dynamic_bounds = (self.calculate_dynamic_bounds(q))
        A_base = (dynamic_bounds['A'])
        b_base = (dynamic_bounds['b'])
        # 6×3 -> 6×5
        A_hard_base = np.hstack([
            A_base,
            np.zeros(
                (A_base.shape[0], 2)
            )
        ])
        # 第一条软约束
        # q1_dot <= -0.04 + s1
        # q1_dot - s1 <= -0.04
        A_soft_1 = np.array([
            [
                1.0,
                0.0,
                0.0,
                -1.0,
                0.0
            ]
        ])
        b_soft_1 = np.array([
            -0.04
        ])
        # 第二条软约束
        # q1_dot >= 0.04 - s2
        # -q1_dot - s2 <= -0.04
        A_soft_2 = np.array([
            [
                -1.0,
                0.0,
                0.0,
                0.0,
                -1.0
            ]
        ])
        b_soft_2 = np.array([
            -0.04
        ])
        # s1 >= 0, s2 >= 0
        A_slack_nonnegative = np.array([
            [
                0.0,
                0.0,
                0.0,
                -1.0,
                0.0
            ],
            [
                0.0,
                0.0,
                0.0,
                0.0,
                -1.0
            ]
        ])
        b_slack_nonnegative = np.array([
            0.0,
            0.0
        ])
        # 拼接
        A_aug = np.vstack([
            A_hard_base,
            A_soft_1,
            A_soft_2,
            A_slack_nonnegative
        ])
        b_aug = np.concatenate([
            b_base,
            b_soft_1,
            b_soft_2,
            b_slack_nonnegative
        ])
        #求解
        result = self.solve_experiment_qp(
            H_aug,
            f_aug,
            A_aug,
            b_aug
        )
        status = (
            result.info.status
        )
        if status not in (
            'solved',
            'solved inaccurate'
        ):
            self.get_logger().info(
                '\n'
                '===== Multiple Slack Experiment =====\n'
                f'QP status: {status}'
            )
            return
        x_solution = (result.x)

        q_dot = (x_solution[:n])

        slack_1 = (x_solution[n])

        slack_2 = (x_solution[n + 1])
        # 真实违反量
        violation_1 = max(
            q_dot[0] + 0.04,
            0.0
        )
        violation_2 = max(
            0.04 - q_dot[0],
            0.0
        )
        self.get_logger().info(
            '\n'
            '=====多松弛变量实验 =====\n'
            f'rho_1: {rho_1}\n'
            f'rho_2: {rho_2}\n'
            f'QP status: {status}\n'
            f'q_dot: {q_dot}\n'
            f'slack_1: {slack_1:.10f}\n'
            f'slack_2: {slack_2:.10f}\n'
            f'violation_1: {violation_1:.10f}\n'
            f'violation_2: {violation_2:.10f}'
        )

    # ==================================================
   
    def joint_state_callback(self, msg):
        """升级到三关节运动学状态"""
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
        """根据期望末端速度构造并求解当前 QP。"""
        # ==========================================
        # 检查当前机器人状态
        # ==========================================
        if (self.current_J is None
            or self.current_q is None
            or self.current_p_ee is None
            ):
            self.get_logger().warn('No joint state available yet.')
            return
        jacobian = self.current_J
        q = self.current_q
        p_ee = self.current_p_ee
        desired_velocity = np.array([
            msg.linear.x,
            msg.linear.y
        ])
        # 动态关节速度约束
        dynamic_bounds = (self.calculate_dynamic_bounds(q))
        # 障碍物距离约束
        obstacle = (self.calculate_obstacle_constraint(p_ee,jacobian))
        # 构造完整 QP
        qp = self.build_qp(
            jacobian,
            desired_velocity,
            dynamic_bounds,
            obstacle
        )
        # 求解 QP
        solution = self.solve_qp(qp)
        if not solution['success']:
            self.get_logger().warn(f"QP failed: {solution['status']}")
            return
        # 检查 QP的解 和 TTK条件
        check = self.check_qp_solution(
            jacobian,
            desired_velocity,
            qp,
            solution,
            obstacle
        )
        #    打印
        self.print_qp_result(
            solution,
            check,
            obstacle
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


# ==================================================
# 历史实验调用说明
#  experiment_*() 函数默认不参与当前正式 QP 控制流程。
# 平时不要调用，只有需要复现实验时，
# 才在 desired_velocity_callback() 中临时加入对应函数调用。
# 例如：
# self.experiment_qp_infeasible(q,jacobian,desired_velocity)
# 实验完成后，将对应调用重新注释掉即可。
# 这样不会影响当前正式 QP 主线。
# ==================================================
# ------------------------------------------
# 1. 人工不可行 QP 实验
#
# 人为加入互相冲突的硬约束，
# 预期 OSQP 返回：
# primal infeasible
# ------------------------------------------
# self.experiment_qp_infeasible(
#     q,
#     jacobian,
#     desired_velocity
# )

# ------------------------------------------
# 2. 单 Slack 松弛变量实验
#
# 优化变量：
# x = [q_dot1, q_dot2, q_dot3, s]
#
# rho 控制 Slack 的惩罚权重。
# 需要实验时取消下面代码的注释。
# ------------------------------------------
# self.experiment_single_slack(
#     q,
#     jacobian,
#     desired_velocity,
#     rho=1000.0
# )

# ------------------------------------------
# 3. 两个 Slack 松弛变量实验
#
# 优化变量：
# x = [q_dot1, q_dot2, q_dot3, s1, s2]
#
# rho_1、rho_2 分别控制
# 两个 Slack 变量的惩罚权重。
# ------------------------------------------
# self.experiment_multiple_slack(
#     q,
#     jacobian,
#     desired_velocity,
#     rho_1=100.0,
#     rho_2=100.0
# )

# ==========================================
# 使用原则
#
# 需要复现实验：
#     取消对应 experiment_*() 调用的注释
#
# 实验结束：
#     再将对应调用注释掉
#
# 正式 QP 主线保持不变：
#
# calculate_dynamic_bounds()
#          ↓
# calculate_obstacle_constraint()
#          ↓
# build_qp()
#          ↓
# solve_qp()
#          ↓
# check_qp_solution()
#          ↓
# print_qp_result()
# ==========================================