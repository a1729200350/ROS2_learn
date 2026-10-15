from collections import deque
from pathlib import Path
import csv
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from three_link_arm_kinematics.operational_space_controller import (OperationalSpaceController,)
from three_link_arm_kinematics.operational_space_dynamics import (OperationalSpaceDynamics,)
DT_SIM = 0.001
DT_CTRL = 0.010
HOLD_SECONDS = 1.0
TORQUE_LIMIT = np.array([10.0, 10.0, 10.0])
K_P0 = np.eye(3) * 1.0
K_D0 = np.eye(3) * 0.5
LAMBDA_EIGENVALUE_CAP = 100.0
MODES = ("A_strict", "B_all_damped", "C_split")

def load_reference(filename):
  """
  从 NPZ 文件读取参考轨迹。

  返回：
    source:轨迹文件路径

    ref:
      t    ：时间序列
      q    ：期望关节位置
      dq   ：期望关节速度
      ddq  ：期望关节加速度
  """

  # 将输入路径转换为绝对路径
  source = ( Path(filename) .expanduser().resolve())
  # 读取 NPZ 文件
  with np.load( source, allow_pickle=False,) as d:
    # 提取时间、位置、速度、加速度
    ref = {
      key: d[key].copy()
      for key in ("t","q", "dq", "ddq",)
    }

    # 读取关节名称
    names = d["joint_names"].tolist()
    # 读取任务空间参考坐标系
    frame = str(d["task_frame"])
  # 时间数组
  t = ref["t"]
  # 轨迹采样点数量
  n = len(t)
  # 检查关节顺序和坐标系
  if (names != [ "joint1", "joint2", "joint3",] or frame != "arm_base"):
    raise ValueError(f"关节名称或坐标系不正确: {names}, {frame}")
  # 检查轨迹数组维度
  # t的形状：(N,)
  # q、dq、ddq的形状：(N,3)
  if (
    n < 2
    or t.shape != (n,)
    or any( ref[k].shape != (n, 3) for k in ("q", "dq", "ddq"))
  ):
    raise ValueError( "参考轨迹数组维度不正确")
  # 检查数据中是否存在NaN或无穷大
  if any(not np.all(np.isfinite(v))for v in ref.values()):
    raise ValueError("参考轨迹包含非有限数值")
  # 时间必须从0开始，并且严格递增
  if ( abs(t[0]) > 1e-9 or np.any(np.diff(t) <= 0)):
    raise ValueError( "参考轨迹时间必须从0开始并严格递增")
  return source, ref

def sample_reference(time_now, ref):
  """
  根据当前仿真时间插值得到关节期望状态。

  输入：
    time_now: 当前仿真时间
    ref     ：完整参考轨迹

  返回：
    qd   ：期望关节位置
    dqd  ：期望关节速度
    ddqd ：期望关节加速度
  """

  # 提取参考轨迹数组
  t, q, dq, ddq = (
    ref[k]
    for k in ("t","q", "dq", "ddq",)
  )

  # 轨迹结束后：
  # 保持最终关节位置
  # 期望速度、加速度设置为0
  if time_now >= t[-1]:
    return ( q[-1].copy(), np.zeros(3), np.zeros(3),)

  # 对三个关节分别进行线性插值
  def interp(a):
    return np.array([
      np.interp( time_now, t, a[:, j],)
      for j in range(3)
    ])

  # 返回当前时间对应的期望状态
  return (
    interp(q),
    interp(dq),
    interp(ddq),
  )

def calculate_torque(mode,q,dq,qd,dqd,ddqd,xdd_cmd,J,osc, controller,):
  """
  根据控制模式计算任务力矩和零空间姿态力矩。

  关节空间动力学：

    M(q)q_ddot + V(q,q_dot) + G(q) = tau

  任务空间逆惯性：

    A = J M^-1 J^T

  严格操作空间惯性：

    Lambda = A^-1

  任务力矩与姿态力矩：

    tau_total = tau_task + tau_posture

  定义关节动力学偏置：

    bias = V + G

  当 J 满行秩、投影器精确时：

    J^T A^-1 (
      xdd_cmd - Jdot*dq + J*M^-1*bias
    ) + NT*tau0

  等价于：

    bias
    + J^T A^-1 (xdd_cmd - Jdot*dq)
    + NT*(tau0 - bias)

  其中：

    A方案使用原始严格操作空间计算路径。

    B方案对任务逆惯性和投影器同时进行正则化。

    C方案仅对任务逆惯性进行正则化,
    零空间仍然使用精确投影器。

  B和C都保留完整的关节空间偏置补偿 V+G。
  """

  # A方案：严格操作空间动力学
  if mode == "A_strict":

    # 调用原有操作空间动力学接口
    # 内部包含 Lambda、mu、p 的计算
    dyn = osc.calculate_task_force( q,dq,xdd_cmd,)
    # 提取动力学量
    M = dyn["M"]
    V = dyn["V"]
    G = dyn["G"]
    # 主任务控制力矩
    tau_task = dyn["tau"]
    # 精确动态一致零空间力矩投影器
    NT = ( osc.calculate_null_space_projector(q)["N_T"])
    # A = J M^-1 J^T
    # 使用solve，避免显式求M的逆
    A = J @ np.linalg.solve( M, J.T,)
    # 对A进行对称化后求最小特征值
    alpha_min = float( np.linalg.eigvalsh( 0.5 * (A + A.T))[0])
    # 严格方案不使用正则化
    epsilon = 0.0
    # 参考轨迹对应的关节逆动力学力矩
    tau_id = ( M @ ddqd + V + G)
    # 姿态控制原始力矩
    # 包含逆动力学前馈、位置反馈、速度反馈
    tau0 = (tau_id + K_P0 @ (qd - q) + K_D0 @ (dqd - dq) )
    # 将姿态力矩投影到动态一致零空间
    tau_posture = NT @ tau0
  # B/C方案：正则化操作空间动力学
  else:
    # 当前真实关节状态对应的质量矩阵
    M = osc.dynamics.calculate_mass_matrix(q)
    # 速度相关项
    V = osc.dynamics.calculate_velocity_term(q, dq,)
    # 重力项
    G = osc.dynamics.calculate_gravity(q)
    # Jacobian时间导数
    Jdot = controller.kinematics.jacobian_dot(q,dq,)
    # M^-1 J^T
    Minv_JT = np.linalg.solve( M, J.T, )
    # A = J M^-1 J^T
    A = J @ Minv_JT
    # 数值对称化
    A = 0.5 * (A + A.T)
    # A的最小特征值
    alpha_min = float(np.linalg.eigvalsh(A)[0])
    # A必须是对称正定矩阵
    if ( not np.isfinite(alpha_min) or alpha_min <= 0):
      raise RuntimeError( "任务空间逆惯性矩阵不是正定矩阵")
    
    # 计算正则化系数
    # 原始操作空间惯性：
    # Lambda = A^-1
    # 正则化后：
    # Lambda_eps = (A + epsilon*I)^-1
    # 通过epsilon限制Lambda_eps的最大特征值
    epsilon = max(0.0, 1.0 / LAMBDA_EIGENVALUE_CAP - alpha_min,)
    # 正则化后的任务空间逆惯性矩阵
    A_eps = ( A + epsilon * np.eye(2))
    # B方案：阻尼零空间投影器
    if mode == "B_all_damped":
      # N_T = I - J^T (A+epsilon I)^-1 J M^-1
      # epsilon > 0时，它通常不再是严格投影器
      NT = (
        np.eye(3)
        - J.T @ np.linalg.solve( A_eps, J @ np.linalg.solve( M, np.eye(3),),)
      )
    # C方案：精确零空间基投影器
    elif mode == "C_split":
      # 使用SVD分解Jacobian
      # J = U Sigma V^T
      # 对于2×3且满行秩的J：
      # 零空间维度为1
      _, singular_values, Vt = np.linalg.svd( J, full_matrices=True,)
      # 检查最小奇异值
      if singular_values[-1] < 0.005:
        raise RuntimeError("Jacobian过于接近奇异状态" )
      # 提取Jacobian的零空间基
      # Z的形状：(3,1)
      Z = Vt[-1:, :].T
      # 动态一致零空间力矩投影器
      # NT = M Z (Z^T M Z)^-1 Z^T
      NT = ( M @ Z @ np.linalg.solve( Z.T @ M @ Z, Z.T,))
    else:
      raise ValueError(f"未知控制模式: {mode}")

    # 任务力矩与姿态力矩
    # 关节空间动力学偏置项
    bias = V + G
    # 任务空间加速度右端项
    # x_ddot = J q_ddot + Jdot q_dot
    # 因此：
    # J q_ddot = xdd_cmd - Jdot q_dot
    task_accel_rhs = ( xdd_cmd - Jdot @ dq)

    # 主任务控制力矩
    # tau_task =
    #   bias + J^T (A+epsilon I)^-1 task_accel_rhs
    tau_task = ( bias + J.T @ np.linalg.solve( A_eps,task_accel_rhs,))
    # 关节空间逆动力学前馈
    tau_id = (M @ ddqd+ bias)

    # 原始姿态控制力矩
    tau0 = ( tau_id + K_P0 @ (qd - q) + K_D0 @ (dqd - dq))
    # 保留完整偏置补偿
    # 只对姿态控制中扣除bias后的部分投影
    tau_posture = ( NT @ (tau0 - bias))
  # 合成最终力矩
  # 检查任务空间逆惯性矩阵的正定性
  if alpha_min <= 0:
    raise RuntimeError("任务空间逆惯性矩阵不是正定矩阵")

  # 主任务力矩 + 二级姿态力矩
  # 此处尚未施加关节力矩限幅
  tau_raw = ( tau_task + tau_posture)

  # 测量零空间加速度泄漏
  # 只考虑姿态力矩引起的任务空间加速度
  # a_leak = J M^-1 tau_posture
  # 理想动态一致零空间中：
  # a_leak 应接近0
  a_leak = ( J @ np.linalg.solve( M, tau_posture,))

  # 检查最终力矩是否为有效数值
  if not np.all(np.isfinite(tau_raw)):
    raise RuntimeError("控制力矩包含非有限数值")

  # 返回：
  # 原始力矩、正则化参数、最小特征值、
  # 姿态加速度泄漏模长
  return ( tau_raw, epsilon, alpha_min, np.linalg.norm(a_leak),)

def report_case(r):
  """
  输出当前A/B/C方案的完成状态、
  数值稳定性指标、力矩及跟踪误差。
  """
  print( f"\n========== {r['mode']} ==========")
  # 是否完成整个参考轨迹及保持阶段
  print("是否完成完整轨迹及保持阶段:", "是" if r["completed"] else "否",)
  # 如果失败，打印失败原因
  if r["failure"]:
    print( "安全退出原因:", r["failure"], )
  # 没有有效数据时直接返回
  if len(r["history"]["time"]) == 0:
    return
  # 数值稳定性与零空间指标
  print( "实际最小奇异值:", f"{r['min_sigma']:.9f}",)
  print("最大正则化 epsilon:", f"{r['max_eps']:.9f}",)
  print("正则化启动控制周期数:",r["eps_active_steps"],)
  print( "最大姿态力矩加速度泄漏 [m/s^2]:", f"{r['max_leak']:.9e}",)

  # 关节力矩统计

  print( "实际最大各关节力矩 [Nm]:", r["max_tau"],)
  print( "限幅前最大各关节力矩 [Nm]:", r["max_tau_raw"],)
  print("触发力矩限幅的控制周期数:", r["saturated_steps"],)

  # 跟踪性能统计
  if r["completed"]:
    print( "运动阶段末端 RMS [mm]:", r["rms_x_mm"], )
    print( "运动阶段末端最大绝对误差 [mm]:", r["max_x_mm"],)
    print( "运动阶段关节 RMS [rad]:", r["rms_q"],)
    print( "末端保持结束误差 [mm]:",r["final_x_mm"],)
  else:
    # 没有完整运行时，不比较全程RMS
    print("本方案未完整执行，" "不能与其他方案比较整程 RMS")

def write_comparison(out, results):
  """
  将所有方案的统计结果写入CSV。

  同时生成：
    末端X误差曲线
    末端Y误差曲线
    正则化参数曲线
    Jacobian最小奇异值曲线
    零空间加速度泄漏曲线
  """

  # 定义CSV表头

  cols = [
    "mode",
    "completed",
    "failure",
    "rms_x_mm",
    "rms_y_mm",
    "max_x_mm",
    "max_y_mm",
    "rms_q1_rad",
    "rms_q2_rad",
    "rms_q3_rad",
    "min_sigma",
    "max_epsilon",
    "epsilon_active_steps",
    "max_accel_leak_mps2",
    "max_tau1_Nm",
    "max_tau2_Nm",
    "max_tau3_Nm",
    "saturated_control_steps",
  ]

  # 写入统计结果
  # 使用with自动关闭CSV文件
  with ( out / "summary.csv").open( "w", encoding="utf-8",newline="",) as f:
    # 创建字典格式的CSV写入器
    writer = csv.DictWriter( f, fieldnames=cols,)
    # 写入表头
    writer.writeheader()
    # 遍历每个实验方案
    for r in results:
      # 基础结果
      row = {
        "mode": r["mode"],
        "completed": r["completed"],
        "failure": r["failure"] or "",
      }
      # 如果有有效数据，保存诊断指标
      if "min_sigma" in r:
        row.update({
          "min_sigma": r["min_sigma"],
          "max_epsilon": r["max_eps"],
          "epsilon_active_steps": r["eps_active_steps"],
          "max_accel_leak_mps2": r["max_leak"],
          "max_tau1_Nm": r["max_tau"][0],
          "max_tau2_Nm": r["max_tau"][1],
          "max_tau3_Nm": r["max_tau"][2],
          "saturated_control_steps": r["saturated_steps"],
        })
      # 只有完整完成的实验才保存全程RMS
      if r["completed"]:
        row.update({
          "rms_x_mm": r["rms_x_mm"][0],
          "rms_y_mm": r["rms_x_mm"][1],
          "max_x_mm": r["max_x_mm"][0],
          "max_y_mm": r["max_x_mm"][1],
          "rms_q1_rad": r["rms_q"][0],
          "rms_q2_rad": r["rms_q"][1],
          "rms_q3_rad": r["rms_q"][2],
        })
      # 写入当前方案的一行统计数据
      writer.writerow(row)

  # 绘制A/B/C对比曲线
  # 每个元组分别表示：
  # (数据字段, 纵轴名称, 图片文件名, 单位缩放)
  for key, ylabel, filename, factor in (
    (
      "e_x",
      "End-effector X error (mm)",
      "compare_error_x.png",
      1000,
    ),

    (
      "e_x",
      "End-effector Y error (mm)",
      "compare_error_y.png",
      1000,
    ),

    (
      "epsilon",
      "Regularization epsilon",
      "compare_epsilon.png",
      1,
    ),

    (
      "sigma_actual",
      "Minimum singular value",
      "compare_sigma.png",
      1,
    ),

    (
      "posture_accel_leak",
      "Posture acceleration leakage (m/s^2)",
      "compare_posture_leak.png",
      1,
    ),

  ):

    # 创建一张新图
    fig, ax = plt.subplots( figsize=(11, 4))
    # 在同一张图上绘制三个实验方案
    for r in results:
      # 获取当前方案的历史数据
      h = r["history"]
      # 无有效数据则跳过
      if len(h["time"]) == 0:
        continue
      # 读取对应字段
      values = h[key]
      # 如果是末端误差，需要区分X和Y
      if key == "e_x":
        # X图选择第0列
        # Y图选择第1列
        values = values[:,0 if "_x." in filename else 1]
      # 绘制当前方案曲线
      ax.plot(
        h["time"],
        values * factor,
        label=r["mode"],
      )

    # 设置横坐标
    ax.set_xlabel("Time (s)")
    # 设置纵坐标
    ax.set_ylabel(ylabel)
    # 显示网格
    ax.grid(True)
    # 显示图例
    ax.legend()
    # 自动调整布局
    fig.tight_layout()
    # 保存图片到输出目录
    fig.savefig( out / filename, dpi=150,)
    # 关闭当前图片，释放内存
    plt.close(fig)

def run_case(mode, ref, delay_ms=0):
  """
  在同一个理想动力学模型上执行一组闭环实验。

  输入：
    mode      ：A_strict / B_all_damped / C_split
    ref       ：与原 A/B/C 测试相同的 MoveIt 参考轨迹
    delay_ms  ：力矩命令生效延迟（必须是控制周期整数倍）

  返回：
    与原 A/B/C 程序相同的结果字典，并附加命令年龄记录。
  """

  #  复用原控制周期、物理积分周期和安全参数
  dt_sim = DT_SIM
  dt_ctrl = DT_CTRL
  torque_limit = TORQUE_LIMIT

  steps_per_control = round(dt_ctrl / dt_sim)
  delay_control_cycles = round((delay_ms / 1000.0) / dt_ctrl)

  if abs(dt_ctrl / dt_sim - steps_per_control) > 1e-12:
    raise ValueError("控制周期必须是动力学积分周期的整数倍")

  if delay_ms < 0 or abs(delay_control_cycles * dt_ctrl * 1000 - delay_ms) > 1e-8:
    raise ValueError("延迟必须为非负的控制周期整数倍，例如 0、10、20 ms")

  #  控制器和机器人动力学与原测试完全一致
  controller = OperationalSpaceController(
    kp=(10.0, 10.0),
    kd=(4.43, 4.43),
  )
  osc = OperationalSpaceDynamics()
  osc.dynamics.g = 9.8

  # 设置原来的初始关节状态
  q = ref["q"][0].copy()
  dq = ref["dq"][0].copy()

  # tau_applied：真正送入动力学积分的力矩
  # pending：队列元素为 (计划生效积分步, 指令计算时刻, 关节力矩)
  tau_applied = np.zeros(3)
  applied_source_time = 0.0
  pending = deque()

  stop_time = float(ref["t"][-1] + HOLD_SECONDS)
  n_steps = int(np.ceil(stop_time / dt_sim))

  # 原有全部指标保留，再添加命令、执行与年龄记录
  fields = (
    "time", "q", "q_d", "e_q", "x", "x_d", "e_x",
    "tau", "tau_raw", "sigma_actual", "epsilon",
    "lambda_max", "lambda_max_regularized", "posture_accel_leak",
    "tau_computed", "command_age_ms",
  )
  history = {key: [] for key in fields}
  saturated_steps = 0
  failure = None

  #  开始完整闭环仿真
  for step in range(n_steps + 1):
    time_now = step * dt_sim

    try:
      if step % steps_per_control == 0:

        #  先取出已经达到生效时间的旧指令
        # 例如 20 ms 延迟：t=20 ms 时才施加 t=0 的指令
        while pending and pending[0][0] <= step:
          _, applied_source_time, tau_applied = pending.popleft()

        #  从同一份 MoveIt 文件获取期望轨迹
        qd, dqd, ddqd = sample_reference(time_now, ref)
        Jd = controller.kinematics.jacobian(qd)
        xd = controller.kinematics.forward_kinematics(qd)
        dxd = Jd @ dqd
        ddxd = (
          Jd @ ddqd
          + controller.kinematics.jacobian_dot(qd, dqd) @ dqd
        )

        #  在真实仿真状态上检查奇异性与关节安全边界
        J = controller.kinematics.jacobian(q)
        sigma = float(np.linalg.svd(J, compute_uv=False)[-1])
        if (
          sigma < 0.005
          or np.max(np.abs(q)) > 3.14
          or np.max(np.abs(dq)) > 10.0
          or np.linalg.norm(q - qd) > 1.0
        ):
          raise RuntimeError(
            f"安全检查失败：sigma={sigma:.6g}, q={q}, dq={dq}, "
            f"关节误差范数={np.linalg.norm(q-qd):.6f}"
          )

        #  计算 OSC 主任务加速度命令
        task_cmd = controller.calculate_task_acceleration_command(
          q, dq, xd, dxd, ddxd
        )

        #  完全复用已验证的 A/B/C 力矩计算和正则化逻辑
        tau_raw, epsilon, alpha_min, accel_leak = calculate_torque(
          mode,
          q,
          dq,
          qd,
          dqd,
          ddqd,
          task_cmd["x_ddot_command"],
          J,
          osc,
          controller,
        )

        #  限幅依然作用于控制器算出的命令
        if np.any(np.abs(tau_raw) > torque_limit):
          saturated_steps += 1
        tau_computed = np.clip(tau_raw, -torque_limit, torque_limit)

        #  唯一的对照变量：何时把新力矩施加给机器人
        if delay_control_cycles == 0:
          tau_applied = tau_computed.copy()
          applied_source_time = time_now
        else:
          if step == 0:
            # 首个命令预置为保持值，避免启动时有20ms自由下落
            # 之后的新命令严格等待指定的完整控制周期
            tau_applied = tau_computed.copy()
            applied_source_time = time_now

          due_step = step + delay_control_cycles * steps_per_control
          pending.append((due_step, time_now, tau_computed.copy()))

        #  分别记录新计算指令与实际施加力矩
        # tau: 物理积分实际使用的力矩（有延迟）
        # tau_computed: 控制器本次新计算并限幅的力矩
        record = {
          "time": time_now,
          "q": q.copy(),
          "q_d": qd.copy(),
          "e_q": (qd - q).copy(),
          "x": task_cmd["x"].copy(),
          "x_d": xd.copy(),
          "e_x": (xd - task_cmd["x"]).copy(),
          "tau": tau_applied.copy(),
          "tau_raw": tau_raw.copy(),
          "sigma_actual": sigma,
          "epsilon": epsilon,
          "lambda_max": 1.0 / alpha_min,
          "lambda_max_regularized": 1.0 / (alpha_min + epsilon),
          "posture_accel_leak": accel_leak,
          "tau_computed": tau_computed.copy(),
          "command_age_ms": (time_now - applied_source_time) * 1000.0,
        }

        for key, value in record.items():
          history[key].append(value)

      if step == n_steps:
        break

      #  物理积分：始终使用 tau_applied，而不是 tau_computed
      M = osc.dynamics.calculate_mass_matrix(q)
      V = osc.dynamics.calculate_velocity_term(q, dq)
      G = osc.dynamics.calculate_gravity(q)

      qdd = np.linalg.solve(M, tau_applied - V - G)

      #  与原实验相同的半隐式欧拉积分
      dq = dq + qdd * dt_sim
      q = q + dq * dt_sim

      if not (np.all(np.isfinite(q)) and np.all(np.isfinite(dq))):
        raise RuntimeError("动力学积分发散")

    except (RuntimeError, np.linalg.LinAlgError, FloatingPointError) as err:
      failure = f"t={time_now:.3f}s: {err}"
      break

  #  使用原 A/B/C 程序一致的指标定义
  h = {key: np.asarray(values) for key, values in history.items()}
  completed = (
    failure is None
    and len(h["time"]) > 0
    and h["time"][-1] >= stop_time - dt_ctrl - 1e-8
  )

  result = {
    "mode": mode,
    "completed": completed,
    "failure": failure,
    "history": h,
    "saturated_steps": saturated_steps,
  }

  if len(h["time"]) > 0:
    result["max_eps"] = float(np.max(h["epsilon"]))
    result["eps_active_steps"] = int(np.count_nonzero(h["epsilon"] > 1e-12))
    result["min_sigma"] = float(np.min(h["sigma_actual"]))
    result["max_leak"] = float(np.max(h["posture_accel_leak"]))
    result["max_tau"] = np.max(np.abs(h["tau"]), axis=0)
    result["max_tau_raw"] = np.max(np.abs(h["tau_raw"]), axis=0)
    result["max_cmd_age_ms"] = float(np.max(h["command_age_ms"]))

  if completed:
    moving = h["time"] <= ref["t"][-1]
    result["rms_x_mm"] = (
      1000.0 * np.sqrt(np.mean(h["e_x"][moving] ** 2, axis=0))
    )
    result["max_x_mm"] = (
      1000.0 * np.max(np.abs(h["e_x"][moving]), axis=0)
    )
    result["rms_q"] = np.sqrt(np.mean(h["e_q"][moving] ** 2, axis=0))
    result["final_x_mm"] = 1000.0 * h["e_x"][-1]

  return result


def calculate_posture_torque(
  J, M, D, A,
  q, dq,
  q_desired, dq_desired, ddq_desired,
  Kp0, Kd0,
):
  """计算动态一致零空间姿态力矩。

  当前调用方已在主任务参考力矩中补偿 V+G，
  因此这里不重复添加动力学偏置。

  返回：
    tau_posture：零空间姿态力矩
    leak：姿态力矩引起的任务加速度泄漏
  """

  # 计算动态一致零空间力矩投影器
  NT = (
    np.eye(M.shape[0])
    - J.T @ np.linalg.solve(A, D)
  )

  # 计算关节姿态误差
  e_q = q_desired - q
  e_dq = dq_desired - dq

  # 计算姿态控制的原始力矩
  tau0 = (
    M @ ddq_desired
    + Kp0 @ e_q
    + Kd0 @ e_dq
  )

  # 将姿态力矩投影到任务动力学零空间
  tau_posture = NT @ tau0

  # 验证零空间姿态力矩不会改变末端加速度
  leak = np.linalg.norm(D @ tau_posture)

  return tau_posture, leak



