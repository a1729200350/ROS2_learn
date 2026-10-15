"""离线 QP 实验的公共辅助函数。"""
import numpy as np
import scipy.sparse as sp
import osqp
from scipy.optimize import linprog

def report_result( name, tau, D, *,acceleration_target, torque_limit,):
  """输出简化任务力矩实验的结果。

  此处使用动力学偏置已补偿的增量模型：
      a_actual = D @ tau

  tau 为任务力矩，不是包含完整动力学偏置的总力矩。
  """
  a_actual = D @ tau
  a_error = acceleration_target - a_actual
  margin = torque_limit - np.abs(tau)
  print(f"\n========== {name} ==========")
  print("关节任务力矩 [Nm]:")
  print(tau)
  print("实际任务加速度 [m/s^2]:")
  print(a_actual)
  print( "加速度误差范数 [m/s^2]:", np.linalg.norm(a_error),)
  print( "最小力矩余量 [Nm]:", np.min(margin),)
  print( "是否满足力矩约束:", bool(np.all(margin >= -1e-5)),)

def calculate_task_matrices(q, controller, osc):
  """返回 J、M、D=J M^-1 和 A=J M^-1 J.T。"""
  J = controller.kinematics.jacobian(q)
  M = osc.dynamics.calculate_mass_matrix(q)
  n = M.shape[0]

  D = J @ np.linalg.solve(M, np.eye(n))
  A = J @ np.linalg.solve(M, J.T)
  A = 0.5 * (A + A.T)

  return J, M, D, A

def check_task_feasibility(D, b, torque_limit):
  """检查 D @ tau = b 与力矩边界能否同时满足。"""
  return linprog(
    c=np.zeros(D.shape[1]),
    A_eq=D,
    b_eq=b,
    bounds=[(-float(v), float(v)) for v in torque_limit],
    method="highs",
  )

def _create_qp(P, f, constraints, lower, upper, warm_start):
  """创建求解器，保留原有求解精度与初始化设置。"""
  solver = osqp.OSQP()
  solver.setup(
    P=sp.csc_matrix(P),
    q=f,
    A=sp.csc_matrix(constraints),
    l=lower,
    u=upper,
    verbose=False,
    eps_abs=1e-6,
    eps_rel=1e-6,
    max_iter=20000,
    polishing=True,
    adaptive_rho=True,
  )
  if warm_start is not None:
    solver.warm_start(x=warm_start)
  return solver

def create_soft_task_solver(D, b, tau_reference, torque_limit, *, rho, warm_start=None):
  """创建软任务QP，返回可继续更新的求解器。"""
  n = D.shape[1]
  P = D.T @ D + rho * np.eye(n)
  f = -D.T @ b - rho * tau_reference
  return _create_qp(
    P, f, sp.eye(n, format="csc"),
    -torque_limit, torque_limit, warm_start,
  )

def solve_soft_task_qp(D, b, tau_reference, torque_limit, *, rho, warm_start=None):
  """求解一次软任务QP，保持已有调用方式。"""
  solver = create_soft_task_solver(
    D, b, tau_reference, torque_limit,
    rho=rho, warm_start=warm_start,
  )
  return solver.solve()

def create_hard_task_solver(D, b, tau_reference, torque_limit, *, warm_start=None):
  """创建硬任务QP，约束矩阵按关节力矩、任务等式排列。"""
  n = D.shape[1]
  constraints = sp.vstack([sp.eye(n, format="csc"), sp.csc_matrix(D)], format="csc")
  lower = np.concatenate([-torque_limit, b])
  upper = np.concatenate([torque_limit, b])
  return _create_qp(
    sp.eye(n, format="csc"), -tau_reference.copy(),
    constraints, lower, upper, warm_start,
  )

def solve_hard_task_qp(D, b, tau_reference, torque_limit, *, warm_start=None):
  """求解一次硬任务QP，保持已有调用方式。"""
  solver = create_hard_task_solver(
    D, b, tau_reference, torque_limit, warm_start=warm_start,
  )
  return solver.solve()

def print_solver_info(name, result):
  print(f"\n========== {name} ==========")
  print("求解状态:", result.info.status)
  print("状态编号:", result.info.status_val)
  print("迭代次数:", result.info.iter)
  print("原始约束残差:", result.info.prim_res)
  print("对偶残差:", result.info.dual_res)

def get_solved_torque(result, torque_limit):
  """仅在预期成功的实验中调用；不可行性实验直接检查 result。"""
  if result.info.status_val != 1:
    raise RuntimeError( f"QP未达到要求精度: {result.info.status}")
  tau = np.asarray(result.x, dtype=float)
  if not np.all(np.isfinite(tau)):
    raise RuntimeError("QP力矩包含非有限数值")
  if np.any(np.abs(tau) > torque_limit + 1e-5):
    raise RuntimeError("QP结果超过关节力矩限制")
  return tau

def calculate_dynamics_bias(q, dq, controller, osc):
  """返回速度项、重力项、总偏置，以及 Jdot @ dq。"""
  V = np.asarray(osc.dynamics.calculate_velocity_term(q, dq), dtype=float).reshape(len(q))
  G = np.asarray(osc.dynamics.calculate_gravity(q), dtype=float).reshape(len(q))
  h = V + G
  jdq = controller.kinematics.jacobian_dot(q, dq) @ dq
  return V, G, h, jdq


def report_total_result(name, tau, D, h, jdq, *, acceleration_target, torque_limit):
  """输出总力矩结果，返回实际加速度、误差范数和各关节力矩余量。"""
  a_actual = D @ (tau - h) + jdq
  error = np.linalg.norm(a_actual - acceleration_target)
  margin = torque_limit - np.abs(tau)

  print(f"\n========== {name} ==========")
  print("关节总力矩 [Nm]:", tau)
  print("实际任务加速度 [m/s^2]:", a_actual)
  print("加速度误差范数 [m/s^2]:", error)
  print("各关节剩余力矩余量 [Nm]:", margin)
  print("是否满足总力矩约束:", bool(np.all(margin >= -1e-5)))
  return a_actual, error, margin

