"""动力学偏置已补偿时，比较截断、软任务、硬任务和不可行目标。"""
import numpy as np
from scipy.optimize import linprog
from three_link_arm_kinematics.operational_space_controller import (OperationalSpaceController,)
from three_link_arm_kinematics.operational_space_dynamics import (OperationalSpaceDynamics,)
from osc_qp_common import (
  calculate_task_matrices,
  check_task_feasibility,
  create_soft_task_solver,
  create_hard_task_solver,
  print_solver_info,
  get_solved_torque,
  report_result,
)

Q_TEST = np.array([-0.00395574, 0.03543855, 0.04522382])
B_TEST = np.array([0.9, -0.45])
TAU_LIMIT = np.array([10.0, 10.0, 10.0])
RHO = 1e-4


def main():
  controller = OperationalSpaceController()
  osc = OperationalSpaceDynamics()
  J, M, D, A = calculate_task_matrices(Q_TEST, controller, osc)
  tau_strict = J.T @ np.linalg.solve(A, B_TEST)
  tau_clipped = np.clip(tau_strict, -TAU_LIMIT, TAU_LIMIT)

  feasibility = check_task_feasibility(D, B_TEST, TAU_LIMIT)
  print("原任务是否可行:", feasibility.success)
  if not feasibility.success:
    raise RuntimeError(f"原任务可行性检查失败: {feasibility.message}")
  print("LP可行力矩 [Nm]:", feasibility.x)
  print("LP任务残差:", np.linalg.norm(D @ feasibility.x - B_TEST))

  # 正常任务只求解一次，后续继续使用这两个求解器。
  solver_soft = create_soft_task_solver(
    D, B_TEST, tau_strict, TAU_LIMIT, rho=RHO, warm_start=tau_clipped,
  )
  result_soft = solver_soft.solve()
  print_solver_info("正常目标下的软任务QP", result_soft)
  tau_soft = get_solved_torque(result_soft, TAU_LIMIT)

  solver_hard = create_hard_task_solver(
    D, B_TEST, tau_strict, TAU_LIMIT, warm_start=tau_soft,
  )
  result_hard = solver_hard.solve()
  print_solver_info("正常目标下的硬任务QP", result_hard)
  tau_hard = get_solved_torque(result_hard, TAU_LIMIT)
  if np.linalg.norm(D @ tau_hard - B_TEST) > 1e-4:
    raise RuntimeError("硬任务QP未满足任务等式")

  for name, tau in [
    ("严格OSC", tau_strict),
    ("直接截断", tau_clipped),
    ("软任务QP", tau_soft),
    ("硬任务QP", tau_hard),
  ]:
    report_result(name, tau, D, acceleration_target=B_TEST, torque_limit=TAU_LIMIT)

  # 构造超出X方向能力上限的目标。
  max_accel_x = np.sum(np.abs(D[0, :]) * TAU_LIMIT)
  b_impossible = np.array([max_accel_x + 0.5, B_TEST[1]])
  print("\nX方向单独最大加速度:", max_accel_x)
  print("不可行目标:", b_impossible)
  solver_hard.update(
    l=np.concatenate([-TAU_LIMIT, b_impossible]),
    u=np.concatenate([TAU_LIMIT, b_impossible]),
  )
  result_hard_bad = solver_hard.solve()
  print_solver_info("不可行目标下的硬任务QP", result_hard_bad)
  if result_hard_bad.info.status_val != 3:
    raise RuntimeError(f"未得到预期的不可行状态: {result_hard_bad.info.status}")

  # 保留原力矩参考，仅改变软任务的加速度目标。
  solver_soft.update(q=-D.T @ b_impossible - RHO * tau_strict)
  result_soft_bad = solver_soft.solve()
  print_solver_info("不可行目标下的软任务QP", result_soft_bad)
  tau_soft_bad = get_solved_torque(result_soft_bad, TAU_LIMIT)
  actual_accel = D @ tau_soft_bad
  report_result(
    "允许任务误差后的结果", tau_soft_bad, D,
    acceleration_target=b_impossible, torque_limit=TAU_LIMIT,
  )

  tau_x_max = TAU_LIMIT * np.sign(D[0, :])
  a_x_max = D @ tau_x_max
  print("\n单独最大化X时的力矩:", tau_x_max)
  print("对应的完整任务加速度:", a_x_max)

  # 固定Y方向要求，分析X方向真正可用的余量。
  result_boundary = linprog(
    c=-D[0, :],
    A_eq=D[1:2, :],
    b_eq=b_impossible[1:2],
    bounds=[(-float(v), float(v)) for v in TAU_LIMIT],
    method="highs",
  )
  if not result_boundary.success:
    raise RuntimeError(f"任务边界求解失败: {result_boundary.message}")
  tau_boundary = result_boundary.x
  accel_boundary = D @ tau_boundary
  print("保持Y精确时的边界力矩:", tau_boundary)
  print("边界任务加速度:", accel_boundary)
  print("软任务的X/Y误差:", b_impossible - actual_accel)
  print("\n简化任务QP对照完成")


if __name__ == "__main__":
  main()