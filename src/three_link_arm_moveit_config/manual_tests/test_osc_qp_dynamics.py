"""完整动力学下，检查总力矩可行性、QP分配和非零速度更新。"""
import numpy as np
from three_link_arm_kinematics.operational_space_controller import (OperationalSpaceController,)
from three_link_arm_kinematics.operational_space_dynamics import (OperationalSpaceDynamics,)
from osc_simulation_common import calculate_posture_torque
from osc_qp_common import (
  calculate_task_matrices,
  calculate_dynamics_bias,
  check_task_feasibility,
  create_hard_task_solver,
  print_solver_info,
  get_solved_torque,
  report_total_result,
)

Q_TEST = np.array([-0.00395574, 0.03543855, 0.04522382])
B_TEST = np.array([0.9, -0.45])
TAU_LIMIT = np.array([10.0, 10.0, 10.0])
DQ_MOVING = np.array([0.6, -0.4, 0.3])


def main():
  controller = OperationalSpaceController()
  osc = OperationalSpaceDynamics()
  J, M, D, A = calculate_task_matrices(Q_TEST, controller, osc)
  solver = None

  for name, dq in [("静止状态", np.zeros(3)), ("非零速度状态", DQ_MOVING)]:
    V, G, h, jdq = calculate_dynamics_bias(Q_TEST, dq, controller, osc)
    b = B_TEST - jdq + D @ h
    tau_nom = h + J.T @ np.linalg.solve(A, B_TEST - jdq)

    print(f"\n========== {name} ==========")
    print("关节速度 [rad/s]:", dq)
    print("速度相关力矩 V [Nm]:", V)
    print("重力项 G [Nm]:", G)
    print("总偏置 h [Nm]:", h)
    print("Jdot @ dq [m/s^2]:", jdq)
    print("总力矩等式右端:", b)
    report_total_result(
      "原OSC参考", tau_nom, D, h, jdq,
      acceleration_target=B_TEST, torque_limit=TAU_LIMIT,
    )

    feasibility = check_task_feasibility(D, b, TAU_LIMIT)
    print("总力矩任务是否可行:", feasibility.success)
    if not feasibility.success:
      if solver is None:
        raise RuntimeError(f"静止状态任务不可行: {feasibility.message}")
      print("运动状态任务不可行，需要改用软任务QP")
      return

    report_total_result(
      "LP可行解", feasibility.x, D, h, jdq,
      acceleration_target=B_TEST, torque_limit=TAU_LIMIT,
    )
    if solver is None:
      solver = create_hard_task_solver(
        D, b, tau_nom, TAU_LIMIT, warm_start=feasibility.x,
      )
    else:
      # 姿态固定，只更新速度相关的目标与约束右端。
      solver.update(
        q=-tau_nom.copy(),
        l=np.concatenate([-TAU_LIMIT, b]),
        u=np.concatenate([TAU_LIMIT, b]),
      )
      solver.warm_start(x=feasibility.x)

    result = solver.solve()
    print_solver_info("总力矩硬任务QP", result)
    tau_qp = get_solved_torque(result, TAU_LIMIT)

    # 从完整关节动力学独立验证任务加速度。
    qdd_actual = np.linalg.solve(M, tau_qp - h)
    xdd_actual = J @ qdd_actual + jdq
    task_error = np.linalg.norm(xdd_actual - B_TEST)
    torque_margin = TAU_LIMIT - np.abs(tau_qp)
    null_error = np.linalg.norm(D @ (tau_qp - tau_nom))
    distance_lp = np.linalg.norm(feasibility.x - tau_nom)
    distance_qp = np.linalg.norm(tau_qp - tau_nom)

    print("QP总力矩 [Nm]:", tau_qp)
    print("实际末端加速度:", xdd_actual)
    print("任务误差:", task_error)
    print("力矩余量:", torque_margin)
    print("力矩修正的零空间残差:", null_error)
    print("LP/QP距离原参考:", distance_lp, distance_qp)

    if task_error > 1e-4:
      raise RuntimeError("任务加速度误差超限")
    if np.any(torque_margin < -1e-5):
      raise RuntimeError("总力矩约束被违反")
    if null_error > 1e-4:
      raise RuntimeError("力矩修正未满足任务零空间条件")
    # 只在非零速度状态下进行姿态控制对照
    if name == "非零速度状态":
      # 选择一个不完全伸直的测试目标姿态
      q_posture_desired = np.array([0.5, -0.8, 1.2])
      dq_posture_desired = np.zeros(3)
      ddq_posture_desired = np.zeros(3)
      # 保持与之前闭环仿真一致的姿态增益
      Kp0 = np.eye(3) * 1.0
      Kd0 = np.eye(3) * 0.5
      # 计算动态一致零空间姿态力矩
      tau_posture, posture_leak = calculate_posture_torque(
        J, M, D, A,
        Q_TEST, dq,
        q_posture_desired,
        dq_posture_desired,
        ddq_posture_desired,
        Kp0, Kd0,
      )
      # 在现有参考总力矩上增加姿态项
      tau_nom_posture = tau_nom + tau_posture
      print("\n========== 零空间姿态控制 ==========")
      print("姿态目标 [rad]:", q_posture_desired)
      print("零空间姿态力矩 [Nm]:", tau_posture)
      print("姿态加速度泄漏:", posture_leak)
      print("新的参考总力矩 [Nm]:", tau_nom_posture)
      # 验证参考总力矩仍然实现原任务加速度
      reference_error = np.linalg.norm( D @ (tau_nom_posture - h) + jdq - B_TEST)
      print("参考加速度误差:", reference_error)
      # 复用当前运动状态下的硬任务QP
      # 仅改变目标函数，不修改任务等式和力矩限制
      solver.update(q=-tau_nom_posture.copy(),)
      solver.warm_start(x=tau_qp)
      result_posture = solver.solve()
      print_solver_info("加入姿态目标后的硬任务QP",result_posture,)
      tau_qp_posture = get_solved_torque(result_posture, TAU_LIMIT,)
      # 使用公共输出函数验证完整动力学
      _, task_error_posture, margin_posture = report_total_result(
        "姿态控制QP最终结果",
        tau_qp_posture,
        D, h, jdq,
        acceleration_target=B_TEST,
        torque_limit=TAU_LIMIT,
      )

      # 验证QP修正是否仍处于任务动力学零空间
      correction_leak = np.linalg.norm( D @ (tau_qp_posture - tau_nom_posture))
      # 与加入姿态目标之前的QP输出进行对比
      torque_change = tau_qp_posture - tau_qp
      print("\n========== 姿态控制对照 ==========")
      print("加入姿态目标前的QP力矩:", tau_qp)
      print("加入姿态目标后的QP力矩:", tau_qp_posture)
      print("QP力矩变化 [Nm]:", torque_change)
      print("QP修正的零空间残差:", correction_leak)
      # 验证数值结果
      if posture_leak > 1e-8:
        raise RuntimeError("姿态力矩产生任务加速度泄漏")
      if reference_error > 1e-8:
        raise RuntimeError("姿态参考破坏了主任务")
      if task_error_posture > 1e-4:
        raise RuntimeError("QP未满足末端加速度任务")
      if np.any(margin_posture < -1e-5):
        raise RuntimeError("QP总力矩超限")
      if correction_leak > 1e-4:
        raise RuntimeError("QP修正不在任务零空间内")

      print("\n零空间姿态控制与QP验证通过")


  print("\n完整动力学QP对照完成")


if __name__ == "__main__":
  main()