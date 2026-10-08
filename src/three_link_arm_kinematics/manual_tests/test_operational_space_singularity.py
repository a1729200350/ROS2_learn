
import numpy as np
import matplotlib.pyplot as plt
from three_link_arm_kinematics.dynamics_model import DynamicsModel
from three_link_arm_kinematics.operational_space_kinematics import ( OperationalSpaceKinematics,)
from simulation_core import run_osc_simulation
def main():
  np.set_printoptions(precision=8, suppress=True)
  kinematics = OperationalSpaceKinematics()
  cases = [
    ("Normal", [0.5, -0.8, 1.2]),
    ("Less bent", [0.5, -0.3, 0.3]),
    ("Near straight", [0.5, -0.03, 0.03]),
    ("Fully straight", [0.5, 0.0, 0.0]),
  ]
  results = {}
  print("\n========== 奇异点边界测试 ==========\n")
  for name, q_values in cases:
    q_initial = np.asarray(q_values, dtype=float)

    # 初始 Jacobian
    J_initial = kinematics.jacobian(q_initial)
    singular_values = np.linalg.svd(J_initial,compute_uv=False,)
    sigma_max = singular_values[0]
    sigma_min = singular_values[-1]

    print(f"\n--- {name} ---")
    print("Initial q =", q_initial)
    print("Initial sigma_min =", sigma_min)

    # 严格 OSC 不允许在奇异点直接求逆
    if sigma_min < 1e-8:
      print("Status: Singular initial configuration")
      print("Strict OSC inverse is undefined here.")
      continue
    print("Initial condition number =", sigma_max / sigma_min)
    # 为不同初始构型构造统一的径向内移轨迹
    x_start = kinematics.forward_kinematics(q_initial)
    radial_direction = x_start / np.linalg.norm(x_start)
    goal_delta = -0.05 * radial_direction
    # 控制器模型与机器人模型完全一致
    controller_model = DynamicsModel()
    plant_model = DynamicsModel()
    try:
      result = run_osc_simulation(
        plant_model=plant_model,
        controller_model=controller_model,
        dt_control=0.002,
        dt_sim=0.001,
        enable_posture=True,
        q_initial=q_initial,
        goal_delta=goal_delta,
      )
    except (
      np.linalg.LinAlgError,
      FloatingPointError,
      ValueError,
      OverflowError,
    ) as error:
      print("Status: Simulation failed")
      print("Reason:", error)
      continue
    results[name] = result
    error_norm = np.linalg.norm(
      result["error"],
      axis=1,
    )
    min_sigma_trajectory = np.min(result["sigma_min"])
    max_joint_speed = np.max(np.abs(result["q_dot"]))
    max_torque = np.max(np.abs(result["tau"]))
    
    print("Status: Completed")
    print("Min sigma during motion =", min_sigma_trajectory)
    print("Max tracking error (mm) =", np.max(error_norm) * 1000)
    print("Final tracking error (mm) =", error_norm[-1] * 1000)
    print("Max joint speed (rad/s) =", max_joint_speed)
    print("Max joint torque (Nm) =", max_torque)

  # 统一绘制已完成工况的跟踪误差
  plt.figure()
  for name, result in results.items():
    error_mm = (np.linalg.norm(result["error"], axis=1)* 1000.0 )
    plt.plot( result["time"], error_mm,label=name,)

  plt.xlabel("Time [s]")
  plt.ylabel("Task Error Norm [mm]")
  plt.title("OSC Singularity Boundary Comparison")
  plt.grid(True)
  plt.legend()
  plt.tight_layout()
  plt.show()


if __name__ == "__main__":
  main()
