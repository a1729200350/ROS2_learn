import numpy as np
import matplotlib.pyplot as plt
from three_link_arm_kinematics.dynamics_model import DynamicsModel
from simulation_core import run_osc_simulation
def main():
  # link2 的真实质量和惯量变化比例
  cases = [ ("Exact + ON", 1.00, True),
    ("Mismatch + ON", 1.10, True),
    ("Exact + OFF", 1.00, False),
    ("Mismatch + OFF", 1.10, False),]
  # 保存各组实验结果
  results = {}
  for name, scale, enable_posture in cases:
    # 控制器始终使用标称模型
    controller_model = DynamicsModel()

    # 仿真机器人使用独立模型
    plant_model = DynamicsModel()

    # 只改变真实机器人的 link2
    plant_model.m2 *= scale
    plant_model.I2 *= scale
    # 运行闭环仿真
    result = run_osc_simulation(
      plant_model=plant_model,
      controller_model=controller_model,
      dt_control=0.002,
      dt_sim=0.001,
      enable_posture=enable_posture,
    )
     # 以实验名称作为键，保存仿真结果
    results[name] = result

  print("\n========== 模型不匹配比较 ==========\n")
  print(
    f"{'Case':<16}"
    f"{'RMSx(mm)':>12}"
    f"{'RMSy(mm)':>12}"
    f"{'MaxErr(mm)':>13}"
    f"{'FinalErr(mm)':>15}"
    f"{'MaxTau1':>12}"
  )

  for name, result in results.items():
    # 任务空间误差与关节力矩历史数据
    error = result["error"]
    torque = result["tau"]
    # x和y方向的RMS误差，单位mm
    rms_xy = np.sqrt(np.mean(error**2, axis=0)) * 1000
    # 每个时间点的二维位置误差模长
    error_norm = np.linalg.norm(error, axis=1) * 1000
    # 最大位置误差
    max_error = np.max(error_norm)
    # 最终位置误差
    final_error = error_norm[-1]
    # 第一关节最大绝对力矩
    max_tau1 = np.max(np.abs(torque[:, 0]))

    print(
      f"{name:<16}"
      f"{rms_xy[0]:12.5f}"
      f"{rms_xy[1]:12.5f}"
      f"{max_error:13.5f}"
      f"{final_error:15.6f}"
      f"{max_tau1:12.4f}"
    )
  # 只创建一张图
  plt.figure()
  for name, result in results.items():
    # 计算任务空间位置误差的欧氏范数
    error_norm_mm = (np.linalg.norm(result["error"], axis=1) * 1000)
    # 在同一张图上绘制不同实验曲线
    plt.plot(
      result["time"],
      error_norm_mm,
      label=name,
    )

  print("\n========== 最终状态检查 ==========\n")
  for name, result in results.items():
    print(f"\n{name}")
    print("Final q =", result["q"][-1])
    print("Final q_dot =", result["q_dot"][-1])
    print("Final task error (mm) =",
      result["error"][-1] * 1000.0,
    )
  plt.xlabel("Time [s]")
  plt.ylabel("Task Error Norm [mm]")

  print("\n========== 奇异性与速度检查 ==========\n")
  for name, result in results.items():
    sigma_min_values = result["sigma_min"]
    min_index = np.argmin(sigma_min_values)
    min_sigma = sigma_min_values[min_index]
    min_sigma_time = result["time"][min_index]
    max_speed = np.max(np.abs(result["q_dot"]))

    print(f"\n{name}")
    print("Min sigma(J) =", min_sigma)
    print("Time at min sigma =", min_sigma_time)
    print("Max joint speed =", max_speed)

  plt.title("Operational-Space Model Mismatch")
  plt.grid(True)
  plt.legend()
  plt.tight_layout()
  plt.show()


if __name__ == "__main__":
    main()
