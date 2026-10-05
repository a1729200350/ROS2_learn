import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
FILE = "tracking_data.csv"
def calculate_metrics(error):
  max_error = np.max(np.abs(error),axis=0)
  rms_error = np.sqrt(np.mean(error**2,axis=0))
  return max_error, rms_error

def main():
  data = pd.read_csv(FILE)
  time = data["time"].values

  # 实际关节位置
  q = data[["q1","q2","q3",]].values

  # 期望关节位置
  q_d = data[["qd1","qd2","qd3",]].values

  # 误差
  error = data[["e1","e2","e3"]].values
  # 力矩
  torque = data[["tau1","tau2","tau3"]].values
  # 运动阶段
  motion_mask = (time <= 2.5)
  motion_error = error[motion_mask]
  max_motion, rms_motion = (calculate_metrics(motion_error))
  # 稳态阶段
  steady_mask = (time > 3.0)
  steady_error = error[steady_mask]
  max_steady, rms_steady = (calculate_metrics(steady_error))
  # 最大力矩
  max_tau = np.max(np.abs(torque),axis=0)
  print("\n====================")
  print("轨迹跟踪表现")
  print("====================\n")
  print("运动阶段 (0~2.5s)")
  print("--------------------")
  for i in range(3):
    print(f"joint{i+1}:")
    print(f"  max error = {max_motion[i]:.8f} rad")
    print(f"  RMS error = {rms_motion[i]:.8f} rad")
  print("\n稳定 阶段 (>3s)")
  print("--------------------")

  for i in range(3):
    print(f"joint{i+1}:")
    print(f"  max error = {max_steady[i]:.8f} rad")
    print(f"  RMS error = {rms_steady[i]:.8f} rad")

  print("\n最大力矩")
  print("--------------------")
  for i in range(3):
    print(f"joint{i+1}:")
    print(f"  |tau|max = {max_tau[i]:.6f} N*m")

  # 位置跟踪曲线
  for i in range(3):
    plt.figure()
    # 所有时刻，第 i 个关节的位置。
    plt.plot(time,q[:, i],label=f"joint{i+1} actual")
    plt.plot(time,q_d[:, i],"--",label=f"joint{i+1} desired")
    plt.xlabel("Time [s]")
    plt.ylabel("Joint Position [rad]")
    plt.title(f"Joint {i+1} Position Tracking")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
  # 跟踪误差曲线
  plt.figure()
  for i in range(3):
    plt.plot(time,error[:, i],label=f"joint{i+1}")
    plt.xlabel("Time [s]")
    plt.ylabel("Tracking Error [rad]")
    plt.title("Joint Tracking Error")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
  # 控制力矩曲线
  plt.figure()
  for i in range(3):
    plt.plot( time,torque[:, i],label=f"joint{i+1}")
    plt.xlabel("Time [s]")
    plt.ylabel("Command Torque [N*m]")
    plt.title("Computed Torque Command")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    
  # 显示所有图
  plt.show()


if __name__ == "__main__":
  main()