from pathlib import Path
from datetime import datetime
import argparse
import csv
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import osc_simulation_common as sim

def compare_baseline(filename, results):
  with filename.open("r", encoding="utf-8", newline="") as f:
    previous = {row["mode"]: row for row in csv.DictReader(f)}

  print("\n与指定基准文件比较:", filename)
  for result in results:
    old = previous.get(result["mode"])
    if (
      not result["completed"]
      or old is None
      or not old.get("rms_x_mm")
      or not old.get("rms_y_mm")
    ):
      print(result["mode"], "缺少完整结果，跳过比较")
      continue

    old_rms = np.array([
      float(old["rms_x_mm"]),
      float(old["rms_y_mm"]),
    ])
    print(result["mode"])
    print("基准末端 RMS [mm]:", old_rms)
    print("本次末端 RMS [mm]:", result["rms_x_mm"])
    print("RMS 变化 [mm]:", result["rms_x_mm"] - old_rms)


def plot_command_comparison(output, results):
  fig, axes = plt.subplots(
    len(results), 1,
    figsize=(11, 3 * len(results)),
    sharex=True,
    squeeze=False,
  )
  for ax, result in zip(axes[:, 0], results):
    h = result["history"]
    ax.set_title(result["mode"])
    ax.set_ylabel("Torque (Nm)")
    ax.grid(True)
    if len(h["time"]) == 0:
      continue
    ax.plot(h["time"], h["tau_computed"][:, 0], label="Computed joint1")
    ax.plot(h["time"], h["tau"][:, 0], label="Applied joint1")
    ax.legend()

  axes[-1, 0].set_xlabel("Trajectory time (s)")
  fig.tight_layout()
  fig.savefig(output / "compare_command_vs_applied_joint1.png", dpi=150)
  plt.close(fig)


def main():
  parser = argparse.ArgumentParser(description="MoveIt OSC 离线闭环对照")
  parser.add_argument("reference")
  parser.add_argument("--modes", nargs="+", choices=sim.MODES, default=list(sim.MODES))
  parser.add_argument("--delay-ms", type=int, default=0)
  parser.add_argument("--output", type=Path)
  parser.add_argument("--baseline", type=Path)
  args = parser.parse_args()

  period_ms = sim.DT_CTRL * 1000.0
  cycles = round(args.delay_ms / period_ms)
  if args.delay_ms < 0 or abs(cycles * period_ms - args.delay_ms) > 1e-8:
    parser.error(f"延迟必须是非负的 {period_ms:g} ms 整数倍")
  if len(set(args.modes)) != len(args.modes):
    parser.error("控制模式不能重复")

  source, ref = sim.load_reference(args.reference)
  if args.baseline is not None:
    args.baseline = args.baseline.expanduser().resolve()
    if not args.baseline.is_file():
      parser.error("基准 CSV 文件不存在")

  stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
  output = (
    args.output.expanduser().resolve()
    if args.output is not None
    else source.parent / (
      f"{source.stem}_closed_loop_delay_{args.delay_ms:03d}ms_{stamp}"
    )
  )
  if output.exists():
    parser.error(f"结果目录已存在，请换一个新目录: {output}")
  output.mkdir(parents=True)

  print("参考轨迹:", source)
  print("控制模式:", args.modes)
  print("额外力矩执行延迟 [ms]:", args.delay_ms)
  print("控制/积分周期 [s]:", sim.DT_CTRL, sim.DT_SIM)

  results = []
  for mode in args.modes:
    print(f"\n开始运行: {mode}", flush=True)
    result = sim.run_case(mode, ref, delay_ms=args.delay_ms)
    sim.report_case(result)
    if "max_cmd_age_ms" in result:
      print("力矩指令的滞后 [ms]:", result["max_cmd_age_ms"])
    np.savez_compressed(output / f"{mode}.npz", **result["history"])
    results.append(result)

  sim.write_comparison(output, results)
  plot_command_comparison(output, results)
  if args.baseline is not None:
    compare_baseline(args.baseline, results)

  print("\n结果目录:", output)
  print("汇总 CSV:", output / "summary.csv")


if __name__ == "__main__":
  main()