import argparse
import numpy as np
from osc_direction_common import run_direction_experiment


def main():
  parser = argparse.ArgumentParser(description="奇异方向与任务降级对照")
  parser.add_argument("reference")
  parser.add_argument("gazebo")
  parser.add_argument("--scale", type=float, nargs="+", default=[1.0, 3.0])
  args = parser.parse_args()
  if any(not np.isfinite(s) or s <= 0 for s in args.scale):
    parser.error("scale必须是有限正数")

  for scale in args.scale:
    b_test = np.array([0.3, -0.15]) * scale
    print(f"\n========== 加速度倍率 {scale:g} ==========")
    run_direction_experiment(b_test, args.reference, args.gazebo)


if __name__ == "__main__":
  main()