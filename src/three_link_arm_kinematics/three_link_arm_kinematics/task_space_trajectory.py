"""二维任务空间的五次时间缩放轨迹。"""
import numpy as np
def sample_task_trajectory(t, duration, start, goal):
  start = np.asarray(start, dtype=float)
  goal = np.asarray(goal, dtype=float)
  if duration <= 0:
    raise ValueError("duration 必须大于零")
  if t >= duration:
    return goal.copy(), np.zeros(2), np.zeros(2)
  r = max(0.0, t) / duration
  delta = goal - start
  s = 10*r**3 - 15*r**4 + 6*r**5
  s_dot = (30*r**2 - 60*r**3 + 30*r**4) / duration
  s_ddot = (60*r - 180*r**2 + 120*r**3) / duration**2
  return (start + s * delta, s_dot * delta, s_ddot * delta,)
