import numpy as np
# 三连杆长度，单位 m
L = np.array([0.5, 0.4, 0.4])
# MoveIt 规划轨迹第70个点：t = 7.0s
q_d = np.array([ -0.05852992761421058,  0.10281373857901961,0.00004025136153399875,])

dq_d = np.array([ -0.09309125291837035,  0.15047641027506622, -0.20000000000009208,])

ddq_d = np.array([-4.284389777609975e-11, 6.923285294777778e-11, -9.201448512017278e-11,])


def fk(q):
  theta = np.cumsum(q)
  return np.array([ np.sum(L * np.cos(theta)), np.sum(L * np.sin(theta))])

def jacobian(q):
  theta = np.cumsum(q)
  J = np.zeros((2, 3))
  for i in range(3):
    J[0, i] = -np.sum( L[i:] * np.sin(theta[i:]))
    J[1, i] = np.sum( L[i:] * np.cos(theta[i:]))
  return J

# 1. 末端期望位置
x_d = fk(q_d)

# 2. 末端期望速度
J_d = jacobian(q_d)
dx_d = J_d @ dq_d

# 3. Jdot @ dq
theta = np.cumsum(q_d)
omega = np.cumsum(dq_d)

Jdot_dq = np.array([
  -np.sum(L * np.cos(theta) * omega**2),
  -np.sum(L * np.sin(theta) * omega**2),
])

# 4. 末端期望加速度
ddx_d = J_d @ ddq_d + Jdot_dq

# 5. 用数值差分独立验证 Jdot @ dq
eps = 1e-6

J_plus = jacobian(q_d + eps * dq_d)
J_minus = jacobian(q_d - eps * dq_d)
Jdot_dq_numeric = ( (J_plus - J_minus) / (2 * eps)) @ dq_d

print("========== MoveIt -> OSC ==========")
print("x_d =", x_d, "m")
print("dx_d =", dx_d, "m/s")
print("ddx_d =", ddx_d, "m/s^2")
print("Jdot_dq =", Jdot_dq)
print("Jdot_dq_numeric =", Jdot_dq_numeric)
print("Jdot项数值验证误差 =", np.linalg.norm(Jdot_dq - Jdot_dq_numeric),)
