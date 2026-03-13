import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

# 设置中文字体
plt.rcParams['font.sans-serif'] = ['SimHei']
plt.rcParams['axes.unicode_minus'] = False

print("="*50)
print("单连杆拉格朗日方程教学实例")
print("="*50)

# 系统参数
m = 1.0    # 质量 (kg)
L = 0.5    # 质心距离 (m)
I = 0.1    # 转动惯量 (kg·m²)
g = 9.8    # 重力加速度 (m/s²)

print(f"\n【系统参数】")
print(f"质量 m = {m} kg")
print(f"质心距离 L = {L} m")
print(f"转动惯量 I = {I} kg·m²")

# ==================== 1. 拉格朗日函数 ====================
print("\n" + "="*50)
print("1. 拉格朗日函数")
print("="*50)

J = m * L**2 + I  # 等效转动惯量
print(f"\n● 动能 T = 1/2 * J * θ̇²")
print(f"  等效转动惯量 J = {J:.3f} kg·m²")
print(f"\n● 势能 P = m*g*L*(1-cosθ)  (以最低点为零点)")
print(f"\n● 拉格朗日函数 L = T - P")

# ==================== 2. 运动方程 ====================
print("\n" + "="*50)
print("2. 运动方程")
print("="*50)

print("\n● 拉格朗日方程：d/dt (∂L/∂θ̇) - ∂L/∂θ = 0")
print("\n● 得到运动方程：")
print("  J * θ̈ + m*g*L*sinθ = 0")
print(f"  即：{J:.3f} * θ̈ + {m*g*L:.3f} * sinθ = 0")

# ==================== 3. 数值求解 ====================
print("\n" + "="*50)
print("3. 数值求解")
print("="*50)

def pendulum_dynamics(t, state):
    """单摆动力学方程"""
    theta, theta_dot = state
    theta_ddot = -(m*g*L/J) * np.sin(theta)
    return [theta_dot, theta_ddot]

# 初始条件
theta0 = np.radians(30)  # 初始角度30°
theta_dot0 = 0.0          # 初始角速度0

# 求解
t_span = (0, 10)
t_eval = np.linspace(0, 10, 500)

sol = solve_ivp(pendulum_dynamics, t_span, [theta0, theta_dot0], 
                t_eval=t_eval)

theta = sol.y[0]
theta_dot = sol.y[1]

print(f"\n初始条件: θ=30°, θ̇=0")
print(f"求解时间: 0-10秒")

# ==================== 4. 能量计算 ====================
print("\n" + "="*50)
print("4. 能量分析")
print("="*50)

# 计算能量
T = 0.5 * J * theta_dot**2
P = m * g * L * (1 - np.cos(theta))
E = T + P

print(f"\n初始总能量: {E[0]:.3f} J")
print(f"最终总能量: {E[-1]:.3f} J")
print(f"能量变化: {E[-1] - E[0]:.3f} J (应接近0，机械能守恒)")

# ==================== 5. 结果可视化 ====================
print("\n" + "="*50)
print("5. 结果可视化")
print("="*50)

plt.figure(figsize=(12, 8))

# 子图1：角度随时间变化
plt.subplot(2, 2, 1)
plt.plot(t_eval, np.degrees(theta), 'b-', linewidth=2)
plt.xlabel('时间 (s)')
plt.ylabel('角度 (度)')
plt.title('角度响应')
plt.grid(True)

# 子图2：角速度随时间变化
plt.subplot(2, 2, 2)
plt.plot(t_eval, theta_dot, 'r-', linewidth=2)
plt.xlabel('时间 (s)')
plt.ylabel('角速度 (rad/s)')
plt.title('角速度响应')
plt.grid(True)

# 子图3：相图
plt.subplot(2, 2, 3)
plt.plot(np.degrees(theta), theta_dot, 'g-', linewidth=1.5)
plt.xlabel('角度 (度)')
plt.ylabel('角速度 (rad/s)')
plt.title('相图')
plt.grid(True)

# 子图4：能量变化
plt.subplot(2, 2, 4)
plt.plot(t_eval, T, 'b-', label='动能', alpha=0.7)
plt.plot(t_eval, P, 'r-', label='势能', alpha=0.7)
plt.plot(t_eval, E, 'g-', label='总能量', linewidth=2)
plt.xlabel('时间 (s)')
plt.ylabel('能量 (J)')
plt.title('能量变化')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()

print("\n" + "="*50)
print("总结：")
print("="*50)
print("• 拉格朗日方程：d/dt(∂L/∂θ̇) - ∂L/∂θ = 0")
print(f"• 运动方程：{J:.3f}θ̈ + {m*g*L:.3f}sinθ = 0")
print("• 系统机械能守恒")
print("="*50)