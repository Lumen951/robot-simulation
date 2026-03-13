import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

# 设置中文字体
plt.rcParams['font.sans-serif'] = ['SimHei']
plt.rcParams['axes.unicode_minus'] = False

# ==================== 参数设置 ====================
L = 1.0          # 杆长 (m)
m = 1.0          # 质量 (kg)
g = 9.8          # 重力加速度 (m/s²)
theta0 = np.pi/6  # 初始角度 30°
omega0 = 0.0     # 初始角速度

# ==================== 运动方程 ====================
def equations(t, y):
    theta, omega = y
    # 角加速度: α = [2ω²cosθsinθ - g sinθ] / [2(1/3 + cos²θ)]
    alpha = (2*omega**2 * np.cos(theta) * np.sin(theta) - g * np.sin(theta)) / \
            (2 * (1/3 + np.cos(theta)**2))
    return [omega, alpha]

# ==================== 数值求解 ====================
t_span = (0, 10)
t_eval = np.linspace(0, 10, 300)
sol = solve_ivp(equations, t_span, [theta0, omega0], t_eval=t_eval)

theta = sol.y[0]
omega = sol.y[1]
t = sol.t

# ==================== 计算位置 ====================
# B点坐标
xB = L * np.sin(theta)
yB = -L * np.cos(theta)

# C点坐标 (根据约束 θ2 = π - 2θ)
xC = 2 * L * np.sin(theta)
yC = np.zeros_like(theta)  # 约束保证 yC = 0

# ==================== 图1：角度和角速度 ====================
fig1, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6))

ax1.plot(t, theta * 180/np.pi, 'b-')
ax1.set_ylabel('θ₁ (度)')
ax1.grid(True)
ax1.set_title('角度随时间变化')

ax2.plot(t, omega * 180/np.pi, 'r-')
ax2.set_xlabel('时间 (秒)')
ax2.set_ylabel('ω₁ (度/秒)')
ax2.grid(True)

plt.tight_layout()
plt.show()

# ==================== 图2：能量 ====================
# 动能 T = (1/3 + cos²θ) ω²
T = (1/3 + np.cos(theta)**2) * omega**2
# 势能 V = -m g cosθ
V = -m * g * np.cos(theta)
E = T + V  # 总机械能

plt.figure(figsize=(8, 4))
plt.plot(t, T, 'b-', label='动能', linewidth=1.5)
plt.plot(t, V, 'g-', label='势能', linewidth=1.5)
plt.plot(t, E, 'r--', label='总机械能', linewidth=1.5)
plt.xlabel('时间 (秒)')
plt.ylabel('能量 (焦耳)')
plt.title('系统能量变化')
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()

# ==================== 图3：杆件运动快照 ====================
plt.figure(figsize=(8, 6))

# 取5个时间点绘制杆件位置快照
n_snapshots = 5
idx = np.linspace(0, len(t)-1, n_snapshots, dtype=int)

for i in idx:
    # 绘制AB杆
    plt.plot([0, xB[i]], [0, yB[i]], 'b-', alpha=0.5, linewidth=2)
    # 绘制BC杆
    plt.plot([xB[i], xC[i]], [yB[i], yC[i]], 'r-', alpha=0.5, linewidth=2)
    # 绘制关节
    plt.plot(xB[i], yB[i], 'bo', markersize=4)

# 标记固定点和末端轨迹
plt.plot(0, 0, 'ks', markersize=8, label='A点(固定)')
plt.plot(xC, yC, 'g--', alpha=0.5, label='C点轨迹')
plt.plot(xC[0], yC[0], 'go', markersize=6, label='C点(起始)')
plt.plot(xC[-1], yC[-1], 'ro', markersize=6, label='C点(结束)')

plt.xlabel('X (米)')
plt.ylabel('Y (米)')
plt.title('杆件运动快照')
plt.grid(True)
plt.legend()
plt.axis('equal')
plt.tight_layout()
plt.show()

# ==================== 输出结果 ====================
print("="*40)
print("平面二连杆系统分析结果")
print("="*40)
print(f"杆长: L = {L} m")
print(f"质量: m = {m} kg")
print(f"初始角度: θ₁(0) = {theta0*180/np.pi:.1f}°")
print(f"运动方程: d²θ/dt² = [2ω²cosθsinθ - {g:.1f}sinθ] / [2(1/3 + cos²θ)]")
print("="*40)