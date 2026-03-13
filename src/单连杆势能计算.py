import numpy as np
import matplotlib.pyplot as plt

# 设置中文字体
plt.rcParams['font.sans-serif'] = ['SimHei']
plt.rcParams['axes.unicode_minus'] = False

# 固定系统参数
m = 2.0    # 质量 (kg)
L = 0.8    # 质心距离转轴的距离 (m)
g = 9.8    # 重力加速度 (m/s²)

print("="*50)
print("单连杆系统势能计算")
print("="*50)
print(f"系统参数：质量 m = {m} kg")
print(f"         质心距离 L = {L} m")
print(f"         重力加速度 g = {g} m/s²")
print("="*50)

def calculate_potential_energy(theta):
    """
    计算单连杆系统的重力势能
    
    参数:
    theta: 角度 (rad)
    
    返回:
    P: 势能 (J)
    """
    # 质心高度（假设转轴在原点）
    height = L * np.sin(theta)
    
    # 重力势能 P = m * g * h
    P = m * g * height
    
    return P

# 示例：匀速旋转过程中的势能变化
print("\n【匀速旋转过程中的势能变化】")
print("-" * 50)

t = np.linspace(0, 4, 200)  # 时间0-4秒
omega = 1.5  # 恒定角速度 rad/s
theta = omega * t  # 角度随时间线性增加

# 计算每个时刻的势能
P_values = []
for angle in theta:
    P = calculate_potential_energy(angle)
    P_values.append(P)

# 输出几个时间点的数据
print("时间(s) | 角度(rad) | 角度(°)  | 势能(J)")
print("-" * 50)
for i in [0, 50, 100, 150]:
    angle_deg = np.degrees(theta[i])
    print(f"{t[i]:6.2f}   | {theta[i]:8.3f}   | {angle_deg:7.1f}   | {P_values[i]:8.3f}")

# 绘制势能变化图
plt.figure(figsize=(12, 4))

# 子图1：势能随时间变化
plt.subplot(1, 2, 1)
plt.plot(t, P_values, 'b-', linewidth=2, label='势能')
plt.plot(t, theta, 'r--', alpha=0.5, label='角度')
plt.xlabel('时间 (s)')
plt.ylabel('势能 (J) / 角度 (rad)')
plt.title('匀速旋转过程中的势能变化')
plt.legend()
plt.grid(True)

# 子图2：势能与角度的关系
plt.subplot(1, 2, 2)
plt.plot(np.degrees(theta), P_values, 'g-', linewidth=2)
plt.xlabel('角度 (度)')
plt.ylabel('势能 (J)')
plt.title('势能与角度的关系')
plt.grid(True)

plt.tight_layout()
plt.show()

# 势能分析
print("\n" + "="*50)
print("【势能分析】")
print("="*50)

# 计算一个完整周期内的势能最大值和最小值
angles = np.linspace(-np.pi, np.pi, 100)
P_all = [calculate_potential_energy(a) for a in angles]

max_P = max(P_all)
min_P = min(P_all)
max_angle = angles[np.argmax(P_all)]
min_angle = angles[np.argmin(P_all)]

print(f"最大势能: {max_P:.3f} J (在 {np.degrees(max_angle):.1f}°)")
print(f"最小势能: {min_P:.3f} J (在 {np.degrees(min_angle):.1f}°)")
print(f"势能变化范围: {max_P - min_P:.3f} J")
print("="*50)