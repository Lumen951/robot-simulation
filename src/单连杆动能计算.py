import numpy as np
import matplotlib.pyplot as plt

# 设置中文显示
plt.rcParams['font.sans-serif'] = ['SimHei']
plt.rcParams['axes.unicode_minus'] = False

# 固定系统参数
m = 2.0    # 质量 (kg)
L = 0.8    # 质心距离 (m)
I = 0.5    # 转动惯量 (kg·m²)

print("="*60)
print("单连杆系统动能计算（固定参数）")
print("="*60)
print(f"系统参数：质量 m = {m} kg")
print(f"         质心距离 L = {L} m")
print(f"         转动惯量 I = {I} kg·m²")
print(f"         等效转动惯量 J = m*L² + I = {m*L**2 + I:.3f} kg·m²")
print("="*60)

def calculate_kinetic_energy(theta_dot):
    """
    计算单连杆系统的动能
    
    参数:
    theta_dot: 角速度 (rad/s)
    
    返回:
    T_total: 总动能 (J)
    T_trans: 平动动能 (J)
    T_rot: 转动动能 (J)
    """
    T_trans = 0.5 * m * (L * theta_dot)**2  # 平动动能
    T_rot = 0.5 * I * theta_dot**2          # 转动动能
    T_total = T_trans + T_rot                # 总动能
    
    return T_total, T_trans, T_rot

# 示例1：不同角速度下的动能
print("\n【示例1】不同角速度下的动能")
print("-" * 60)
print("注意：动能与角度无关，只与角速度有关")
print("-" * 60)

omega_values = [0.5, 1.0, 2.0, 3.0, 4.0, 5.0]  # 角速度 (rad/s)

print("角速度(rad/s) | 平动动能(J) | 转动动能(J) | 总动能(J)")
print("-" * 60)

for omega in omega_values:
    T_total, T_trans, T_rot = calculate_kinetic_energy(omega)
    print(f"{omega:10.1f}    | {T_trans:10.3f}   | {T_rot:9.3f}   | {T_total:9.3f}")

# 示例2：匀速转动（角速度恒定）
print("\n【示例2】匀速转动 - 角速度恒定")
print("-" * 60)

t1 = np.linspace(0, 5, 100)  # 时间 0-5秒
omega_const = 2.0  # 恒定角速度 rad/s
theta1 = omega_const * t1  # 角度随时间线性增加

T_total1, T_trans1, T_rot1 = calculate_kinetic_energy(omega_const)

print(f"角速度 = {omega_const} rad/s (恒定)")
print(f"角度范围: 0 到 {theta1[-1]:.1f} rad ({np.degrees(theta1[-1]):.1f}°)")
print(f"平动动能 = {T_trans1:.3f} J")
print(f"转动动能 = {T_rot1:.3f} J")
print(f"总动能 = {T_total1:.3f} J (保持不变)")

# 示例3：简谐振动（角度和角速度都变化）
print("\n【示例3】简谐振动 - 角度和角速度都变化")
print("-" * 60)

t2 = np.linspace(0, 6, 200)
omega0 = 2.0  # 振动频率
A = np.pi/4   # 振幅 45度

theta2 = A * np.sin(omega0 * t2)  # 角度变化
theta_dot2 = A * omega0 * np.cos(omega0 * t2)  # 角速度变化

# 计算每个时刻的动能
T_total2 = []
T_trans2 = []
T_rot2 = []

for omega in theta_dot2:
    T_total, T_trans, T_rot = calculate_kinetic_energy(omega)
    T_total2.append(T_total)
    T_trans2.append(T_trans)
    T_rot2.append(T_rot)

# 输出几个时间点的数据
print("时间(s) | 角度(rad) | 角度(°)  | 角速度(rad/s) | 总动能(J)")
print("-" * 70)
for i in [0, 50, 100, 150]:
    angle_deg = np.degrees(theta2[i])
    print(f"{t2[i]:6.2f}   | {theta2[i]:8.3f}   | {angle_deg:7.1f}   | {theta_dot2[i]:12.3f}   | {T_total2[i]:8.3f}")

# 示例4：匀加速转动
print("\n【示例4】匀加速转动")
print("-" * 60)

t3 = np.linspace(0, 4, 100)
alpha = 1.0  # 角加速度 rad/s²
theta3 = 0.5 * alpha * t3**2  # 角度
theta_dot3 = alpha * t3  # 角速度

T_total3 = []
for omega in theta_dot3:
    T_total, _, _ = calculate_kinetic_energy(omega)
    T_total3.append(T_total)

print(f"角加速度 = {alpha} rad/s²")
print("时间(s) | 角度(rad) | 角度(°)  | 角速度(rad/s) | 总动能(J)")
print("-" * 70)

# 修正：使用正确的索引计算
time_points = [1, 2, 3, 4]
for t_val in time_points:
    # 找到最接近的时间点索引
    idx = np.argmin(np.abs(t3 - t_val))
    angle_deg = np.degrees(theta3[idx])
    print(f"{t3[idx]:6.2f}   | {theta3[idx]:8.3f}   | {angle_deg:7.1f}   | {theta_dot3[idx]:12.3f}   | {T_total3[idx]:8.3f}")

# 绘制动能变化图
plt.figure(figsize=(15, 5))

# 子图1：匀速转动
plt.subplot(1, 3, 1)
plt.plot(t1, [T_total1]*len(t1), 'b-', linewidth=2, label='总动能')
plt.plot(t1, theta1, 'g--', alpha=0.5, label='角度')
plt.xlabel('时间 (s)')
plt.ylabel('动能 (J) / 角度 (rad)')
plt.title('匀速转动 (ω=2 rad/s)')
plt.legend()
plt.grid(True)

# 子图2：简谐振动
plt.subplot(1, 3, 2)
plt.plot(t2, T_total2, 'r-', linewidth=2, label='总动能')
plt.plot(t2, theta2, 'g--', alpha=0.5, label='角度')
plt.xlabel('时间 (s)')
plt.ylabel('动能 (J) / 角度 (rad)')
plt.title('简谐振动')
plt.legend()
plt.grid(True)

# 子图3：匀加速转动
plt.subplot(1, 3, 3)
plt.plot(t3, T_total3, 'purple', linewidth=2, label='总动能')
plt.plot(t3, theta3, 'g--', alpha=0.5, label='角度')
plt.xlabel('时间 (s)')
plt.ylabel('动能 (J) / 角度 (rad)')
plt.title('匀加速转动 (α=1 rad/s²)')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()

# 添加角度和动能关系的验证
print("\n" + "="*60)
print("【验证】动能与角度的关系")
print("="*60)

# 固定角速度，改变角度
fixed_omega = 2.0
test_angles = [0, 30, 60, 90, 120, 150, 180]  # 度

T_fixed, _, _ = calculate_kinetic_energy(fixed_omega)
print(f"固定角速度 ω = {fixed_omega} rad/s")
print("角度(°) | 动能(J)")
print("-" * 30)
for angle in test_angles:
    print(f"{angle:6d}    | {T_fixed:8.3f}")

print("\n结论：动能只与角速度有关，与角度无关！")

# 总结
print("\n" + "="*60)
print("总结：")
print("="*60)
print("• 单连杆系统的动能只与角速度有关，与角度无关")
print("• 动能公式：T = 1/2 * (m*L² + I) * θ̇²")
print(f"• 本系统中 (m*L² + I) = {m*L**2 + I:.3f} kg·m²")
print("• 动能与角速度的平方成正比")
print("="*60)