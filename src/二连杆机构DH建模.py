import numpy as np
import matplotlib.pyplot as plt

plt.rcParams['font.sans-serif'] = ['SimHei']  # 用来正常显示中文标签
plt.rcParams['axes.unicode_minus'] = False    # 用来正常显示负号

def dh_matrix(theta, a):
    """简化的DH矩阵 - 只针对平面二连杆"""
    return np.array([
        [np.cos(theta), -np.sin(theta), 0, a*np.cos(theta)],
        [np.sin(theta), np.cos(theta), 0, a*np.sin(theta)],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

def fk_dh(theta1, theta2, a1=1.0, a2=0.8):
    """使用DH矩阵的正运动学"""
    t1, t2 = np.radians(theta1), np.radians(theta2)
    
    # DH变换矩阵
    T01 = dh_matrix(t1, a1)
    T12 = dh_matrix(t2, a2)
    T02 = T01 @ T12
    
    # 提取位置
    j1 = (T01[0,3], T01[1,3])
    j2 = (T02[0,3], T02[1,3])
    
    return (0,0), j1, j2

def plot_robot(theta1, theta2):
    """可视化机械臂"""
    j0, j1, j2 = fk_dh(theta1, theta2)
    
    plt.figure(figsize=(6, 6))
    plt.plot([j0[0], j1[0], j2[0]], [j0[1], j1[1], j2[1]], 'b-o', linewidth=2)
    plt.plot(j0[0], j0[1], 'ks', markersize=10, label='基座')
    plt.plot(j1[0], j1[1], 'ro', markersize=8, label='关节2')
    plt.plot(j2[0], j2[1], 'g*', markersize=12, label='末端')
    
    plt.grid(True)
    plt.axis('equal')
    plt.xlim(-2, 2)
    plt.ylim(-0.5, 2.5)
    plt.xlabel('X'); plt.ylabel('Y')
    plt.title(f'DH法: θ1={theta1}°, θ2={theta2}°')
    plt.legend()
    plt.show()
    
    print(f"末端位置(DH法): ({j2[0]:.3f}, {j2[1]:.3f})")

# 测试
if __name__ == "__main__":
    plot_robot(120, 60)  # 测试一个典型姿态