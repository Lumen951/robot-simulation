import numpy as np
import matplotlib.pyplot as plt

# 设置中文字体
plt.rcParams['font.sans-serif'] = ['SimHei']
plt.rcParams['axes.unicode_minus'] = False

class NewtonEulerDemo:
    """牛顿-欧拉法教学演示（纯牛顿-欧拉法）"""
    
    def __init__(self):
        self.L = 1.0      # 连杆长度
        self.m = 1.0      # 连杆质量
        self.g = 9.81     # 重力加速度
        
    def newton_euler_static(self, θ1, θ2):
        """静态牛顿-欧拉法"""
        # 关节和质心位置
        j1 = np.array([self.L * np.cos(θ1), self.L * np.sin(θ1)])
        c1 = np.array([(self.L/2) * np.cos(θ1), (self.L/2) * np.sin(θ1)])
        c2 = j1 + np.array([(self.L/2) * np.cos(θ1 + θ2), 
                           (self.L/2) * np.sin(θ1 + θ2)])
        
        # 重力矢量（垂直向下）
        G = np.array([0, -self.m * self.g])
        
        # 向内递推：从末端开始
        f3 = np.array([0, 0])  # 末端不受力
        
        # 连杆2的力平衡
        f2 = f3 + G  # 连杆2受到重力
        
        # 连杆2对关节2的力矩
        r2 = c2 - j1  # 力臂：从关节2到连杆2质心
        τ2 = np.cross(r2, G)  # 力矩 = 力臂 × 力
        
        # 连杆1的力平衡
        f1 = f2 + G  # 连杆1受到重力 + 连杆2传递的力
        
        # 连杆1对关节1的力矩（两部分组成）
        r1 = c1  # 力臂：从关节1到连杆1质心
        τ1_from_link1 = np.cross(r1, G)  # 连杆1自身重力产生的力矩
        
        # 通过关节2传递的力矩（包括连杆2的力矩和力）
        τ1_from_link2 = τ2 + np.cross(j1, f2)
        
        τ1 = τ1_from_link1 + τ1_from_link2
        
        # 统一方向约定：逆时针为正
        return -float(τ1), -float(τ2)
    
    def newton_euler_dynamic(self, θ1, θ2, ω1, ω2, α1, α2):
        """动态牛顿-欧拉法"""
        # 关节和质心位置
        j1 = np.array([self.L * np.cos(θ1), self.L * np.sin(θ1)])
        c1 = np.array([(self.L/2) * np.cos(θ1), (self.L/2) * np.sin(θ1)])
        c2 = j1 + np.array([(self.L/2) * np.cos(θ1 + θ2), 
                           (self.L/2) * np.sin(θ1 + θ2)])
        
        # 重力矢量
        G = np.array([0, -self.m * self.g])
        
        # ===== 向外递推：计算加速度 =====
        # 连杆1质心的加速度
        # 向心加速度: -ω²r
        a1_centripetal = -ω1**2 * c1
        # 切向加速度: α × r
        a1_tangential = α1 * np.array([-c1[1], c1[0]]) / (self.L/2)
        # 重力加速度
        a1_gravity = np.array([0, -self.g])
        # 总加速度
        a1 = a1_centripetal + a1_tangential + a1_gravity
        
        # 连杆2质心的加速度（相对复杂）
        # 关节2的加速度
        a_joint2 = -ω1**2 * j1 + α1 * np.array([-j1[1], j1[0]]) / self.L
        
        # 连杆2相对关节2的加速度
        r2_local = c2 - j1
        a2_centripetal = -(ω1+ω2)**2 * r2_local
        a2_tangential = (α1+α2) * np.array([-r2_local[1], r2_local[0]]) / (self.L/2)
        
        a2 = a_joint2 + a2_centripetal + a2_tangential + a1_gravity
        
        # ===== 惯性力 =====
        F1_inertia = -self.m * a1
        F2_inertia = -self.m * a2
        
        # ===== 向内递推：计算力矩 =====
        f3 = np.array([0, 0])  # 末端无力
        
        # 连杆2
        f2 = f3 - F2_inertia
        τ2 = np.cross(c2 - j1, -F2_inertia)
        
        # 连杆1
        f1 = f2 - F1_inertia
        τ1_from_link1 = np.cross(c1, -F1_inertia)
        τ1_from_link2 = τ2 + np.cross(j1, f2)
        τ1 = τ1_from_link1 + τ1_from_link2
        
        return -float(τ1), -float(τ2)
    
    def demonstrate(self):
        """演示2静态 + 2动态工况"""
        
        print("="*60)
        print("牛顿-欧拉法教学演示")
        print("="*60)
        
        # 测试工况
        cases = [
            # [θ1, θ2, ω1, ω2, α1, α2, 标题]
            [-90, 0, 0, 0, 0, 0, "静态1: 自由下垂"],
            [0, 0, 0, 0, 0, 0, "静态2: 水平伸展"],
            [30, 45, 1.0, 0.5, 0, 0, "动态1: 匀速运动"],
            [30, 45, 1.0, 0.5, 0.2, 0.1, "动态2: 加速运动"]
        ]
        
        results = []
        
        for θ1_deg, θ2_deg, ω1, ω2, α1, α2, title in cases:
            θ1 = θ1_deg * np.pi/180
            θ2 = θ2_deg * np.pi/180
            
            print(f"\n{title} (θ1={θ1_deg}°, θ2={θ2_deg}°)")
            print("-" * 40)
            
            if title.startswith("静态"):
                τ1, τ2 = self.newton_euler_static(θ1, θ2)
                print(f"关节力矩: τ1={τ1:7.2f} Nm, τ2={τ2:7.2f} Nm")
            else:
                τ1, τ2 = self.newton_euler_dynamic(θ1, θ2, ω1, ω2, α1, α2)
                print(f"关节力矩: τ1={τ1:7.2f} Nm, τ2={τ2:7.2f} Nm")
            
            results.append((θ1, θ2, τ1, τ2, title))
        
        # 绘制结果
        self.plot_results(results)
    
    def plot_results(self, results):
        """绘制所有工况的机械臂姿态"""
        fig, axes = plt.subplots(2, 2, figsize=(12, 10))
        axes = axes.flatten()
        
        for i, (θ1, θ2, τ1, τ2, title) in enumerate(results):
            ax = axes[i]
            
            # 计算位置
            x1 = self.L * np.cos(θ1)
            y1 = self.L * np.sin(θ1)
            x2 = x1 + self.L * np.cos(θ1 + θ2)
            y2 = y1 + self.L * np.sin(θ1 + θ2)
            
            # 绘制连杆
            ax.plot([0, x1], [0, y1], 'b-', linewidth=3, label='连杆1')
            ax.plot([x1, x2], [y1, y2], 'r-', linewidth=3, label='连杆2')
            
            # 绘制关节和末端
            ax.plot(0, 0, 'ko', markersize=8, label='关节1')
            ax.plot(x1, y1, 'ko', markersize=8, label='关节2')
            ax.plot(x2, y2, 'r*', markersize=12, label='末端')
            
            # 显示力矩
            info = f'τ₁={τ1:.2f} Nm\nτ₂={τ2:.2f} Nm'
            ax.text(1.5, 1.8, info, bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
            
            ax.set_xlim(-2, 2.5)
            ax.set_ylim(-2, 2.5)
            ax.set_aspect('equal')
            ax.grid(True, alpha=0.3)
            ax.set_title(title)
            ax.legend(loc='lower right')
        
        plt.tight_layout()
        plt.show()


def main():
    demo = NewtonEulerDemo()
    demo.demonstrate()
    
    print("\n" + "="*60)
    print("牛顿-欧拉法核心思想")
    print("="*60)
    print("""
    1️⃣ 向外递推：计算各连杆的加速度
       - 向心加速度: -ω²r
       - 切向加速度: α × r
       - 重力加速度: g
       
    2️⃣ 惯性力: F = -m·a
    
    3️⃣ 向内递推：计算各关节的力和力矩
       - 从末端开始，逐连杆向内
       - 力矩 = 力臂 × 力
    """)


if __name__ == "__main__":
    main()