import numpy as np
import matplotlib.pyplot as plt

# 设置中文字体
plt.rcParams['font.sans-serif'] = ['SimHei']
plt.rcParams['axes.unicode_minus'] = False

class JacobianDemo:
    """雅可比矩阵简单演示"""
    
    def __init__(self):
        # 简化的2连杆机器人
        self.l1 = 0.5  # 连杆1长度
        self.l2 = 0.4  # 连杆2长度
        
    def forward_kinematics(self, theta1, theta2):
        """正运动学"""
        theta = np.deg2rad([theta1, theta2])
        
        x = self.l1 * np.cos(theta[0]) + self.l2 * np.cos(theta[0] + theta[1])
        y = self.l1 * np.sin(theta[0]) + self.l2 * np.sin(theta[0] + theta[1])
        
        return np.array([x, y])
    
    def jacobian(self, theta1, theta2):
        """计算2x2雅可比矩阵"""
        theta = np.deg2rad([theta1, theta2])
        
        J = np.zeros((2, 2))
        
        # 对theta1求导
        J[0, 0] = -self.l1 * np.sin(theta[0]) - self.l2 * np.sin(theta[0] + theta[1])
        J[1, 0] = self.l1 * np.cos(theta[0]) + self.l2 * np.cos(theta[0] + theta[1])
        
        # 对theta2求导
        J[0, 1] = -self.l2 * np.sin(theta[0] + theta[1])
        J[1, 1] = self.l2 * np.cos(theta[0] + theta[1])
        
        return J
    
    def demonstrate(self, theta1=30, theta2=45):
        """演示雅可比矩阵"""
        
        # 计算末端位置和雅可比矩阵
        pos = self.forward_kinematics(theta1, theta2)
        J = self.jacobian(theta1, theta2)
        
        print("="*50)
        print("雅可比矩阵演示 (2连杆机器人)")
        print("="*50)
        print(f"关节角度: θ1={theta1}°, θ2={theta2}°")
        print(f"末端位置: ({pos[0]:.3f}, {pos[1]:.3f})")
        
        print("\n【雅可比矩阵 J】")
        print("将关节速度映射到末端速度:")
        print("[vx] = [ J11  J12 ] [dθ1]")
        print("[vy]   [ J21  J22 ] [dθ2]")
        print("\n数值结果:")
        print(f"J = [{J[0,0]:.3f}  {J[0,1]:.3f}]")
        print(f"    [{J[1,0]:.3f}  {J[1,1]:.3f}]")
        
        print("\n【物理意义】")
        print("第1列: 关节1转动对末端速度的影响")
        print(f"  dθ1=1 → vx={J[0,0]:.3f}, vy={J[1,0]:.3f}")
        print("第2列: 关节2转动对末端速度的影响") 
        print(f"  dθ2=1 → vx={J[0,1]:.3f}, vy={J[1,1]:.3f}")
        
        # 可操作度
        manipulability = np.abs(np.linalg.det(J))
        print(f"\n可操作度: {manipulability:.4f}")
        print("(值越大,运动越灵活; 0表示奇异位形)")
        
        # 可视化
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
        
        # 左图: 机器人
        theta = np.deg2rad([theta1, theta2])
        p0 = np.array([0, 0])
        p1 = np.array([self.l1 * np.cos(theta[0]), self.l1 * np.sin(theta[0])])
        p2 = pos
        
        ax1.plot([p0[0], p1[0]], [p0[1], p1[1]], 'b-o', linewidth=3, markersize=8, label='连杆1')
        ax1.plot([p1[0], p2[0]], [p1[1], p2[1]], 'r-o', linewidth=3, markersize=8, label='连杆2')
        ax1.scatter(*p2, c='g', s=200, marker='*', label='末端')
        ax1.set_xlim([-1, 1])
        ax1.set_ylim([-0.2, 1])
        ax1.set_xlabel('X')
        ax1.set_ylabel('Y')
        ax1.set_title(f'机器人位形 (θ1={theta1}°, θ2={theta2}°)')
        ax1.grid(True)
        ax1.legend()
        ax1.set_aspect('equal')
        
        # 右图: 速度映射
        ax2.arrow(0, 0, J[0,0], J[1,0], head_width=0.05, head_length=0.05, 
                 fc='blue', ec='blue', label='关节1速度影响')
        ax2.arrow(0, 0, J[0,1], J[1,1], head_width=0.05, head_length=0.05, 
                 fc='red', ec='red', label='关节2速度影响')
        
        # 合成速度示例
        dq = np.array([0.1, 0.1])  # 两个关节都转0.1 rad/s
        v = J @ dq
        ax2.arrow(0, 0, v[0], v[1], head_width=0.05, head_length=0.05,
                 fc='green', ec='green', label='合成速度(dθ1=dθ2=0.1)')
        
        ax2.set_xlim([-0.5, 0.5])
        ax2.set_ylim([-0.5, 0.5])
        ax2.set_xlabel('vx')
        ax2.set_ylabel('vy')
        ax2.set_title('关节速度引起的末端速度')
        ax2.grid(True)
        ax2.legend()
        ax2.set_aspect('equal')
        
        plt.tight_layout()
        plt.show()
        
        return J

def main():
    print("雅可比矩阵简单演示程序")
    print("-" * 40)
    
    demo = JacobianDemo()
    
    while True:
        print("\n1. 使用默认角度 (30°, 45°)")
        print("2. 输入自定义角度")
        print("3. 退出")
        
        choice = input("请选择: ")
        
        if choice == '1':
            demo.demonstrate(30, 45)
        elif choice == '2':
            try:
                a1 = float(input("输入关节1角度 (度): "))
                a2 = float(input("输入关节2角度 (度): "))
                demo.demonstrate(a1, a2)
            except:
                print("输入错误!")
        elif choice == '3':
            break

if __name__ == "__main__":
    main()