import numpy as np
import matplotlib.pyplot as plt

# 设置中文字体
plt.rcParams['font.sans-serif'] = ['SimHei']
plt.rcParams['axes.unicode_minus'] = False

class SimplePuma560:
    """极简版Puma560机器人正运动学教学程序"""
    
    def __init__(self):
        # Puma560标准参数 (米)
        self.d1 = 0.5     # 基座高度
        self.a2 = 0.432   # 大臂长度
        self.a3 = 0.432   # 小臂长度
        self.d4 = 0.15    # 腕部偏置
        self.d6 = 0.1     # 末端长度
        
    def dh_transform(self, theta, d, a, alpha):
        """计算DH变换矩阵"""
        return np.array([
            [np.cos(theta), -np.sin(theta)*np.cos(alpha),  
             np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
            [np.sin(theta), np.cos(theta)*np.cos(alpha), 
             -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1]
        ])
    
    def forward_kinematics(self, j1, j2, j3, j4, j5, j6):
        """正运动学计算（角度：度）"""
        theta = np.deg2rad([j1, j2, j3, j4, j5, j6])
        
        # DH参数表 [theta, d, a, alpha]
        dh = [
            [theta[0], self.d1, 0, -np.pi/2],           # J1 (有基座高度)
            [theta[1] - np.pi/2, 0, self.a2, 0],        # J2 
            [theta[2], 0, 0, -np.pi/2],                  # J3
            [np.pi/2, self.d4, 0, np.pi/2],              # 结构变换
            [theta[3], 0, 0, -np.pi/2],                  # J4
            [theta[4], 0, 0, np.pi/2],                   # J5
            [theta[5], self.d6, 0, 0]                     # J6+末端
        ]
        
        # 计算所有关节位置（包括基座）
        positions = []
        
        # 基座位置 (0,0,0)
        positions.append(np.array([0, 0, 0]))
        
        # 计算各关节位置
        T = np.eye(4)
        for params in dh:
            T = T @ self.dh_transform(*params)
            positions.append(T[:3, 3].copy())
            
        return np.array(positions), T  # 返回关节位置和末端变换矩阵
    
    def visualize(self, j1=0, j2=0, j3=0, j4=0, j5=0, j6=0):
        """可视化机器人和计算结果"""
        # 计算正运动学
        pos, T_end = self.forward_kinematics(j1, j2, j3, j4, j5, j6)
        
        # 打印结果
        print(f"\n关节角度: {j1:4.0f} {j2:4.0f} {j3:4.0f} {j4:4.0f} {j5:4.0f} {j6:4.0f} 度")
        print("末端位姿矩阵:")
        print(np.array2string(T_end, precision=3, suppress_small=True))
        print(f"末端位置: x={T_end[0,3]:.3f}, y={T_end[1,3]:.3f}, z={T_end[2,3]:.3f} 米")
        
        # 绘图
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        # 绘制机器人和关节
        ax.plot(pos[:,0], pos[:,1], pos[:,2], 'b-o', linewidth=3, markersize=8)
        
        # 标记关节
        labels = ['基座(J0)', 'J1', 'J2', 'J3', 'J4', 'J5', 'J6', '末端']
        colors = ['darkred', 'red', 'orange', 'gold', 'green', 'blue', 'purple', 'magenta']
        
        for i, (x,y,z) in enumerate(pos):
            ax.scatter(x, y, z, c=colors[i], s=100 if i<7 else 200, 
                      marker='s' if i==0 else 'o' if i<7 else '*')
            ax.text(x, y+0.05, z+0.05, labels[i], ha='center', fontsize=9,
                   bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
        
        # 设置坐标轴
        ax.set_xlabel('X (米)')
        ax.set_ylabel('Y (米)')
        ax.set_zlabel('Z (米)')
        ax.set_title(f'Puma560机器人 (角度: {j1} {j2} {j3} {j4} {j5} {j6})')
        ax.set_xlim([-1, 1])
        ax.set_ylim([-1, 1])
        ax.set_zlim([-0.2, 1.5])
        ax.grid(True)
        
        # 添加地面
        X, Y = np.meshgrid([-1,1], [-1,1])
        ax.plot_surface(X, Y, np.zeros((2,2)), alpha=0.3, color='lightgray')
        
        # 打印各关节坐标
        print("\n关节坐标:")
        for i, label in enumerate(labels):
            print(f"{label}: ({pos[i,0]:.3f}, {pos[i,1]:.3f}, {pos[i,2]:.3f})")
        
        plt.show()

def main():
    """主函数"""
    print("="*50)
    print("Puma560 机器人正运动学教学程序")
    print("="*50)
    print("说明: 基座(J0)在(0,0,0)位置")
    
    robot = SimplePuma560()
    
    while True:
        print("\n请选择操作:")
        print("1. 演示初始位形 (0,0,0,0,0,0)")
        print("2. 演示伸展位形 (0,45,45,0,0,0)")
        print("3. 演示弯曲位形 (30,45,30,45,60,90)")
        print("4. 输入自定义角度")
        print("5. 退出")
        
        choice = input("请输入选择 (1-5): ").strip()
        
        if choice == '1':
            robot.visualize(0, 0, 0, 0, 0, 0)
        elif choice == '2':
            robot.visualize(0, 45, 45, 0, 0, 0)
        elif choice == '3':
            robot.visualize(30, 45, 30, 45, 60, 90)
        elif choice == '4':
            try:
                angles = input("输入6个关节角度 (用空格分隔): ").split()
                if len(angles) == 6:
                    j = [float(a) for a in angles]
                    robot.visualize(*j)
                else:
                    print("需要输入6个角度!")
            except:
                print("输入错误!")
        elif choice == '5':
            print("再见!")
            break
        else:
            print("无效选择!")

if __name__ == "__main__":
    main()