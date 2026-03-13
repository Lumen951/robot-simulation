import numpy as np
import matplotlib.pyplot as plt

# 设置中文字体
plt.rcParams['font.sans-serif'] = ['SimHei']
plt.rcParams['axes.unicode_minus'] = False

class Puma560IK:
    def __init__(self):
        # 机器人参数
        self.d1 = 0.5     # 基座高度
        self.a2 = 0.432   # 大臂长度
        self.a3 = 0.432   # 小臂长度
        
    def forward_kinematics(self, j1, j2, j3):
        """正运动学计算（用于验证）"""
        theta = np.deg2rad([j1, j2, j3])
        
        # 简化的正运动学
        x = np.cos(theta[0]) * (self.a2 * np.cos(theta[1]) + self.a3 * np.cos(theta[1] + theta[2]))
        y = np.sin(theta[0]) * (self.a2 * np.cos(theta[1]) + self.a3 * np.cos(theta[1] + theta[2]))
        z = self.d1 + self.a2 * np.sin(theta[1]) + self.a3 * np.sin(theta[1] + theta[2])
        
        return np.array([x, y, z])
    
    def inverse_kinematics(self, target_x, target_y, target_z):
        """
        几何法求解逆运动学
        返回：两种可能的解 [j1, j2, j3]
        """
        # 计算J1
        j1 = np.arctan2(target_y, target_x)
        
        # 计算腕部位置
        r = np.sqrt(target_x**2 + target_y**2)
        z = target_z - self.d1
        
        # 余弦定理求J3
        cos_j3 = (r**2 + z**2 - self.a2**2 - self.a3**2) / (2 * self.a2 * self.a3)
        
        # 检查可达性
        if abs(cos_j3) > 1:
            return None
        
        # 两种解
        j3_1 = np.arccos(cos_j3)
        j3_2 = -j3_1
        
        # 计算J2
        phi = np.arctan2(z, r)
        psi_1 = np.arctan2(self.a3 * np.sin(j3_1), self.a2 + self.a3 * np.cos(j3_1))
        psi_2 = np.arctan2(self.a3 * np.sin(j3_2), self.a2 + self.a3 * np.cos(j3_2))
        
        j2_1 = phi - psi_1
        j2_2 = phi - psi_2
        
        # 转换为角度并返回
        solutions = [
            [np.rad2deg(j1), np.rad2deg(j2_1), np.rad2deg(j3_1)],
            [np.rad2deg(j1), np.rad2deg(j2_2), np.rad2deg(j3_2)]
        ]
        
        return solutions
    
    def visualize(self, target, solutions):
        """可视化结果"""
        fig = plt.figure(figsize=(12, 5))
        
        for i, q in enumerate(solutions):
            ax = fig.add_subplot(1, 2, i+1, projection='3d')
            
            # 计算机器人各关节位置
            theta = np.deg2rad(q)
            
            # 计算关键点
            p0 = np.array([0, 0, 0])  # 基座
            p1 = np.array([0, 0, self.d1])  # J1
            
            # 计算J2
            p2_x = self.a2 * np.cos(theta[0]) * np.cos(theta[1])
            p2_y = self.a2 * np.sin(theta[0]) * np.cos(theta[1])
            p2_z = self.d1 + self.a2 * np.sin(theta[1])
            p2 = np.array([p2_x, p2_y, p2_z])
            
            # 计算末端
            pe = self.forward_kinematics(*q)
            
            # 绘制机器人
            points = np.array([p0, p1, p2, pe])
            ax.plot(points[:,0], points[:,1], points[:,2], 'b-o', linewidth=3, markersize=8)
            
            # 标记目标点
            ax.scatter(*target, c='red', s=150, marker='*', label='目标点')
            
            # 添加标签
            labels = ['基座', 'J1', 'J2', '末端']
            for j, (x,y,z) in enumerate(points):
                ax.text(x, y+0.05, z+0.05, labels[j], ha='center')
            
            ax.set_xlabel('X (米)')
            ax.set_ylabel('Y (米)')
            ax.set_zlabel('Z (米)')
            ax.set_title(f'解{i+1}: J1={q[0]:.1f}°, J2={q[1]:.1f}°, J3={q[2]:.1f}°')
            ax.set_xlim([-1, 1])
            ax.set_ylim([-1, 1])
            ax.set_zlim([0, 1.5])
            ax.legend()
            ax.grid(True)
            
            # 添加地面
            X, Y = np.meshgrid([-1,1], [-1,1])
            ax.plot_surface(X, Y, np.zeros((2,2)), alpha=0.2, color='gray')
        
        plt.tight_layout()
        plt.show()

def main():
    print("="*50)
    print("Puma560 逆运动学教学程序")
    print("="*50)
    print("输入目标位置，程序将计算两种可能的关节角度解")
    
    robot = Puma560IK()
    
    while True:
        print("\n" + "-"*40)
        try:
            xyz = input("输入目标位置 x y z (米),例如0.600, 0.500, 0.500, 用空格分隔. 输入q退出): ").strip()
            
            if xyz.lower() == 'q':
                break
            
            target = [float(v) for v in xyz.split()]
            if len(target) != 3:
                print("请输入3个数值！")
                continue
            
            # 求解逆运动学
            solutions = robot.inverse_kinematics(*target)
            
            if solutions:
                print(f"\n找到两种解:")
                for i, q in enumerate(solutions):
                    # 验证正运动学
                    pos = robot.forward_kinematics(*q)
                    error = np.linalg.norm(pos - target)
                    
                    print(f"\n解{i+1}:")
                    print(f"  J1 = {q[0]:.1f}°")
                    print(f"  J2 = {q[1]:.1f}°")
                    print(f"  J3 = {q[2]:.1f}°")
                    print(f"  实际到达位置: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
                    print(f"  位置误差: {error:.6f} 米")
                
                # 可视化
                robot.visualize(target, solutions)
            else:
                print("目标点不可达！")
                print(f"可达范围: 半径 {robot.a2 + robot.a3:.3f} 米")
                
        except ValueError:
            print("输入格式错误！")
        except KeyboardInterrupt:
            break
    
    print("\n再见！")

if __name__ == "__main__":
    main()