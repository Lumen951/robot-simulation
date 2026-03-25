"""
UR3e简化模型仿真
包含: 正运动学、数值逆运动学、圆形轨迹规划、动画演示、简化动力学
"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.optimize import least_squares

# 设置中文字体
plt.rcParams['font.sans-serif'] = ['SimHei']
plt.rcParams['axes.unicode_minus'] = False


class UR3eRobot:
    """UR3e机器人运动学与轨迹规划仿真"""

    def __init__(self):
        # UR3e DH参数 (标准DH约定, 单位:米)
        self.d = [0.15185, 0, 0, 0.13105, 0.08535, 0.0921]
        self.a = [0, -0.24355, -0.2132, 0, 0, 0]
        self.alpha = [np.pi/2, 0, 0, np.pi/2, -np.pi/2, 0]

        # 关节限位 (度)
        self.joint_limits = [
            (-360, 360),   # J1
            (-360, 360),   # J2
            (-360, 360),   # J3
            (-360, 360),   # J4
            (-360, 360),   # J5
            (-360, 360)    # J6
        ]

        # 简化动力学参数 (kg)
        self.masses = [2.0, 2.5, 1.5, 1.0, 0.8, 0.5]  # 各连杆质量估计
        self.g = 9.81  # 重力加速度

    def dh_transform(self, theta, d, a, alpha):
        """
        计算标准DH齐次变换矩阵

        参数:
            theta: 关节角度 (弧度)
            d: 连杆偏置 (米)
            a: 连杆长度 (米)
            alpha: 连杆扭转角 (弧度)

        返回:
            4x4 齐次变换矩阵
        """
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)

        return np.array([
            [ct, -st * ca,  st * sa, a * ct],
            [st,  ct * ca, -ct * sa, a * st],
            [0,   sa,       ca,      d],
            [0,   0,        0,       1]
        ])

    def get_dh_params(self, theta_rad):
        """
        返回UR3e的DH参数表

        参数:
            theta_rad: 6个关节角度 (弧度)

        返回:
            DH参数列表 [[theta, d, a, alpha], ...]
        """
        return [
            [theta_rad[0], self.d[0], self.a[0], self.alpha[0]],
            [theta_rad[1], self.d[1], self.a[1], self.alpha[1]],
            [theta_rad[2], self.d[2], self.a[2], self.alpha[2]],
            [theta_rad[3], self.d[3], self.a[3], self.alpha[3]],
            [theta_rad[4], self.d[4], self.a[4], self.alpha[4]],
            [theta_rad[5], self.d[5], self.a[5], self.alpha[5]]
        ]

    def forward_kinematics(self, joints_deg):
        """
        正运动学计算

        参数:
            joints_deg: 6个关节角度 (度)

        返回:
            positions: 7x3数组, 各关节位置(包括基座)
            T_end: 4x4末端变换矩阵
        """
        theta = np.deg2rad(joints_deg)
        dh = self.get_dh_params(theta)

        positions = [np.array([0, 0, 0])]  # 基座位置
        T = np.eye(4)

        for params in dh:
            T = T @ self.dh_transform(*params)
            positions.append(T[:3, 3].copy())

        return np.array(positions), T

    def inverse_kinematics(self, x, y, z, q0=None, verbose=False):
        """
        数值逆运动学 (使用scipy.optimize.least_squares)

        参数:
            x, y, z: 目标位置 (米)
            q0: 初始猜测 (度), 若为None则使用默认值
            verbose: 是否打印调试信息

        返回:
            6个关节角度 (度), 失败返回None
        """
        target_pos = np.array([x, y, z])
        target_rot = np.array([0, 0, -1])  # 期望末端向下

        # 默认初始猜测 (UR3e典型姿态)
        if q0 is None:
            q0 = [0, -90, 90, 0, 0, 0]

        def error_func(q):
            """误差函数: 位置误差 + 姿态误差 (6维)"""
            _, T = self.forward_kinematics(q)
            pos = T[:3, 3]
            rot = T[:3, 2]  # z轴方向

            return np.concatenate([pos - target_pos, rot - target_rot])

        try:
            result = least_squares(
                error_func,
                q0,
                method='lm',  # Levenberg-Marquardt
                ftol=1e-4,
                xtol=1e-4,
                max_nfev=500
            )

            if verbose:
                print(f"  IK result: cost={result.cost:.6f}, "
                      f"final_error={np.linalg.norm(result.fun[:3]):.6f}")
                print(f"  solution: {result.x}")

            # 检查位置误差
            pos_error = np.linalg.norm(result.fun[:3])
            if pos_error < 0.02:  # 2cm误差内
                return result.x.tolist()

        except Exception as e:
            if verbose:
                print(f"  IK exception: {e}")

        return None

    def jacobian(self, joints_deg):
        """
        数值微分法计算6x6雅可比矩阵

        参数:
            joints_deg: 6个关节角度 (度)

        返回:
            J: 6x6 雅可比矩阵
        """
        delta = 1e-6
        q = np.array(joints_deg, dtype=float)
        _, T0 = self.forward_kinematics(q)

        # 提取末端位置和姿态
        pos0 = T0[:3, 3]

        J = np.zeros((6, 6))

        for j in range(6):
            q_plus = q.copy()
            q_plus[j] += np.rad2deg(delta)
            _, T_plus = self.forward_kinematics(q_plus)

            # 位置部分 (前3行)
            J[:3, j] = (T_plus[:3, 3] - pos0) / delta

            # 姿态部分 (后3行) - 使用旋转矩阵的差异
            R0 = T0[:3, :3]
            R_plus = T_plus[:3, :3]

            # 旋转轴的小角度近似
            dR = R_plus @ R0.T
            d_theta = np.array([
                dR[2, 1] - dR[1, 2],
                dR[0, 2] - dR[2, 0],
                dR[1, 0] - dR[0, 1]
            ]) / 2
            J[3:, j] = d_theta / delta

        return J

    def plan_circle(self, center, radius=0.05, n=100, plane='xy'):
        """
        圆形轨迹规划

        参数:
            center: 圆心 [cx, cy, cz] (米)
            radius: 半径 (米), 默认0.05m = 10cm直径
            n: 轨迹点数
            plane: 平面 ('xy', 'xz', 'yz')

        返回:
            path_xyz: nx3 轨迹点
            joints_list: 关节角度列表
        """
        angles = np.linspace(0, 2 * np.pi, n, endpoint=False)
        path = np.zeros((n, 3))

        if plane == 'xy':
            path[:, 0] = center[0] + radius * np.cos(angles)
            path[:, 1] = center[1] + radius * np.sin(angles)
            path[:, 2] = center[2]
        elif plane == 'xz':
            path[:, 0] = center[0] + radius * np.cos(angles)
            path[:, 2] = center[2] + radius * np.sin(angles)
            path[:, 1] = center[1]
        elif plane == 'yz':
            path[:, 1] = center[1] + radius * np.cos(angles)
            path[:, 2] = center[2] + radius * np.sin(angles)
            path[:, 0] = center[0]

        # 对每个轨迹点求逆运动学
        joints_list = []
        valid_path = []
        prev_q = [0, -90, 90, 0, 0, 0]  # 初始猜测

        for i, pt in enumerate(path):
            q = self.inverse_kinematics(*pt, q0=prev_q)
            if q is not None:
                # 验证精度
                _, T = self.forward_kinematics(q)
                actual_pos = T[:3, 3]
                error = np.linalg.norm(actual_pos - pt)
                if error < 0.02:  # 2cm误差内, 放宽容差
                    joints_list.append(q)
                    valid_path.append(pt)
                    prev_q = q  # 用当前解作为下一点的初值

        valid_path = np.array(valid_path)
        print(f"圆形轨迹规划: {len(joints_list)}/{n} 个点可达")

        return valid_path, joints_list

    def animate(self, path_xyz, joints_list, title="UR3e 圆形轨迹动画", save_path=None):
        """
        3D动画显示机器人沿轨迹运动

        参数:
            path_xyz: 期望轨迹点 (nx3)
            joints_list: 关节角度序列
            title: 动画标题
            save_path: 保存路径 (如 'animation.gif'), None则不保存
        """
        fig = plt.figure(figsize=(12, 9))
        ax = fig.add_subplot(111, projection='3d')

        # 预计算所有帧的关节位置
        all_positions = []
        for q in joints_list:
            pos, _ = self.forward_kinematics(q)
            all_positions.append(pos)

        # 绘制元素初始化
        line_robot, = ax.plot([], [], [], 'b-o', linewidth=3, markersize=6, label='UR3e机器人')
        line_trail, = ax.plot([], [], [], 'r-', linewidth=1.5, alpha=0.7, label='末端实际轨迹')
        line_target, = ax.plot(path_xyz[:, 0], path_xyz[:, 1], path_xyz[:, 2],
                               'g--', linewidth=1.5, alpha=0.4, label='期望轨迹')
        point_end = ax.scatter([], [], [], c='red', s=150, marker='*', zorder=5, label='末端')

        # 绘制地面
        xx, yy = np.meshgrid(np.linspace(-0.5, 0.5, 2), np.linspace(-0.5, 0.5, 2))
        ax.plot_surface(xx, yy, np.zeros((2, 2)), alpha=0.2, color='gray')

        # 设置坐标轴
        ax.set_xlim([-0.5, 0.5])
        ax.set_ylim([-0.5, 0.5])
        ax.set_zlim([0, 0.6])
        ax.set_xlabel('X (米)')
        ax.set_ylabel('Y (米)')
        ax.set_zlabel('Z (米)')
        ax.set_title(title)
        ax.legend(loc='upper left')

        # 轨迹累积
        trail_x, trail_y, trail_z = [], [], []

        def update(frame):
            pos = all_positions[frame]

            # 更新机器人连杆
            line_robot.set_data(pos[:, 0], pos[:, 1])
            line_robot.set_3d_properties(pos[:, 2])

            # 更新末端轨迹
            trail_x.append(pos[-1, 0])
            trail_y.append(pos[-1, 1])
            trail_z.append(pos[-1, 2])
            line_trail.set_data(trail_x, trail_y)
            line_trail.set_3d_properties(trail_z)

            # 更新末端点
            point_end._offsets3d = ([pos[-1, 0]], [pos[-1, 1]], [pos[-1, 2]])

            ax.set_title(f'{title} (帧 {frame + 1}/{len(joints_list)})')
            return line_robot, line_trail, point_end

        anim = FuncAnimation(fig, update, frames=len(joints_list),
                             interval=50, blit=False, repeat=True)

        # 保存动画
        if save_path:
            print(f"正在保存动画到 {save_path}...")
            anim.save(save_path, writer='pillow', fps=20, dpi=100)
            print(f"动画已保存: {save_path}")

        plt.show()
        return anim

    def plot_joint_curves(self, joints_list, title="圆形轨迹 - 关节角度变化"):
        """
        绘制关节角度随轨迹点变化的曲线

        参数:
            joints_list: 关节角度序列
            title: 图表标题
        """
        joints_arr = np.array(joints_list)
        t = np.arange(len(joints_list))

        fig, axes = plt.subplots(2, 3, figsize=(14, 8))
        fig.suptitle(title, fontsize=14)

        labels = ['J1 (基座旋转)', 'J2 (肩部)', 'J3 (肘部)',
                  'J4 (腕部旋转)', 'J5 (腕部俯仰)', 'J6 (腕部回转)']
        colors = ['#e41a1c', '#377eb8', '#4daf4a', '#984ea3', '#ff7f00', '#a65628']

        for i, ax in enumerate(axes.flat):
            ax.plot(t, joints_arr[:, i], color=colors[i], linewidth=1.5)
            ax.set_title(labels[i])
            ax.set_xlabel('轨迹点')
            ax.set_ylabel('角度 (°)')
            ax.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.show()

    def compute_torques_simple(self, joints_list):
        """
        简化力矩计算 - 雅可比转置法 + 重力补偿

        方法: tau = J.T @ F_ext + tau_gravity
        假设无外力, 仅计算重力补偿力矩

        参数:
            joints_list: 关节角度序列

        返回:
            torques: nx6 力矩数组 (N·m)
        """
        torques = []

        for q in joints_list:
            # 计算雅可比矩阵
            J = self.jacobian(q)

            # 简化重力补偿: 假设每个连杆质量集中在其末端
            # 使用雅可比转置计算等效重力力矩
            tau_gravity = np.zeros(6)

            positions, _ = self.forward_kinematics(q)

            for i in range(6):  # 对每个关节
                # 累加后续连杆对该关节的重力力矩
                for j in range(i, 6):
                    # 简化: 力臂近似为连杆在z方向的距离
                    dz = positions[j+1, 2] - positions[i, 2]
                    tau_gravity[i] += self.masses[j] * self.g * dz

            torques.append(tau_gravity)

        return np.array(torques)

    def plot_torques(self, torques, title="圆形轨迹 - 关节力矩变化"):
        """
        绘制关节力矩曲线

        参数:
            torques: nx6 力矩数组
            title: 图表标题
        """
        t = np.arange(len(torques))

        fig, axes = plt.subplots(2, 3, figsize=(14, 8))
        fig.suptitle(title, fontsize=14)

        labels = ['J1 (基座旋转)', 'J2 (肩部)', 'J3 (肘部)',
                  'J4 (腕部旋转)', 'J5 (腕部俯仰)', 'J6 (腕部回转)']
        colors = ['#e41a1c', '#377eb8', '#4daf4a', '#984ea3', '#ff7f00', '#a65628']

        for i, ax in enumerate(axes.flat):
            ax.plot(t, torques[:, i], color=colors[i], linewidth=1.5)
            ax.fill_between(t, torques[:, i], alpha=0.2, color=colors[i])
            ax.set_title(labels[i])
            ax.set_xlabel('轨迹点')
            ax.set_ylabel('力矩 (N·m)')
            ax.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.show()


def main():
    """主函数"""
    print("=" * 60)
    print("UR3e简化模型仿真")
    print("=" * 60)

    robot = UR3eRobot()

    # ---- 1. 打印DH参数表 ----
    print("\nUR3e DH参数表:")
    print(f"{'关节':<6} {'θ(rad)':<10} {'d(m)':<10} {'a(m)':<10} {'α(rad)':<10}")
    print("-" * 50)
    for i in range(6):
        print(f"J{i+1:<5} {'q'+str(i+1):<10} {robot.d[i]:<10.5f} "
              f"{robot.a[i]:<10.5f} {robot.alpha[i]:<10.4f}")

    # ---- 2. 测试正运动学 ----
    test_joints = [0, -90, 90, 0, 0, 0]
    positions, T_end = robot.forward_kinematics(test_joints)
    print(f"\n测试位形 {test_joints} (度):")
    print(f"末端位置: x={T_end[0,3]:.4f}, y={T_end[1,3]:.4f}, z={T_end[2,3]:.4f} 米")

    # 测试逆运动学 - 用已知的可达点
    target_x, target_y, target_z = T_end[0,3], T_end[1,3], T_end[2,3]
    print(f"\n测试逆运动学: 目标=({target_x:.4f}, {target_y:.4f}, {target_z:.4f})")
    q_ik = robot.inverse_kinematics(target_x, target_y, target_z, verbose=True)
    if q_ik is not None:
        print(f"IK求解成功: {[f'{x:.2f}' for x in q_ik]}")
        _, T_ik = robot.forward_kinematics(q_ik)
        error = np.linalg.norm([target_x-T_ik[0,3], target_y-T_ik[1,3], target_z-T_ik[2,3]])
        print(f"验证位置: ({T_ik[0,3]:.4f}, {T_ik[1,3]:.4f}, {T_ik[2,3]:.4f}), 误差={error:.6f}")
    else:
        print("IK求解失败!")

    # ---- 3. 圆形轨迹规划 ----
    print("\n" + "=" * 60)
    print("圆形轨迹规划 (XY平面, 直径10cm)")
    print("=" * 60)

    # 圆心位置: 根据测试位形的末端位置调整
    # 测试位形[0,-90,90,0,0,0]的末端在 (-0.2132, -0.2231, 0.31)
    # 选择一个在工作空间内的位置
    center = [-0.2, -0.15, 0.35]  # 调整到可达区域
    radius = 0.05  # 5cm半径 = 10cm直径

    print(f"圆心: ({center[0]}, {center[1]}, {center[2]}) 米")
    print(f"半径: {radius*100:.1f} cm (直径 {radius*2*100:.1f} cm)")

    path_xyz, joints_list = robot.plan_circle(center, radius, n=100, plane='xy')

    if len(joints_list) == 0:
        print("错误: 无法规划轨迹, 请调整圆心位置")
        return

    # ---- 4. 绘制关节角度曲线 ----
    robot.plot_joint_curves(joints_list, "圆形轨迹 - 关节角度变化")

    # ---- 5. 计算并绘制力矩 ----
    print("\n计算关节力矩...")
    torques = robot.compute_torques_simple(joints_list)
    print(f"力矩范围: J1 [{torques[:,0].min():.2f}, {torques[:,0].max():.2f}] N·m")
    print(f"         J2 [{torques[:,1].min():.2f}, {torques[:,1].max():.2f}] N·m")
    print(f"         J3 [{torques[:,2].min():.2f}, {torques[:,2].max():.2f}] N·m")

    robot.plot_torques(torques, "圆形轨迹 - 关节力矩变化 (简化)")

    # ---- 6. 动画 ----
    print("\n生成动画...")

    # 保存GIF到output目录
    import os
    output_dir = "output/ur3e-简化模型仿真"
    os.makedirs(output_dir, exist_ok=True)
    gif_path = os.path.join(output_dir, "ur3e_animation.gif")

    robot.animate(path_xyz, joints_list, "UR3e 圆形轨迹动画 (XY平面)", save_path=gif_path)

    print(f"\n仿真完成! 动画已保存到: {gif_path}")


if __name__ == "__main__":
    main()
