import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

plt.rcParams['font.sans-serif'] = ['SimHei']
plt.rcParams['axes.unicode_minus'] = False


class Puma560Trajectory:
    """Puma560机器人轨迹规划与动画仿真"""

    def __init__(self):
        # Puma560 DH参数 (米)
        self.d1 = 0.5     # 基座高度
        self.a2 = 0.432   # 大臂长度
        self.a3 = 0.432   # 小臂长度
        self.d4 = 0.15    # 腕部偏置
        self.d6 = 0.1     # 末端长度

    def dh_transform(self, theta, d, a, alpha):
        """计算单个关节的DH齐次变换矩阵"""
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)
        return np.array([
            [ct, -st * ca,  st * sa, a * ct],
            [st,  ct * ca, -ct * sa, a * st],
            [0,   sa,       ca,      d],
            [0,   0,        0,       1]
        ])

    def get_dh_params(self, theta):
        """返回DH参数表，theta为6个关节角(弧度)"""
        return [
            [theta[0],          self.d1, 0,       -np.pi / 2],
            [theta[1] - np.pi / 2, 0,    self.a2,  0],
            [theta[2],          0,       0,       -np.pi / 2],
            [np.pi / 2,         self.d4, 0,        np.pi / 2],
            [theta[3],          0,       0,       -np.pi / 2],
            [theta[4],          0,       0,        np.pi / 2],
            [theta[5],          self.d6, 0,        0]
        ]

    def forward_kinematics(self, joints_deg):
        """
        正运动学：关节角(度) -> 各关节位置 + 末端变换矩阵
        返回 (positions, T_end)
        """
        theta = np.deg2rad(joints_deg)
        dh = self.get_dh_params(theta)

        positions = [np.array([0, 0, 0])]
        T = np.eye(4)
        for params in dh:
            T = T @ self.dh_transform(*params)
            positions.append(T[:3, 3].copy())

        return np.array(positions), T

    def inverse_kinematics(self, x, y, z, q0=None, max_iter=200, tol=1e-6):
        """
        数值迭代法逆运动学(阻尼最小二乘/DLS)
        目标: 末端位置到达 (x,y,z)，仅调节前3个关节，J4/J5/J6固定为0
        q0: 初始猜测(度)，默认用几何估计
        返回关节角(度) [j1,j2,j3,0,0,0]，失败返回None
        """
        target = np.array([x, y, z])

        # 初始猜测：用简单几何估计
        if q0 is None:
            j1_init = np.rad2deg(np.arctan2(y, x))
            j2_init = 30.0
            j3_init = 30.0
            q0 = [j1_init, j2_init, j3_init, 0, 0, 0]

        q = np.array(q0, dtype=float)
        damping = 0.5

        for _ in range(max_iter):
            _, T = self.forward_kinematics(q)
            pos = T[:3, 3]
            err = target - pos

            if np.linalg.norm(err) < tol:
                return q.tolist()

            # 计算前3关节的3x3雅可比
            J_full = self.jacobian(q)
            J3 = J_full[:, :3]

            # 阻尼最小二乘求解 dq = J^T (J J^T + λI)^{-1} e
            JJT = J3 @ J3.T + damping ** 2 * np.eye(3)
            dq3 = J3.T @ np.linalg.solve(JJT, err)

            q[:3] += np.rad2deg(dq3)

        # 最终检查精度
        _, T = self.forward_kinematics(q)
        if np.linalg.norm(T[:3, 3] - target) < 0.005:
            return q.tolist()
        return None

    def jacobian(self, joints_deg):
        """
        数值微分法计算6x3雅可比矩阵(仅前3个关节对末端位置的映射)
        J[i,j] = d(pos_i) / d(theta_j)
        """
        delta = 1e-6
        q = np.array(joints_deg, dtype=float)
        _, T0 = self.forward_kinematics(q)
        p0 = T0[:3, 3]

        J = np.zeros((3, 6))
        for j in range(6):
            q_plus = q.copy()
            q_plus[j] += np.rad2deg(delta)
            _, T_plus = self.forward_kinematics(q_plus)
            J[:, j] = (T_plus[:3, 3] - p0) / delta

        return J

    def print_transforms(self, joints_deg):
        """打印各关节间的齐次变换矩阵"""
        theta = np.deg2rad(joints_deg)
        dh = self.get_dh_params(theta)
        labels = ['T01(基座→J1)', 'T12(J1→J2)', 'T23(J2→J3)',
                  'T34(结构变换)', 'T45(J4→J5)', 'T56(J5→J6)', 'T6E(J6→末端)']

        print("\n各关节间齐次变换矩阵:")
        print("=" * 60)
        T_total = np.eye(4)
        for i, (params, label) in enumerate(zip(dh, labels)):
            Ti = self.dh_transform(*params)
            T_total = T_total @ Ti
            print(f"\n{label}:")
            print(np.array2string(Ti, precision=4, suppress_small=True))

        print(f"\n总变换矩阵 T0E (基座→末端):")
        print(np.array2string(T_total, precision=4, suppress_small=True))
        print(f"末端位置: x={T_total[0,3]:.4f}, y={T_total[1,3]:.4f}, z={T_total[2,3]:.4f}")

    def _plan_path(self, path, label="轨迹"):
        """
        通用轨迹规划：对路径点序列逐点求逆运动学
        利用上一个解作为下一个点的初始猜测，加速收敛并保证连续性
        """
        joints_list = []
        valid_path = []
        prev_q = None

        for pt in path:
            q = self.inverse_kinematics(*pt, q0=prev_q)
            if q is not None:
                pos, _ = self.forward_kinematics(q)
                err = np.linalg.norm(pos[-1] - pt)
                if err < 0.01:
                    joints_list.append(q)
                    valid_path.append(pt)
                    prev_q = q

        n = len(path)
        if len(joints_list) < 2:
            print(f"警告: {label}大部分点不可达，请调整参数")
            return None, None

        print(f"{label}: {len(joints_list)}/{n} 个点可达")
        return np.array(valid_path), joints_list

    def plan_line(self, start, end, n=100):
        """直线轨迹规划，start/end为[x,y,z]"""
        path = np.linspace(start, end, n)
        return self._plan_path(path, "直线轨迹")

    def plan_circle(self, center, radius, n=100):
        """圆形轨迹规划(XY平面，z固定)，center为[cx,cy,cz]"""
        angles = np.linspace(0, 2 * np.pi, n, endpoint=False)
        path = np.zeros((n, 3))
        path[:, 0] = center[0] + radius * np.cos(angles)
        path[:, 1] = center[1] + radius * np.sin(angles)
        path[:, 2] = center[2]
        return self._plan_path(path, "圆形轨迹")

    def animate(self, path_xyz, joints_list, title="轨迹动画"):
        """3D动画显示机器人沿轨迹运动"""
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')

        # 预计算所有帧的关节位置
        all_positions = []
        for q in joints_list:
            pos, _ = self.forward_kinematics(q)
            all_positions.append(pos)

        # 绘制元素
        line_robot, = ax.plot([], [], [], 'b-o', linewidth=3, markersize=6)
        line_trail, = ax.plot([], [], [], 'r-', linewidth=1.5, alpha=0.7, label='末端轨迹')
        line_target, = ax.plot(path_xyz[:, 0], path_xyz[:, 1], path_xyz[:, 2],
                               '--', color='gray', linewidth=1, alpha=0.5, label='规划路径')
        point_end = ax.scatter([], [], [], c='red', s=120, marker='*', zorder=5)

        # 地面
        gx, gy = np.meshgrid([-1, 1], [-1, 1])
        ax.plot_surface(gx, gy, np.zeros((2, 2)), alpha=0.15, color='lightgray')

        ax.set_xlim([-1, 1])
        ax.set_ylim([-1, 1])
        ax.set_zlim([-0.2, 1.5])
        ax.set_xlabel('X (米)')
        ax.set_ylabel('Y (米)')
        ax.set_zlabel('Z (米)')
        ax.set_title(title)
        ax.legend(loc='upper left')

        trail_x, trail_y, trail_z = [], [], []

        def update(frame):
            pos = all_positions[frame]
            line_robot.set_data(pos[:, 0], pos[:, 1])
            line_robot.set_3d_properties(pos[:, 2])

            trail_x.append(pos[-1, 0])
            trail_y.append(pos[-1, 1])
            trail_z.append(pos[-1, 2])
            line_trail.set_data(trail_x, trail_y)
            line_trail.set_3d_properties(trail_z)

            point_end._offsets3d = ([pos[-1, 0]], [pos[-1, 1]], [pos[-1, 2]])

            ax.set_title(f'{title} (帧 {frame + 1}/{len(joints_list)})')
            return line_robot, line_trail, point_end

        anim = FuncAnimation(fig, update, frames=len(joints_list),
                             interval=50, blit=False, repeat=True)
        plt.show()
        return anim

    def plot_joint_curves(self, joints_list, title="关节角度变化"):
        """绘制关节角度随轨迹点变化的曲线"""
        joints_arr = np.array(joints_list)
        t = np.arange(len(joints_list))

        fig, axes = plt.subplots(2, 3, figsize=(14, 8))
        fig.suptitle(title, fontsize=14)
        labels = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']
        colors = ['#e41a1c', '#377eb8', '#4daf4a', '#984ea3', '#ff7f00', '#a65628']

        for i, ax in enumerate(axes.flat):
            ax.plot(t, joints_arr[:, i], color=colors[i], linewidth=1.5)
            ax.set_title(f'{labels[i]}')
            ax.set_xlabel('轨迹点')
            ax.set_ylabel('角度 (°)')
            ax.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.show()

    def plot_manipulability(self, joints_list, title="可操作度变化"):
        """绘制沿轨迹的可操作度(雅可比矩阵奇异值)"""
        manip = []
        for q in joints_list:
            J = self.jacobian(q)
            # 取3x3子矩阵(前3关节对位置的映射)的行列式绝对值
            J3 = J[:, :3]
            manip.append(abs(np.linalg.det(J3)))

        fig, ax = plt.subplots(figsize=(10, 4))
        ax.plot(manip, 'g-', linewidth=1.5)
        ax.fill_between(range(len(manip)), manip, alpha=0.2, color='green')
        ax.set_xlabel('轨迹点')
        ax.set_ylabel('可操作度 |det(J)|')
        ax.set_title(title)
        ax.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.show()


def main():
    print("=" * 60)
    print("Puma560 轨迹规划与动画仿真")
    print("=" * 60)

    robot = Puma560Trajectory()

    # ---- 1. DH参数表 ----
    print("\nPuma560 DH参数表:")
    print(f"{'关节':<6} {'d(m)':<10} {'a(m)':<10} {'α(rad)':<12}")
    print("-" * 40)
    dh_info = [
        ('J1', robot.d1, 0, -np.pi / 2),
        ('J2', 0, robot.a2, 0),
        ('J3', 0, 0, -np.pi / 2),
        ('结构', robot.d4, 0, np.pi / 2),
        ('J4', 0, 0, -np.pi / 2),
        ('J5', 0, 0, np.pi / 2),
        ('J6', robot.d6, 0, 0),
    ]
    for name, d, a, alpha in dh_info:
        print(f"{name:<6} {d:<10.3f} {a:<10.3f} {alpha:<12.4f}")

    # ---- 2. 坐标变换矩阵 ----
    test_joints = [30, 45, 30, 0, 0, 0]
    print(f"\n典型位形 {test_joints} 下的变换矩阵:")
    robot.print_transforms(test_joints)

    # ---- 3. 雅可比矩阵 ----
    J = robot.jacobian(test_joints)
    print(f"\n雅可比矩阵 (位形 {test_joints}):")
    print("J (3×6) = ")
    print(np.array2string(J, precision=4, suppress_small=True))
    J3 = J[:, :3]
    print(f"前3关节可操作度: |det(J3)| = {abs(np.linalg.det(J3)):.6f}")

    # ---- 4. 直线轨迹 ----
    print("\n" + "=" * 60)
    print("直线轨迹规划")
    print("=" * 60)
    start = np.array([0.5, 0.3, 0.6])
    end = np.array([0.4, -0.2, 0.75])
    print(f"起点: {start}")
    print(f"终点: {end}")

    line_path, line_joints = robot.plan_line(start, end, n=100)
    if line_path is not None:
        robot.plot_joint_curves(line_joints, "直线轨迹 — 关节角度变化")
        robot.plot_manipulability(line_joints, "直线轨迹 — 可操作度变化")
        robot.animate(line_path, line_joints, "Puma560 直线轨迹动画")

    # ---- 5. 圆形轨迹 ----
    print("\n" + "=" * 60)
    print("圆形轨迹规划")
    print("=" * 60)
    center = np.array([0.4, 0, 0.65])
    radius = 0.12
    print(f"圆心: {center}, 半径: {radius}")

    circle_path, circle_joints = robot.plan_circle(center, radius, n=100)
    if circle_path is not None:
        robot.plot_joint_curves(circle_joints, "圆形轨迹 — 关节角度变化")
        robot.plot_manipulability(circle_joints, "圆形轨迹 — 可操作度变化")
        robot.animate(circle_path, circle_joints, "Puma560 圆形轨迹动画")

    print("\n仿真完成！")


if __name__ == "__main__":
    main()
