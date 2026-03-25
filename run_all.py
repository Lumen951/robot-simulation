"""
批量运行 src/ 下所有脚本，将数值输出和图片保存到 output/ 目录。
用法: uv run run_all.py
"""
import os
import sys
import io
import importlib.util
import numpy as np
import matplotlib
matplotlib.use('Agg')  # 非交互式后端，直接保存图片
import matplotlib.pyplot as plt

plt.rcParams['font.sans-serif'] = ['SimHei']
plt.rcParams['axes.unicode_minus'] = False

ROOT = os.path.dirname(os.path.abspath(__file__))
OUTPUT_DIR = os.path.join(ROOT, 'output')
SRC_DIR = os.path.join(ROOT, 'src')


def ensure_dir(path):
    os.makedirs(path, exist_ok=True)


def save_figures(out_dir, prefix='fig'):
    """保存当前所有打开的 matplotlib figure，然后关闭。"""
    figs = [plt.figure(n) for n in plt.get_fignums()]
    paths = []
    for i, fig in enumerate(figs):
        name = f'{prefix}_{i+1}.png' if len(figs) > 1 else f'{prefix}.png'
        p = os.path.join(out_dir, name)
        fig.savefig(p, dpi=150, bbox_inches='tight')
        paths.append(p)
    plt.close('all')
    return paths


def run_script(script_path, out_dir, capture=True):
    """执行一个脚本文件，捕获 stdout 并保存图片。"""
    ensure_dir(out_dir)

    # 捕获 print 输出
    old_stdout = sys.stdout
    if capture:
        sys.stdout = buffer = io.StringIO()

    # 关闭之前残留的图
    plt.close('all')

    try:
        spec = importlib.util.spec_from_file_location("__main__", script_path)
        mod = importlib.util.module_from_spec(spec)
        mod.__name__ = "__main__"
        # 阻止 plt.show() 弹窗，替换为空操作
        original_show = plt.show
        plt.show = lambda *a, **kw: None
        spec.loader.exec_module(mod)
    finally:
        plt.show = original_show
        if capture:
            sys.stdout = old_stdout

    # 保存文本输出
    if capture:
        text = buffer.getvalue()
        if text.strip():
            txt_path = os.path.join(out_dir, 'output.txt')
            with open(txt_path, 'w', encoding='utf-8') as f:
                f.write(text)
            print(f'  -> {txt_path}')

    # 保存图片
    fig_paths = save_figures(out_dir)
    for p in fig_paths:
        print(f'  -> {p}')


def run_interactive_puma560_fk(out_dir):
    """puma560正运动学：用3组典型参数运行。"""
    ensure_dir(out_dir)
    sys.path.insert(0, SRC_DIR)

    spec = importlib.util.spec_from_file_location(
        "_puma_fk", os.path.join(SRC_DIR, 'puma560-正运动学.py'))
    mod = importlib.util.module_from_spec(spec)

    original_show = plt.show
    plt.show = lambda *a, **kw: None
    old_stdout = sys.stdout
    sys.stdout = buffer = io.StringIO()

    try:
        spec.loader.exec_module(mod)
        robot = mod.SimplePuma560()

        configs = [
            (0, 0, 0, 0, 0, 0, '初始位形'),
            (0, 45, 45, 0, 0, 0, '伸展位形'),
            (30, 45, 30, 45, 60, 90, '弯曲位形'),
        ]
        for j1, j2, j3, j4, j5, j6, label in configs:
            plt.close('all')
            robot.visualize(j1, j2, j3, j4, j5, j6)
            save_figures(out_dir, prefix=f'fig_{label}')
    finally:
        plt.show = original_show
        sys.stdout = old_stdout

    text = buffer.getvalue()
    if text.strip():
        with open(os.path.join(out_dir, 'output.txt'), 'w', encoding='utf-8') as f:
            f.write(text)
        print(f'  -> {os.path.join(out_dir, "output.txt")}')

    for fname in os.listdir(out_dir):
        if fname.endswith('.png'):
            print(f'  -> {os.path.join(out_dir, fname)}')


def run_interactive_puma560_ik(out_dir):
    """puma560逆运动学：用固定目标点运行。"""
    ensure_dir(out_dir)

    spec = importlib.util.spec_from_file_location(
        "_puma_ik", os.path.join(SRC_DIR, 'puma560-逆运动学.py'))
    mod = importlib.util.module_from_spec(spec)

    original_show = plt.show
    plt.show = lambda *a, **kw: None
    old_stdout = sys.stdout
    sys.stdout = buffer = io.StringIO()

    try:
        spec.loader.exec_module(mod)
        robot = mod.Puma560IK()

        targets = [
            (0.6, 0.5, 0.5),
            (0.3, 0.3, 0.8),
        ]
        for tx, ty, tz in targets:
            plt.close('all')
            solutions = robot.inverse_kinematics(tx, ty, tz)
            if solutions:
                print(f"\n目标位置: ({tx}, {ty}, {tz})")
                for i, q in enumerate(solutions):
                    pos = robot.forward_kinematics(*q)
                    error = np.linalg.norm(pos - np.array([tx, ty, tz]))
                    print(f"  解{i+1}: J1={q[0]:.1f}°, J2={q[1]:.1f}°, J3={q[2]:.1f}°")
                    print(f"    实际位置: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}), 误差: {error:.6f}")
                robot.visualize([tx, ty, tz], solutions)
                save_figures(out_dir, prefix=f'fig_target_{tx}_{ty}_{tz}')
    finally:
        plt.show = original_show
        sys.stdout = old_stdout

    text = buffer.getvalue()
    if text.strip():
        with open(os.path.join(out_dir, 'output.txt'), 'w', encoding='utf-8') as f:
            f.write(text)
        print(f'  -> {os.path.join(out_dir, "output.txt")}')

    for fname in os.listdir(out_dir):
        if fname.endswith('.png'):
            print(f'  -> {os.path.join(out_dir, fname)}')


def run_interactive_jacobian(out_dir):
    """雅可比矩阵：用固定角度运行。"""
    ensure_dir(out_dir)

    spec = importlib.util.spec_from_file_location(
        "_jacobian", os.path.join(SRC_DIR, '雅可比矩阵.py'))
    mod = importlib.util.module_from_spec(spec)

    original_show = plt.show
    plt.show = lambda *a, **kw: None
    old_stdout = sys.stdout
    sys.stdout = buffer = io.StringIO()

    try:
        spec.loader.exec_module(mod)
        demo = mod.JacobianDemo()

        angles = [(30, 45), (60, 30), (45, 90)]
        for t1, t2 in angles:
            plt.close('all')
            demo.demonstrate(t1, t2)
            save_figures(out_dir, prefix=f'fig_{t1}_{t2}')
    finally:
        plt.show = original_show
        sys.stdout = old_stdout

    text = buffer.getvalue()
    if text.strip():
        with open(os.path.join(out_dir, 'output.txt'), 'w', encoding='utf-8') as f:
            f.write(text)
        print(f'  -> {os.path.join(out_dir, "output.txt")}')

    for fname in os.listdir(out_dir):
        if fname.endswith('.png'):
            print(f'  -> {os.path.join(out_dir, fname)}')


def run_puma560_trajectory(out_dir):
    """puma560轨迹规划：运行直线和圆形轨迹，保存静态图。"""
    ensure_dir(out_dir)

    spec = importlib.util.spec_from_file_location(
        "_puma_traj", os.path.join(SRC_DIR, 'puma560-轨迹规划.py'))
    mod = importlib.util.module_from_spec(spec)

    original_show = plt.show
    plt.show = lambda *a, **kw: None
    old_stdout = sys.stdout
    sys.stdout = buffer = io.StringIO()

    try:
        spec.loader.exec_module(mod)
        robot = mod.Puma560Trajectory()

        # 打印变换矩阵和雅可比
        test_joints = [30, 45, 30, 0, 0, 0]
        robot.print_transforms(test_joints)
        J = robot.jacobian(test_joints)
        print(f"\n雅可比矩阵 (位形 {test_joints}):")
        print(np.array2string(J, precision=4, suppress_small=True))

        # 直线轨迹
        start = np.array([0.5, 0.3, 0.6])
        end = np.array([0.4, -0.2, 0.75])
        lp, lj = robot.plan_line(start, end, n=100)
        if lj:
            plt.close('all')
            robot.plot_joint_curves(lj, "直线轨迹 - 关节角度变化")
            save_figures(out_dir, prefix='fig_直线_关节角度')
            plt.close('all')
            robot.plot_manipulability(lj, "直线轨迹 - 可操作度变化")
            save_figures(out_dir, prefix='fig_直线_可操作度')

        # 圆形轨迹
        center = np.array([0.4, 0, 0.65])
        cp, cj = robot.plan_circle(center, 0.12, n=100)
        if cj:
            plt.close('all')
            robot.plot_joint_curves(cj, "圆形轨迹 - 关节角度变化")
            save_figures(out_dir, prefix='fig_圆形_关节角度')
            plt.close('all')
            robot.plot_manipulability(cj, "圆形轨迹 - 可操作度变化")
            save_figures(out_dir, prefix='fig_圆形_可操作度')
    finally:
        plt.show = original_show
        sys.stdout = old_stdout

    text = buffer.getvalue()
    if text.strip():
        with open(os.path.join(out_dir, 'output.txt'), 'w', encoding='utf-8') as f:
            f.write(text)
        print(f'  -> {os.path.join(out_dir, "output.txt")}')

    for fname in sorted(os.listdir(out_dir)):
        if fname.endswith('.png'):
            print(f'  -> {os.path.join(out_dir, fname)}')


def run_ur3e_simulation(out_dir):
    """UR3e简化模型仿真：圆形轨迹+动画+动力学"""
    ensure_dir(out_dir)

    spec = importlib.util.spec_from_file_location(
        "_ur3e", os.path.join(SRC_DIR, 'ur3e-简化模型仿真.py'))
    mod = importlib.util.module_from_spec(spec)

    original_show = plt.show
    plt.show = lambda *a, **kw: None
    old_stdout = sys.stdout
    sys.stdout = buffer = io.StringIO()

    try:
        spec.loader.exec_module(mod)
        robot = mod.UR3eRobot()

        # 打印DH参数
        print("\nUR3e DH参数表:")
        print(f"{'关节':<6} {'θ(rad)':<10} {'d(m)':<10} {'a(m)':<10} {'α(rad)':<10}")
        print("-" * 50)
        for i in range(6):
            print(f"J{i+1:<5} {'q'+str(i+1):<10} {robot.d[i]:<10.5f} "
                  f"{robot.a[i]:<10.5f} {robot.alpha[i]:<10.4f}")

        # 测试正运动学
        test_joints = [0, -90, 90, 0, 0, 0]
        positions, T_end = robot.forward_kinematics(test_joints)
        print(f"\n测试位形 {test_joints} (度):")
        print(f"末端位置: x={T_end[0,3]:.4f}, y={T_end[1,3]:.4f}, z={T_end[2,3]:.4f} 米")

        # 圆形轨迹规划
        center = [-0.2, -0.15, 0.35]
        radius = 0.05  # 10cm直径
        print(f"\n圆形轨迹规划 (XY平面, 直径10cm)")
        print(f"圆心: {center}, 半径: {radius*100:.1f} cm")

        path_xyz, joints_list = robot.plan_circle(center, radius, n=100, plane='xy')

        if len(joints_list) > 0:
            # 关节角度曲线
            plt.close('all')
            robot.plot_joint_curves(joints_list, "圆形轨迹 - 关节角度变化")
            save_figures(out_dir, prefix='fig_关节角度')

            # 力矩计算
            torques = robot.compute_torques_simple(joints_list)
            plt.close('all')
            robot.plot_torques(torques, "圆形轨迹 - 关节力矩变化")
            save_figures(out_dir, prefix='fig_力矩变化')
    finally:
        plt.show = original_show
        sys.stdout = old_stdout

    text = buffer.getvalue()
    if text.strip():
        with open(os.path.join(out_dir, 'output.txt'), 'w', encoding='utf-8') as f:
            f.write(text)
        print(f'  -> {os.path.join(out_dir, "output.txt")}')

    for fname in sorted(os.listdir(out_dir)):
        if fname.endswith('.png'):
            print(f'  -> {os.path.join(out_dir, fname)}')


def main():
    print('=' * 60)
    print('批量运行 src/ 脚本，输出保存到 output/')
    print('=' * 60)

    # 非交互式脚本，直接执行
    simple_scripts = [
        '旋转矩阵',
        '齐次变换',
        '单连杆动能计算',
        '单连杆势能计算',
        '单摆的拉格朗日方程',
        '二连杆机构DH建模',
        '二连杆的拉格朗日分析',
        '二连杆的牛顿欧拉法分析',
    ]

    for name in simple_scripts:
        script_path = os.path.join(SRC_DIR, f'{name}.py')
        out_dir = os.path.join(OUTPUT_DIR, name)
        print(f'\n[运行] {name}')
        try:
            run_script(script_path, out_dir)
        except Exception as e:
            print(f'  !! 错误: {e}')

    # 交互式脚本，用固定参数调用
    print(f'\n[运行] puma560-正运动学 (3组位形)')
    try:
        run_interactive_puma560_fk(os.path.join(OUTPUT_DIR, 'puma560-正运动学'))
    except Exception as e:
        print(f'  !! 错误: {e}')

    print(f'\n[运行] puma560-逆运动学 (2组目标点)')
    try:
        run_interactive_puma560_ik(os.path.join(OUTPUT_DIR, 'puma560-逆运动学'))
    except Exception as e:
        print(f'  !! 错误: {e}')

    print(f'\n[运行] 雅可比矩阵 (3组角度)')
    try:
        run_interactive_jacobian(os.path.join(OUTPUT_DIR, '雅可比矩阵'))
    except Exception as e:
        print(f'  !! 错误: {e}')

    print(f'\n[运行] puma560-轨迹规划 (直线+圆形轨迹)')
    try:
        run_puma560_trajectory(os.path.join(OUTPUT_DIR, 'puma560-轨迹规划'))
    except Exception as e:
        print(f'  !! 错误: {e}')

    print(f'\n[运行] ur3e-简化模型仿真 (圆形轨迹+动力学)')
    try:
        run_ur3e_simulation(os.path.join(OUTPUT_DIR, 'ur3e-简化模型仿真'))
    except Exception as e:
        print(f'  !! 错误: {e}')

    print('\n' + '=' * 60)
    print('全部完成！输出目录: output/')
    print('=' * 60)


if __name__ == '__main__':
    main()
