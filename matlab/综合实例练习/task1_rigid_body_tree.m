% TASK1_RIGID_BODY_TREE  使用rigidBodyTree创建2-DOF平面机械臂模型
%
% 任务一: 创建机械臂刚体树模型
%   - 定义连杆参数与关节约束
%   - 设置惯性参数
%   - 验证正运动学
%   - 可视化
%
% 参数来源: 二连杆计算力矩法实验
%   杆1: l1=1.0m, m1=10kg, I1=0.83 kg·m²
%   杆2: l2=0.8m, m2=5kg,  I2=0.21 kg·m²

clear; clc; close all;

% 输出目录
out_dir = fullfile(fileparts(mfilename('fullpath')), 'output');
if ~exist(out_dir, 'dir'), mkdir(out_dir); end

%% ========== 1. 创建刚体树模型 ==========
fprintf('========================================\n');
fprintf('任务一: 创建2-DOF机械臂刚体树模型\n');
fprintf('========================================\n\n');

% 平面机械臂, 重力沿-Y方向 (Y轴向上)
robot = rigidBodyTree('DataFormat', 'column');
robot.Gravity = [0 -9.81 0];

% 连杆参数
l1 = 1.0; m1 = 10.0; I1 = 0.83;
l2 = 0.8; m2 = 5.0;  I2 = 0.21;

%% ========== 2. 定义连杆1 ==========
joint1 = rigidBodyJoint('joint1', 'revolute');
setFixedTransform(joint1, trvec2tform([0 0 0]));  % 关节1在基座原点
joint1.JointAxis = [0 0 1];  % 绕Z轴旋转

link1 = rigidBody('link1');
link1.Joint = joint1;
link1.Mass = m1;
link1.CenterOfMass = [0 l1/2 0];  % 质心在杆中点, 沿Y方向
% 惯量 [Ixx, Iyy, Izz, Ixy, Iyz, Ixz]
% 平面杆件沿Y轴, Izz = (1/12)*m*l² ≈ 给定的I值
link1.Inertia = [I1 0.001 I1 0 0 0];

addBody(robot, link1, 'base');

%% ========== 3. 定义连杆2 ==========
joint2 = rigidBodyJoint('joint2', 'revolute');
setFixedTransform(joint2, trvec2tform([0 l1 0]));  % 关节2在连杆1末端 (距关节1为l1)
joint2.JointAxis = [0 0 1];

link2 = rigidBody('link2');
link2.Joint = joint2;
link2.Mass = m2;
link2.CenterOfMass = [0 l2/2 0];  % 质心在杆2中点
link2.Inertia = [I2 0.001 I2 0 0 0];

addBody(robot, link2, 'link1');

%% ========== 4. 添加末端执行器 (无质量, 用于获取末端位置) ==========
joint_ee = rigidBodyJoint('joint_ee', 'fixed');
setFixedTransform(joint_ee, trvec2tform([0 l2 0]));  % 末端距关节2为l2

end_effector = rigidBody('end_effector');
end_effector.Joint = joint_ee;
end_effector.Mass = 0.001;  % 近似无质量
end_effector.CenterOfMass = [0 0 0];
end_effector.Inertia = [0.001 0.001 0.001 0 0 0];

addBody(robot, end_effector, 'link2');

%% ========== 5. 打印模型信息 ==========
fprintf('--- 模型结构 ---\n');
showdetails(robot);
fprintf('\n');

%% ========== 6. 正运动学验证 ==========
fprintf('--- 正运动学验证 ---\n');
fprintf('验证方法: rigidBodyTree vs 手动计算\n\n');

% 在rigidBodyTree中, q=0时杆沿Y轴正方向
% FK: x_ee = -l1*sin(q1) - l2*sin(q1+q2)
%     y_ee =  l1*cos(q1) + l2*cos(q1+q2)

test_configs = {
    [0; 0],        'q = [0, 0] (竖直向上)';
    [pi/4; pi/4],  'q = [pi/4, pi/4]';
    [pi/2; 0],     'q = [pi/2, 0] (水平)';
    [pi/3; -pi/6], 'q = [pi/3, -pi/6]';
};

for i = 1:size(test_configs, 1)
    q = test_configs{i, 1};
    label = test_configs{i, 2};

    % rigidBodyTree计算
    T_fk = getTransform(robot, q, 'end_effector', 'base');
    pos_fk = T_fk(1:3, 4);

    % 手动计算
    x_manual = -l1*sin(q(1)) - l2*sin(q(1)+q(2));
    y_manual =  l1*cos(q(1)) + l2*cos(q(1)+q(2));

    error = norm(pos_fk(1:2) - [x_manual; y_manual]);
    fprintf('%s:\n', label);
    fprintf('  rigidBodyTree: [%.4f, %.4f]\n', pos_fk(1), pos_fk(2));
    fprintf('  手动计算:      [%.4f, %.4f]\n', x_manual, y_manual);
    fprintf('  误差: %.6f m\n\n', error);
end

%% ========== 7. 手动可视化不同位形 ==========
% 用正运动学手动计算关节位置, 避免 show() 与 subplot 不兼容
configs_plot = {[0; 0], [pi/4; pi/4], [pi/2; 0]};
titles_plot = {'q = [0, 0] (竖直)', 'q = [\pi/4, \pi/4]', 'q = [\pi/2, 0] (水平)'};

figure('Name', '2-DOF机械臂 - 可视化', 'Position', [100, 100, 1200, 500], 'Color', 'w');

for i = 1:3
    q = configs_plot{i};
    % 各关节位置 (q=0时沿Y轴正方向)
    x0 = 0;          y0 = 0;
    x1 = -l1*sin(q(1));                     y1 = l1*cos(q(1));
    x2 = -l1*sin(q(1)) - l2*sin(q(1)+q(2)); y2 = l1*cos(q(1)) + l2*cos(q(1)+q(2));

    subplot(1, 3, i); hold on;
    fill([0 0], [0 0.15], 'k', 'FaceAlpha', 0.3);  % 基座标记
    plot([x0 x1], [y0 y1], 'b-o', 'LineWidth', 4, 'MarkerSize', 8, 'MarkerFaceColor', 'b');
    plot([x1 x2], [y1 y2], 'r-o', 'LineWidth', 4, 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    plot(x2, y2, 'k^', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
    title(titles_plot{i}, 'FontSize', 11);
    xlabel('X (m)'); ylabel('Y (m)');
    axis([-2 2 -0.5 2.2]); axis equal; grid on;
end
sgtitle('2-DOF平面机械臂 - rigidBodyTree模型', 'FontSize', 14, 'FontWeight', 'bold');
saveas(gcf, fullfile(out_dir, 'fig1_rigid_body_tree_poses.png'));
fprintf('图1已保存: fig1_rigid_body_tree_poses.png\n');

%% ========== 8. 运动动画 ==========
figure('Name', '2-DOF机械臂 - 运动动画', 'Position', [300, 200, 600, 600], 'Color', 'w');
n_frames = 60;
q_anim = [linspace(0, pi/2, n_frames); linspace(0, pi/3, n_frames)];

for i = 1:n_frames
    q = q_anim(:, i);
    x1 = -l1*sin(q(1));                     y1 = l1*cos(q(1));
    x2 = -l1*sin(q(1)) - l2*sin(q(1)+q(2)); y2 = l1*cos(q(1)) + l2*cos(q(1)+q(2));

    cla; hold on;
    fill([0 0], [0 0.15], 'k', 'FaceAlpha', 0.3);
    plot([0 x1], [0 y1], 'b-o', 'LineWidth', 4, 'MarkerSize', 8, 'MarkerFaceColor', 'b');
    plot([x1 x2], [y1 y2], 'r-o', 'LineWidth', 4, 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    plot(x2, y2, 'k^', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
    title(sprintf('q_1 = %.2f rad, q_2 = %.2f rad', q(1), q(2)), 'FontSize', 12);
    xlabel('X (m)'); ylabel('Y (m)');
    axis([-2 2 -0.5 2.2]); axis equal; grid on;
    drawnow;
    pause(0.05);
end

%% ========== 9. 保存模型 ==========
save(fullfile(out_dir, 'two_link_robot.mat'), 'robot', 'l1', 'l2', 'm1', 'm2', 'I1', 'I2');
fprintf('模型已保存到 output/two_link_robot.mat\n');
fprintf('任务一完成!\n');
