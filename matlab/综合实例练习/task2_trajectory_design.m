% TASK2_TRAJECTORY_DESIGN  关节空间轨迹设计
%
% 任务二: 设计关节空间期望轨迹
%   - 正弦轨迹: q1=sin(2t), q2=0.5*sin(3t)
%   - 五次多项式插值: 平滑点到点运动
%   - 验证轨迹平滑性 (位置、速度、加速度连续)
%   - 保存轨迹数据供Simulink使用

clear; clc; close all;

% 输出目录
out_dir = fullfile(fileparts(mfilename('fullpath')), 'output');
if ~exist(out_dir, 'dir'), mkdir(out_dir); end

%% ========== 参数设置 ==========
T = 10;        % 仿真总时间 (s)
dt = 0.01;     % 采样时间 (s)
t = 0:dt:T;
N = length(t);

fprintf('========================================\n');
fprintf('任务二: 关节空间轨迹设计\n');
fprintf('========================================\n\n');

%% ========== 轨迹1: 正弦运动 ==========
fprintf('--- 轨迹1: 正弦运动 ---\n');

% q1_d(t) = sin(2t),  dq1_d = 2cos(2t),  ddq1_d = -4sin(2t)
% q2_d(t) = 0.5sin(3t), dq2_d = 1.5cos(3t), ddq2_d = -4.5sin(3t)

q_d_sin = [sin(2*t); 0.5*sin(3*t)];
dq_d_sin = [2*cos(2*t); 1.5*cos(3*t)];
ddq_d_sin = [-4*sin(2*t); -4.5*sin(3*t)];

fprintf('关节1: q1_d = sin(2t), 幅值=1.0 rad, 频率=2 rad/s\n');
fprintf('关节2: q2_d = 0.5*sin(3t), 幅值=0.5 rad, 频率=3 rad/s\n');
fprintf('位置范围: q1=[%.2f, %.2f], q2=[%.2f, %.2f] rad\n', ...
    min(q_d_sin(1,:)), max(q_d_sin(1,:)), min(q_d_sin(2,:)), max(q_d_sin(2,:)));

%% ========== 轨迹2: 五次多项式插值 ==========
fprintf('\n--- 轨迹2: 五次多项式插值 ---\n');

% 起点和终点的位置、速度、加速度
q0 = [0; 0];       % 起始位置
qf = [pi/2; pi/4]; % 目标位置
dq0 = [0; 0];      % 起始速度
dqf = [0; 0];      % 终止速度
ddq0 = [0; 0];     % 起始加速度
ddqf = [0; 0];     % 终止加速度

% 五次多项式系数: q(t) = a0 + a1*t + a2*t² + a3*t³ + a4*t⁴ + a5*t⁵
% 边界条件: q(0), q(T), dq(0), dq(T), ddq(0), ddq(T)
q_d_poly = zeros(2, N);
dq_d_poly = zeros(2, N);
ddq_d_poly = zeros(2, N);

for j = 1:2  % 两个关节分别计算
    a0 = q0(j);
    a1 = dq0(j);
    a2 = ddq0(j) / 2;
    % 求解 a3, a4, a5
    % q(T) = a0 + a1*T + a2*T² + a3*T³ + a4*T⁴ + a5*T⁵ = qf
    % dq(T) = a1 + 2*a2*T + 3*a3*T² + 4*a4*T³ + 5*a5*T⁴ = dqf
    % ddq(T) = 2*a2 + 6*a3*T + 12*a4*T² + 20*a5*T³ = ddqf
    A_mat = [T^3, T^4, T^5;
             3*T^2, 4*T^3, 5*T^4;
             6*T, 12*T^2, 20*T^3];
    b_vec = [qf(j) - a0 - a1*T - a2*T^2;
             dqf(j) - a1 - 2*a2*T;
             ddqf(j) - 2*a2];
    coeffs = A_mat \ b_vec;
    a3 = coeffs(1); a4 = coeffs(2); a5 = coeffs(3);

    q_d_poly(j, :)    = a0 + a1*t + a2*t.^2 + a3*t.^3 + a4*t.^4 + a5*t.^5;
    dq_d_poly(j, :)   = a1 + 2*a2*t + 3*a3*t.^2 + 4*a4*t.^3 + 5*a5*t.^4;
    ddq_d_poly(j, :)  = 2*a2 + 6*a3*t + 12*a4*t.^2 + 20*a5*t.^3;
end

fprintf('起始: q0=[0, 0], 终止: qf=[pi/2, pi/4]\n');
fprintf('边界条件: 零速度、零加速度\n');

%% ========== 验证轨迹平滑性 ==========
fprintf('\n--- 轨迹平滑性验证 ---\n');

% 检查数值微分与解析导数的一致性
dq_numerical = [diff(q_d_sin(1,:))/dt, 0; diff(q_d_sin(2,:))/dt, 0];
err_smooth = max(abs(dq_numerical(1,1:end-1) - dq_d_sin(1,1:end-1)));
fprintf('正弦轨迹 - 数值/解析导数最大误差: %.6e (应接近0)\n', err_smooth);

% 五次多项式端点验证
err_start = norm(q_d_poly(:,1) - q0);
err_end = norm(q_d_poly(:,end) - qf);
fprintf('五次多项式 - 起点误差: %.6e, 终点误差: %.6e\n', err_start, err_end);

%% ========== 绘图: 正弦轨迹 ==========
figure('Name', '正弦轨迹', 'Position', [100, 100, 1200, 800], 'Color', 'w');

% 位置
subplot(3, 2, 1);
plot(t, q_d_sin(1,:), 'b-', 'LineWidth', 1.5);
ylabel('q_1 (rad)'); title('关节1 - 位置'); grid on;
subplot(3, 2, 2);
plot(t, q_d_sin(2,:), 'r-', 'LineWidth', 1.5);
ylabel('q_2 (rad)'); title('关节2 - 位置'); grid on;

% 速度
subplot(3, 2, 3);
plot(t, dq_d_sin(1,:), 'b-', 'LineWidth', 1.5);
ylabel('dq_1 (rad/s)'); title('关节1 - 速度'); grid on;
subplot(3, 2, 4);
plot(t, dq_d_sin(2,:), 'r-', 'LineWidth', 1.5);
ylabel('dq_2 (rad/s)'); title('关节2 - 速度'); grid on;

% 加速度
subplot(3, 2, 5);
plot(t, ddq_d_sin(1,:), 'b-', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('ddq_1 (rad/s²)'); title('关节1 - 加速度'); grid on;
subplot(3, 2, 6);
plot(t, ddq_d_sin(2,:), 'r-', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('ddq_2 (rad/s²)'); title('关节2 - 加速度'); grid on;

sgtitle('正弦轨迹 (q_1=sin2t, q_2=0.5sin3t)', 'FontSize', 14, 'FontWeight', 'bold');
saveas(gcf, fullfile(out_dir, 'fig2_sin_trajectory.png'));
fprintf('图2已保存: fig2_sin_trajectory.png\n');

%% ========== 绘图: 五次多项式轨迹 ==========
figure('Name', '五次多项式轨迹', 'Position', [150, 100, 1200, 800], 'Color', 'w');

subplot(3, 2, 1);
plot(t, q_d_poly(1,:), 'b-', 'LineWidth', 1.5);
hold on; plot([0 T], [q0(1) q0(1)], 'k--', 'LineWidth', 0.8);
plot([0 T], [qf(1) qf(1)], 'k--', 'LineWidth', 0.8);
ylabel('q_1 (rad)'); title('关节1 - 位置'); grid on;

subplot(3, 2, 2);
plot(t, q_d_poly(2,:), 'r-', 'LineWidth', 1.5);
hold on; plot([0 T], [q0(2) q0(2)], 'k--', 'LineWidth', 0.8);
plot([0 T], [qf(2) qf(2)], 'k--', 'LineWidth', 0.8);
ylabel('q_2 (rad)'); title('关节2 - 位置'); grid on;

subplot(3, 2, 3);
plot(t, dq_d_poly(1,:), 'b-', 'LineWidth', 1.5);
ylabel('dq_1 (rad/s)'); title('关节1 - 速度'); grid on;
subplot(3, 2, 4);
plot(t, dq_d_poly(2,:), 'r-', 'LineWidth', 1.5);
ylabel('dq_2 (rad/s)'); title('关节2 - 速度'); grid on;

subplot(3, 2, 5);
plot(t, ddq_d_poly(1,:), 'b-', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('ddq_1 (rad/s²)'); title('关节1 - 加速度'); grid on;
subplot(3, 2, 6);
plot(t, ddq_d_poly(2,:), 'r-', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('ddq_2 (rad/s²)'); title('关节2 - 加速度'); grid on;

sgtitle('五次多项式插值轨迹', 'FontSize', 14, 'FontWeight', 'bold');
saveas(gcf, fullfile(out_dir, 'fig2_poly_trajectory.png'));
fprintf('图2已保存: fig2_poly_trajectory.png\n');

%% ========== 保存轨迹数据 ==========
% 保存为.mat文件, 供Simulink和task4使用
save(fullfile(out_dir, 'trajectory_data.mat'), 't', 'dt', 'T', 'N', ...
     'q_d_sin', 'dq_d_sin', 'ddq_d_sin', ...
     'q_d_poly', 'dq_d_poly', 'ddq_d_poly');

fprintf('\n轨迹数据已保存到 output/trajectory_data.mat\n');
fprintf('任务二完成!\n');
