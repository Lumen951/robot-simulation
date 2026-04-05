% RUN_COMPARISON 对比PID和计算力矩控制器的轨迹跟踪性能
%
% 二连杆机械臂参数:
%   杆1: l1=1.0m, m1=10kg, I1=0.83 kg·m²
%   杆2: l2=0.8m, m2=5kg, I2=0.21 kg·m²

clear; clc; close all;

%% 1. 系统参数设置
params.l1 = 1.0;
params.l2 = 0.8;
params.m1 = 10.0;
params.m2 = 5.0;
params.I1 = 0.83;
params.I2 = 0.21;
params.g = 9.81;

fprintf('========================================\n');
fprintf('二连杆机械臂控制对比实验\n');
fprintf('========================================\n');
fprintf('杆1: l1=%.1fm, m1=%.1fkg, I1=%.2f kg·m²\n', params.l1, params.m1, params.I1);
fprintf('杆2: l2=%.1fm, m2=%.1fkg, I2=%.2f kg·m²\n', params.l2, params.m2, params.I2);
fprintf('========================================\n\n');

%% 2. 仿真参数
T = 10;              % 总时间 (s)
dt = 0.01;           % 采样时间 (s)
t = 0:dt:T;
N = length(t);

%% 3. 期望轨迹设计
% 使用不同频率的正弦波
q_d = zeros(2, N);
dq_d = zeros(2, N);
ddq_d = zeros(2, N);

% 关节1: q1_d = sin(2*t)
q_d(1, :) = sin(2*t);
dq_d(1, :) = 2*cos(2*t);
ddq_d(1, :) = -4*sin(2*t);

% 关节2: q2_d = 0.5*sin(3*t)
q_d(2, :) = 0.5*sin(3*t);
dq_d(2, :) = 1.5*cos(3*t);
ddq_d(2, :) = -4.5*sin(3*t);

%% 4. PID控制仿真
fprintf('运行PID控制器仿真...\n');

% 初始状态
q_pid = zeros(2, N);
dq_pid = zeros(2, N);
tau_pid = zeros(2, N);

% 初始条件
q_pid(:, 1) = q_d(:, 1) * 0.9;  % 稍微偏离期望
dq_pid(:, 1) = dq_d(:, 1) * 0.9;

% PID增益
kp_pid = [800; 600];
ki_pid = [50; 30];
kd_pid = [100; 80];

integral_e = zeros(2, 1);

for i = 1:N-1
    % PID控制器
    [tau, integral_e] = pid_controller(...
        q_d(:, i), dq_d(:, i), ...
        q_pid(:, i), dq_pid(:, i), ...
        kp_pid, ki_pid, kd_pid, integral_e, dt);

    tau_pid(:, i) = tau;

    % 动力学积分 (Euler法)
    [ddq, ~] = two_link_dynamics(tau, q_pid(:, i), dq_pid(:, i), params);

    q_pid(:, i+1) = q_pid(:, i) + dq_pid(:, i) * dt;
    dq_pid(:, i+1) = dq_pid(:, i) + ddq * dt;
end

% 最后一步
[tau_pid(:, N), ~] = pid_controller(...
    q_d(:, N), dq_d(:, N), ...
    q_pid(:, N), dq_pid(:, N), ...
    kp_pid, ki_pid, kd_pid, integral_e, dt);

error_pid = q_d - q_pid;

fprintf('PID仿真完成。\n');

%% 5. 计算力矩控制仿真
fprintf('运行计算力矩控制器仿真...\n');

% 初始状态
q_ct = zeros(2, N);
dq_ct = zeros(2, N);
tau_ct = zeros(2, N);

% 初始条件 (与PID相同)
q_ct(:, 1) = q_d(:, 1) * 0.9;
dq_ct(:, 1) = dq_d(:, 1) * 0.9;

% PD增益 (外环)
Kp_ct = [100; 80];
Kd_ct = [30; 25];

for i = 1:N-1
    % 计算力矩控制器
    [tau, ~] = computed_torque_controller(...
        q_d(:, i), dq_d(:, i), ddq_d(:, i), ...
        q_ct(:, i), dq_ct(:, i), ...
        params, Kp_ct, Kd_ct);

    tau_ct(:, i) = tau;

    % 动力学积分
    [ddq, ~] = two_link_dynamics(tau, q_ct(:, i), dq_ct(:, i), params);

    q_ct(:, i+1) = q_ct(:, i) + dq_ct(:, i) * dt;
    dq_ct(:, i+1) = dq_ct(:, i) + ddq * dt;
end

% 最后一步
[tau_ct(:, N), ~] = computed_torque_controller(...
    q_d(:, N), dq_d(:, N), ddq_d(:, N), ...
    q_ct(:, N), dq_ct(:, N), ...
    params, Kp_ct, Kd_ct);

error_ct = q_d - q_ct;

fprintf('计算力矩仿真完成。\n\n');

%% 6. 性能统计
fprintf('========================================\n');
fprintf('轨迹跟踪误差统计 (RMS)\n');
fprintf('========================================\n');
fprintf('关节1:\n');
fprintf('  PID:         %.6f rad\n', rms(error_pid(1, :)));
fprintf('  计算力矩:    %.6f rad\n', rms(error_ct(1, :)));
fprintf('  改善:        %.1f%%\n', ...
    (1 - rms(error_ct(1,:))/rms(error_pid(1,:))) * 100);

fprintf('关节2:\n');
fprintf('  PID:         %.6f rad\n', rms(error_pid(2, :)));
fprintf('  计算力矩:    %.6f rad\n', rms(error_ct(2, :)));
fprintf('  改善:        %.1f%%\n', ...
    (1 - rms(error_ct(2,:))/rms(error_pid(2,:))) * 100);
fprintf('========================================\n\n');

%% 7. 绘制对比图
figure('Position', [100, 100, 1400, 900], 'Color', 'w');

% 子图1: 关节1轨迹跟踪
subplot(2, 3, 1); hold on;
plot(t, q_d(1, :), 'k--', 'LineWidth', 1.5, 'DisplayName', '期望');
plot(t, q_pid(1, :), 'b-', 'LineWidth', 1, 'DisplayName', 'PID');
plot(t, q_ct(1, :), 'r-', 'LineWidth', 1, 'DisplayName', '计算力矩');
xlabel('时间 (s)');
ylabel('关节1角度 (rad)');
title('关节1轨迹跟踪');
legend('Location', 'best');
grid on;

% 子图2: 关节2轨迹跟踪
subplot(2, 3, 2); hold on;
plot(t, q_d(2, :), 'k--', 'LineWidth', 1.5, 'DisplayName', '期望');
plot(t, q_pid(2, :), 'b-', 'LineWidth', 1, 'DisplayName', 'PID');
plot(t, q_ct(2, :), 'r-', 'LineWidth', 1, 'DisplayName', '计算力矩');
xlabel('时间 (s)');
ylabel('关节2角度 (rad)');
title('关节2轨迹跟踪');
legend('Location', 'best');
grid on;

% 子图3: 关节1跟踪误差
subplot(2, 3, 4); hold on;
plot(t, error_pid(1, :), 'b-', 'LineWidth', 1.5, 'DisplayName', 'PID');
plot(t, error_ct(1, :), 'r-', 'LineWidth', 1.5, 'DisplayName', '计算力矩');
xlabel('时间 (s)');
ylabel('误差 (rad)');
title('关节1跟踪误差');
legend('Location', 'best');
grid on;

% 子图4: 关节2跟踪误差
subplot(2, 3, 5); hold on;
plot(t, error_pid(2, :), 'b-', 'LineWidth', 1.5, 'DisplayName', 'PID');
plot(t, error_ct(2, :), 'r-', 'LineWidth', 1.5, 'DisplayName', '计算力矩');
xlabel('时间 (s)');
ylabel('误差 (rad)');
title('关节2跟踪误差');
legend('Location', 'best');
grid on;

% 子图5: 控制力矩对比 (关节1)
subplot(2, 3, 3); hold on;
plot(t, tau_pid(1, :), 'b-', 'LineWidth', 1, 'DisplayName', 'PID');
plot(t, tau_ct(1, :), 'r-', 'LineWidth', 1, 'DisplayName', '计算力矩');
xlabel('时间 (s)');
ylabel('力矩 (Nm)');
title('关节1控制力矩');
legend('Location', 'best');
grid on;

% 子图6: 控制力矩对比 (关节2)
subplot(2, 3, 6); hold on;
plot(t, tau_pid(2, :), 'b-', 'LineWidth', 1, 'DisplayName', 'PID');
plot(t, tau_ct(2, :), 'r-', 'LineWidth', 1, 'DisplayName', '计算力矩');
xlabel('时间 (s)');
ylabel('力矩 (Nm)');
title('关节2控制力矩');
legend('Location', 'best');
grid on;

sgtitle('PID vs 计算力矩控制器 - 性能对比', 'FontSize', 14, 'FontWeight', 'bold');

%% 8. 保存图像
saveas(gcf, 'pid_vs_computed_torque_comparison.png');
fprintf('图像已保存: pid_vs_computed_torque_comparison.png\n');

fprintf('\n仿真完成！\n');
