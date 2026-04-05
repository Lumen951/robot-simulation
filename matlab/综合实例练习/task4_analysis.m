% TASK4_ANALYSIS  运行PID和计算力矩控制仿真, 对比分析结果
%
% 任务四: 运行仿真并分析结果
%   - 分别运行PID和计算力矩控制
%   - 绘制轨迹跟踪、误差、力矩曲线
%   - 末端执行器笛卡尔轨迹
%   - 性能统计与对比结论
%
% 依赖: two_link_dynamics.m, pid_controller.m, computed_torque_controller.m
%   (位于 matlab/计算力矩法实验/ 目录)

clear; clc; close all;

% 添加依赖路径
script_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(script_dir, '..', '计算力矩法实验'));

% 输出目录
out_dir = fullfile(script_dir, 'output');
if ~exist(out_dir, 'dir'), mkdir(out_dir); end

%% ========== 1. 系统参数 ==========
fprintf('========================================\n');
fprintf('任务四: 仿真运行与结果分析\n');
fprintf('========================================\n\n');

params.l1 = 1.0;  params.l2 = 0.8;
params.m1 = 10.0;  params.m2 = 5.0;
params.I1 = 0.83;  params.I2 = 0.21;
params.g = 9.81;

fprintf('参数: l1=%.1f, l2=%.1f, m1=%.0f, m2=%.0f\n', ...
    params.l1, params.l2, params.m1, params.m2);

%% ========== 2. 仿真设置 ==========
T = 10;  dt = 0.01;
t = 0:dt:T;  N = length(t);

% 期望轨迹 (正弦)
q_d = [sin(2*t); 0.5*sin(3*t)];
dq_d = [2*cos(2*t); 1.5*cos(3*t)];
ddq_d = [-4*sin(2*t); -4.5*sin(3*t)];

%% ========== 3. PID控制仿真 ==========
fprintf('\n运行PID控制仿真...\n');

q_pid = zeros(2, N);  dq_pid = zeros(2, N);  tau_pid = zeros(2, N);
q_pid(:, 1) = q_d(:, 1) * 0.9;   % 初始偏移
dq_pid(:, 1) = dq_d(:, 1) * 0.9;

kp = [800; 600]; ki = [50; 30]; kd = [100; 80];
integral_e = zeros(2, 1);

for i = 1:N-1
    [tau, integral_e] = pid_controller(q_d(:,i), dq_d(:,i), ...
        q_pid(:,i), dq_pid(:,i), kp, ki, kd, integral_e, dt);
    tau_pid(:, i) = tau;

    [ddq, ~] = two_link_dynamics(tau, q_pid(:,i), dq_pid(:,i), params);
    dq_pid(:, i+1) = dq_pid(:, i) + ddq * dt;
    q_pid(:, i+1) = q_pid(:, i) + dq_pid(:, i) * dt;
end
[tau_pid(:, N), ~] = pid_controller(q_d(:,N), dq_d(:,N), ...
    q_pid(:,N), dq_pid(:,N), kp, ki, kd, integral_e, dt);

error_pid = q_d - q_pid;
fprintf('PID仿真完成。\n');

%% ========== 4. 计算力矩控制仿真 ==========
fprintf('运行计算力矩控制仿真...\n');

q_ct = zeros(2, N);  dq_ct = zeros(2, N);  tau_ct = zeros(2, N);
q_ct(:, 1) = q_d(:, 1) * 0.9;
dq_ct(:, 1) = dq_d(:, 1) * 0.9;

Kp_ct = [100; 80];  Kd_ct = [30; 25];

for i = 1:N-1
    [tau, ~] = computed_torque_controller(q_d(:,i), dq_d(:,i), ddq_d(:,i), ...
        q_ct(:,i), dq_ct(:,i), params, Kp_ct, Kd_ct);
    tau_ct(:, i) = tau;

    [ddq, ~] = two_link_dynamics(tau, q_ct(:,i), dq_ct(:,i), params);
    dq_ct(:, i+1) = dq_ct(:, i) + ddq * dt;
    q_ct(:, i+1) = q_ct(:, i) + dq_ct(:, i) * dt;
end
[tau_ct(:, N), ~] = computed_torque_controller(q_d(:,N), dq_d(:,N), ddq_d(:,N), ...
    q_ct(:,N), dq_ct(:,N), params, Kp_ct, Kd_ct);

error_ct = q_d - q_ct;
fprintf('计算力矩仿真完成。\n');

%% ========== 5. 性能统计 ==========
fprintf('\n========================================\n');
fprintf('性能统计\n');
fprintf('========================================\n');

labels = {'关节1', '关节2'};
for j = 1:2
    fprintf('\n%s:\n', labels{j});
    fprintf('  PID:       RMS=%.6f rad, 最大=%.6f rad\n', ...
        rms(error_pid(j,:)), max(abs(error_pid(j,:))));
    fprintf('  计算力矩:  RMS=%.6f rad, 最大=%.6f rad\n', ...
        rms(error_ct(j,:)), max(abs(error_ct(j,:))));
    fprintf('  改善率:    %.1f%%\n', ...
        (1 - rms(error_ct(j,:))/rms(error_pid(j,:))) * 100);
end

% 稳态误差 (最后2秒)
ss_idx = find(t >= T-2, 1);
fprintf('\n稳态误差 (最后2秒均值):\n');
for j = 1:2
    fprintf('  %s: PID=%.6f, 计算力矩=%.6f\n', labels{j}, ...
        mean(abs(error_pid(j, ss_idx:end))), mean(abs(error_ct(j, ss_idx:end))));
end

%% ========== 6. 图1: 轨迹跟踪对比 ==========
figure('Name', '轨迹跟踪对比', 'Position', [50, 50, 1400, 900], 'Color', 'w');

% 关节1轨迹
subplot(2, 3, 1); hold on;
plot(t, q_d(1,:), 'k--', 'LineWidth', 1.5, 'DisplayName', '期望');
plot(t, q_pid(1,:), 'b-', 'LineWidth', 1, 'DisplayName', 'PID');
plot(t, q_ct(1,:), 'r-', 'LineWidth', 1, 'DisplayName', '计算力矩');
xlabel('时间 (s)'); ylabel('q_1 (rad)'); title('关节1 轨迹跟踪');
legend('Location', 'best'); grid on;

% 关节2轨迹
subplot(2, 3, 2); hold on;
plot(t, q_d(2,:), 'k--', 'LineWidth', 1.5, 'DisplayName', '期望');
plot(t, q_pid(2,:), 'b-', 'LineWidth', 1, 'DisplayName', 'PID');
plot(t, q_ct(2,:), 'r-', 'LineWidth', 1, 'DisplayName', '计算力矩');
xlabel('时间 (s)'); ylabel('q_2 (rad)'); title('关节2 轨迹跟踪');
legend('Location', 'best'); grid on;

% 关节1跟踪误差
subplot(2, 3, 4); hold on;
plot(t, error_pid(1,:), 'b-', 'LineWidth', 1.5, 'DisplayName', 'PID');
plot(t, error_ct(1,:), 'r-', 'LineWidth', 1.5, 'DisplayName', '计算力矩');
xlabel('时间 (s)'); ylabel('误差 (rad)'); title('关节1 跟踪误差');
legend('Location', 'best'); grid on;

% 关节2跟踪误差
subplot(2, 3, 5); hold on;
plot(t, error_pid(2,:), 'b-', 'LineWidth', 1.5, 'DisplayName', 'PID');
plot(t, error_ct(2,:), 'r-', 'LineWidth', 1.5, 'DisplayName', '计算力矩');
xlabel('时间 (s)'); ylabel('误差 (rad)'); title('关节2 跟踪误差');
legend('Location', 'best'); grid on;

% 关节1控制力矩
subplot(2, 3, 3); hold on;
plot(t, tau_pid(1,:), 'b-', 'LineWidth', 1, 'DisplayName', 'PID');
plot(t, tau_ct(1,:), 'r-', 'LineWidth', 1, 'DisplayName', '计算力矩');
xlabel('时间 (s)'); ylabel('\tau_1 (Nm)'); title('关节1 控制力矩');
legend('Location', 'best'); grid on;

% 关节2控制力矩
subplot(2, 3, 6); hold on;
plot(t, tau_pid(2,:), 'b-', 'LineWidth', 1, 'DisplayName', 'PID');
plot(t, tau_ct(2,:), 'r-', 'LineWidth', 1, 'DisplayName', '计算力矩');
xlabel('时间 (s)'); ylabel('\tau_2 (Nm)'); title('关节2 控制力矩');
legend('Location', 'best'); grid on;

sgtitle('PID vs 计算力矩控制 - 性能对比', 'FontSize', 14, 'FontWeight', 'bold');

% 保存
saveas(gcf, fullfile(out_dir, 'fig4_pid_vs_ct_comparison.png'));
fprintf('\n图4-1已保存: fig4_pid_vs_ct_comparison.png\n');

%% ========== 7. 图2: 末端执行器笛卡尔轨迹 ==========
figure('Name', '末端轨迹', 'Position', [100, 100, 1000, 500], 'Color', 'w');

l1 = params.l1; l2 = params.l2;

% 计算笛卡尔位置 (约定: q=0竖直向上)
% x = -l1*sin(q1) - l2*sin(q1+q2)
% y =  l1*cos(q1) + l2*cos(q1+q2)

x_d = -l1*sin(q_d(1,:)) - l2*sin(q_d(1,:)+q_d(2,:));
y_d =  l1*cos(q_d(1,:)) + l2*cos(q_d(1,:)+q_d(2,:));

x_pid = -l1*sin(q_pid(1,:)) - l2*sin(q_pid(1,:)+q_pid(2,:));
y_pid =  l1*cos(q_pid(1,:)) + l2*cos(q_pid(1,:)+q_pid(2,:));

x_ct = -l1*sin(q_ct(1,:)) - l2*sin(q_ct(1,:)+q_ct(2,:));
y_ct =  l1*cos(q_ct(1,:)) + l2*cos(q_ct(1,:)+q_ct(2,:));

subplot(1, 2, 1); hold on;
plot(x_d, y_d, 'k--', 'LineWidth', 1.5, 'DisplayName', '期望');
plot(x_pid, y_pid, 'b-', 'LineWidth', 1, 'DisplayName', 'PID');
plot(x_ct, y_ct, 'r-', 'LineWidth', 1, 'DisplayName', '计算力矩');
xlabel('X (m)'); ylabel('Y (m)'); title('末端执行器轨迹');
legend('Location', 'best'); grid on; axis equal;

% 笛卡尔误差
pos_err_pid = sqrt((x_pid - x_d).^2 + (y_pid - y_d).^2);
pos_err_ct  = sqrt((x_ct  - x_d).^2 + (y_ct  - y_d).^2);

subplot(1, 2, 2); hold on;
plot(t, pos_err_pid*100, 'b-', 'LineWidth', 1.5, 'DisplayName', 'PID');
plot(t, pos_err_ct*100,  'r-', 'LineWidth', 1.5, 'DisplayName', '计算力矩');
xlabel('时间 (s)'); ylabel('位置误差 (cm)'); title('末端位置跟踪误差');
legend('Location', 'best'); grid on;

sgtitle('末端执行器笛卡尔轨迹对比', 'FontSize', 14, 'FontWeight', 'bold');

saveas(gcf, fullfile(out_dir, 'fig4_cartesian_trajectory.png'));
fprintf('图4-2已保存: fig4_cartesian_trajectory.png\n');

%% ========== 8. 图3: 误差统计分析 ==========
figure('Name', '误差统计', 'Position', [150, 150, 800, 400], 'Color', 'w');

% 误差箱线图
subplot(1, 2, 1);
boxplot([error_pid(1,:)', error_ct(1,:)', error_pid(2,:)', error_ct(2,:)'], ...
    'Labels', {'PID-J1', 'CT-J1', 'PID-J2', 'CT-J2'});
ylabel('跟踪误差 (rad)'); title('跟踪误差分布');
grid on;

% 误差能量 (积分)
subplot(1, 2, 2);
E_pid_j1 = cumtrapz(t, error_pid(1,:).^2);
E_pid_j2 = cumtrapz(t, error_pid(2,:).^2);
E_ct_j1  = cumtrapz(t, error_ct(1,:).^2);
E_ct_j2  = cumtrapz(t, error_ct(2,:).^2);
hold on;
plot(t, E_pid_j1, 'b-', 'LineWidth', 1.5, 'DisplayName', 'PID-J1');
plot(t, E_ct_j1,  'r-', 'LineWidth', 1.5, 'DisplayName', 'CT-J1');
plot(t, E_pid_j2, 'b--', 'LineWidth', 1.5, 'DisplayName', 'PID-J2');
plot(t, E_ct_j2,  'r--', 'LineWidth', 1.5, 'DisplayName', 'CT-J2');
xlabel('时间 (s)'); ylabel('累积误差能量'); title('累积误差能量');
legend('Location', 'best'); grid on;

sgtitle('跟踪误差统计分析', 'FontSize', 14, 'FontWeight', 'bold');

saveas(gcf, fullfile(out_dir, 'fig4_error_statistics.png'));
fprintf('图4-3已保存: fig4_error_statistics.png\n');

%% ========== 9. 分析结论 ==========
fprintf('\n========================================\n');
fprintf('分析结论\n');
fprintf('========================================\n');
fprintf('\n1. 轨迹跟踪精度:\n');
fprintf('   计算力矩控制通过动力学补偿, 跟踪误差显著低于PID控制。\n');
fprintf('   关节1改善: %.1f%%, 关节2改善: %.1f%%\n', ...
    (1-rms(error_ct(1,:))/rms(error_pid(1,:)))*100, ...
    (1-rms(error_ct(2,:))/rms(error_pid(2,:)))*100);

fprintf('\n2. 响应速度:\n');
fprintf('   计算力矩控制的误差收敛更快, 稳态误差更小。\n');

fprintf('\n3. 控制力矩:\n');
fprintf('   计算力矩控制的前馈补偿使力矩更平稳。\n');
fprintf('   PID控制的力矩可能出现较大波动。\n');

fprintf('\n4. 总结:\n');
fprintf('   - PID控制: 实现简单, 不需要精确动力学模型, 但跟踪精度有限\n');
fprintf('   - 计算力矩控制: 需要精确动力学参数, 跟踪精度高, 响应快\n');
fprintf('   - 实际应用中, 计算力矩法+鲁棒补偿是更优选择\n');

% 保存数据
save(fullfile(out_dir, 'simulation_results.mat'), 't', 'q_d', 'q_pid', 'q_ct', ...
    'error_pid', 'error_ct', 'tau_pid', 'tau_ct');
fprintf('\n仿真数据已保存到 output/simulation_results.mat\n');
fprintf('任务四完成!\n');
