% TASK3_CREATE_SIMULINK  创建二连杆机械臂PID/计算力矩控制Simulink模型
%
% 任务三: 搭建Simulink控制模型
%   - 期望轨迹生成子系统
%   - PID控制器子系统
%   - 计算力矩控制器子系统
%   - 被控对象 (二连杆动力学)
%   - Manual Switch切换控制器
%   - 观测与记录
%
% 运行后生成 two_link_control.slx

clear; clc;

fprintf('========================================\n');
fprintf('任务三: 创建Simulink控制模型\n');
fprintf('========================================\n\n');

% 输出目录
script_dir = fileparts(mfilename('fullpath'));
out_dir = fullfile(script_dir, 'output');
if ~exist(out_dir, 'dir'), mkdir(out_dir); end

model = 'two_link_control';

% 清理已有模型
if bdIsLoaded(model)
    close_system(model, 0);
end
if exist([model '.slx'], 'file')
    delete([model '.slx']);
end

new_system(model);
open_system(model);

%% ========== 模型参数 ==========
set_param(model, 'StopTime', '10');
set_param(model, 'Solver', 'ode45');
set_param(model, 'MaxStep', '0.01');
set_param(model, 'RelTol', '1e-4');

%% ================================================================
%% 1. 期望轨迹生成子系统 (Trajectory_Generator)
%% ================================================================
% 输入: Clock
% 输出: q_d (2x1), dq_d (2x1), ddq_d (2x1)

add_block('simulink/Sources/Clock', [model '/Clock'], ...
    'Position', [30, 80, 60, 110]);

% 用MATLAB Function生成全部轨迹信号
add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [model '/Trajectory_Gen'], ...
    'Position', [120, 60, 250, 130]);

% 写入MATLAB Function代码
gen_code = strcat( ...
    'function [q_d, dq_d, ddq_d] = gen_trajectory(t)\n', ...
    '  q_d = [sin(2*t); 0.5*sin(3*t)];\n', ...
    '  dq_d = [2*cos(2*t); 1.5*cos(3*t)];\n', ...
    '  ddq_d = [-4*sin(2*t); -4.5*sin(3*t)];\n', ...
    'end');
set_param([model '/Trajectory_Gen'], 'Function', gen_code);

% Demux: 分离q_d为两个标量 (供误差计算用)
add_block('simulink/Signal Routing/Mux', [model '/Mux_qd'], ...
    'Position', [300, 65, 305, 125]);
set_param([model '/Mux_qd'], 'Inputs', '3');

% 连线
add_line(model, 'Clock/1', 'Trajectory_Gen/1');
add_line(model, 'Trajectory_Gen/1', 'Mux_qd/1');

%% ================================================================
%% 2. PID控制器子系统
%% ================================================================
% 输入: q_d, dq_d, q, dq
% 输出: tau_pid

% 创建子系统
add_block('simulink/Ports & Subsystems/Subsystem', [model '/PID_Controller'], ...
    'Position', [380, 200, 480, 300]);

% 清理子系统默认内容
delete_line([model '/PID_Controller'], 'In1/1', 'Out1/1');
delete_block([model '/PID_Controller/In1']);
delete_block([model '/PID_Controller/Out1']);

% 添加输入输出端口
add_block('simulink/Sources/In1', [model '/PID_Controller/q_d_in'], ...
    'Position', [30, 30, 60, 50]);
add_block('simulink/Sources/In1', [model '/PID_Controller/dq_d_in'], ...
    'Position', [30, 80, 60, 100]);
add_block('simulink/Sources/In1', [model '/PID_Controller/q_in'], ...
    'Position', [30, 130, 60, 150]);
add_block('simulink/Sources/In1', [model '/PID_Controller/dq_in'], ...
    'Position', [30, 180, 60, 200]);
add_block('simulink/Sinks/Out1', [model '/PID_Controller/tau_out'], ...
    'Position', [500, 95, 530, 115]);

% 误差计算: e = q_d - q
add_block('simulink/Math Operations/Sum', [model '/PID_Controller/Sum_e'], ...
    'Position', [120, 40, 150, 70]);
set_param([model '/PID_Controller/Sum_e'], 'Inputs', '|+-');

add_block('simulink/Math Operations/Sum', [model '/PID_Controller/Sum_de'], ...
    'Position', [120, 100, 150, 130]);
set_param([model '/PID_Controller/Sum_de'], 'Inputs', '|+-');

% PID增益
add_block('simulink/Math Operations/Gain', [model '/PID_Controller/Kp'], ...
    'Position', [200, 35, 250, 65]);
set_param([model '/PID_Controller/Kp'], 'Gain', '[800; 600]');

add_block('simulink/Math Operations/Gain', [model '/PID_Controller/Kd'], ...
    'Position', [200, 95, 250, 125]);
set_param([model '/PID_Controller/Kd'], 'Gain', '[100; 80]');

% 积分项
add_block('simulink/Continuous/Integrator', [model '/PID_Controller/Int_I'], ...
    'Position', [200, 155, 250, 185]);
set_param([model '/PID_Controller/Int_I'], 'UpperSaturationLimit', '10', ...
    'LowerSaturationLimit', '-10');

add_block('simulink/Math Operations/Gain', [model '/PID_Controller/Ki'], ...
    'Position', [290, 155, 340, 185]);
set_param([model '/PID_Controller/Ki'], 'Gain', '[50; 30]');

% 求和: tau = Kp*e + Kd*de + Ki*integral
add_block('simulink/Math Operations/Sum', [model '/PID_Controller/Sum_pid'], ...
    'Position', [400, 80, 440, 130]);
set_param([model '/PID_Controller/Sum_pid'], 'Inputs', '+++');

% PID子系统内连线
add_line([model '/PID_Controller'], 'q_d_in/1', 'Sum_e/1');
add_line([model '/PID_Controller'], 'q_in/1', 'Sum_e/2');
add_line([model '/PID_Controller'], 'dq_d_in/1', 'Sum_de/1');
add_line([model '/PID_Controller'], 'dq_in/1', 'Sum_de/2');
add_line([model '/PID_Controller'], 'Sum_e/1', 'Kp/1');
add_line([model '/PID_Controller'], 'Sum_de/1', 'Kd/1');
add_line([model '/PID_Controller'], 'Sum_e/1', 'Int_I/1');
add_line([model '/PID_Controller'], 'Int_I/1', 'Ki/1');
add_line([model '/PID_Controller'], 'Kp/1', 'Sum_pid/1');
add_line([model '/PID_Controller'], 'Kd/1', 'Sum_pid/2');
add_line([model '/PID_Controller'], 'Ki/1', 'Sum_pid/3');
add_line([model '/PID_Controller'], 'Sum_pid/1', 'tau_out/1');

%% ================================================================
%% 3. 计算力矩控制器子系统
%% ================================================================
% 输入: q_d, dq_d, ddq_d, q, dq
% 输出: tau_ct

add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [model '/Computed_Torque_Controller'], ...
    'Position', [380, 350, 550, 430]);

ct_code = strcat( ...
    'function tau = ct_control(q_d, dq_d, ddq_d, q, dq)\n', ...
    '  l1=1.0; l2=0.8; m1=10; m2=5; I1=0.83; I2=0.21; g=9.81;\n', ...
    '  c2=cos(q(2)); s2=sin(q(2));\n', ...
    '  M11=I1+I2+m1*l1^2/4+m2*(l1^2+l2^2/4+l1*l2*c2);\n', ...
    '  M12=I2+m2*(l2^2/4+l1*l2*c2/2);\n', ...
    '  M22=I2+m2*l2^2/4;\n', ...
    '  M=[M11,M12;M12,M22];\n', ...
    '  h=-m2*l1*l2*s2/2;\n', ...
    '  C=[h*dq(2)*(2*dq(1)+dq(2)); h*dq(1)^2];\n', ...
    '  G=[g*cos(q(1))*(m1*l1/2+m2*l1)+g*m2*l2*cos(q(1)+q(2))/2; g*m2*l2*cos(q(1)+q(2))/2];\n', ...
    '  e=q_d-q; de=dq_d-dq;\n', ...
    '  Kp=[100;80]; Kd=[30;25];\n', ...
    '  tau = M*(ddq_d+Kp.*e+Kd.*de)+C+G;\n', ...
    'end');
set_param([model '/Computed_Torque_Controller'], 'Function', ct_code);

%% ================================================================
%% 4. 控制器切换开关
%% ================================================================
add_block('simulink/Signal Routing/Manual Switch', [model '/Controller_Switch'], ...
    'Position', [620, 275, 660, 315]);

% 默认选择计算力矩控制
set_param([model '/Controller_Switch'], 'sw', '1');

%% ================================================================
%% 5. 被控对象 (Plant) - 二连杆动力学
%% ================================================================
% 输入: tau (2x1)
% 输出: ddq (2x1)

add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [model '/Robot_Dynamics'], ...
    'Position', [720, 270, 860, 340]);

plant_code = strcat( ...
    'function ddq = robot_plant(tau, q, dq)\n', ...
    '  l1=1.0; l2=0.8; m1=10; m2=5; I1=0.83; I2=0.21; g=9.81;\n', ...
    '  c2=cos(q(2)); s2=sin(q(2));\n', ...
    '  M11=I1+I2+m1*l1^2/4+m2*(l1^2+l2^2/4+l1*l2*c2);\n', ...
    '  M12=I2+m2*(l2^2/4+l1*l2*c2/2);\n', ...
    '  M22=I2+m2*l2^2/4;\n', ...
    '  M=[M11,M12;M12,M22];\n', ...
    '  h=-m2*l1*l2*s2/2;\n', ...
    '  C=[h*dq(2)*(2*dq(1)+dq(2)); h*dq(1)^2];\n', ...
    '  G=[g*cos(q(1))*(m1*l1/2+m2*l1)+g*m2*l2*cos(q(1)+q(2))/2; g*m2*l2*cos(q(1)+q(2))/2];\n', ...
    '  ddq = M \ (tau - C - G);\n', ...
    'end');
set_param([model '/Robot_Dynamics'], 'Function', plant_code);

%% ================================================================
%% 6. 积分器: ddq -> dq -> q
%% ================================================================
% 初始条件: 稍微偏离期望轨迹
q0_init = '[-0.1; -0.05]';   % 初始角度偏移
dq0_init = '[0; 0]';          % 初始角速度

add_block('simulink/Continuous/Integrator', [model '/Integrator_dq'], ...
    'Position', [920, 275, 960, 315]);
set_param([model '/Integrator_dq'], 'InitialCondition', dq0_init);

add_block('simulink/Continuous/Integrator', [model '/Integrator_q'], ...
    'Position', [1020, 275, 1060, 315]);
set_param([model '/Integrator_q'], 'InitialCondition', q0_init);

%% ================================================================
%% 7. 反馈信号分离与连接
%% ================================================================
% 从Trajectory_Gen获取q_d, dq_d, ddq_d
% 需要Demux分离轨迹信号

% 期望轨迹通过Goto/From传递
add_block('simulink/Signal Routing/Goto', [model '/Goto_qd'], ...
    'Position', [320, 55, 380, 75]);
set_param([model '/Goto_qd'], 'GotoTag', 'qd_tag');

add_block('simulink/Signal Routing/Goto', [model '/Goto_dqd'], ...
    'Position', [320, 85, 380, 105]);
set_param([model '/Goto_dqd'], 'GotoTag', 'dqd_tag');

add_block('simulink/Signal Routing/Goto', [model '/Goto_ddqd'], ...
    'Position', [320, 115, 380, 135]);
set_param([model '/Goto_ddqd'], 'GotoTag', 'ddqd_tag');

% 从Mux_qd分出三个信号
% 实际上Trajectory_Gen输出三个端口, 直接连接
add_line(model, 'Trajectory_Gen/1', 'Goto_qd/1');
add_line(model, 'Trajectory_Gen/2', 'Goto_dqd/1');
add_line(model, 'Trajectory_Gen/3', 'Goto_ddqd/1');

% PID控制器输入
add_block('simulink/Signal Routing/From', [model '/From_qd_pid'], ...
    'Position', [300, 205, 360, 225]);
set_param([model '/From_qd_pid'], 'GotoTag', 'qd_tag');

add_block('simulink/Signal Routing/From', [model '/From_dqd_pid'], ...
    'Position', [300, 235, 360, 255]);
set_param([model '/From_dqd_pid'], 'GotoTag', 'dqd_tag');

add_line(model, 'From_qd_pid/1', 'PID_Controller/1');
add_line(model, 'From_dqd_pid/1', 'PID_Controller/2');

% 计算力矩控制器输入
add_block('simulink/Signal Routing/From', [model '/From_qd_ct'], ...
    'Position', [290, 355, 360, 375]);
set_param([model '/From_qd_ct'], 'GotoTag', 'qd_tag');

add_block('simulink/Signal Routing/From', [model '/From_dqd_ct'], ...
    'Position', [290, 385, 360, 405]);
set_param([model '/From_dqd_ct'], 'GotoTag', 'dqd_tag');

add_block('simulink/Signal Routing/From', [model '/From_ddqd_ct'], ...
    'Position', [290, 415, 360, 435]);
set_param([model '/From_ddqd_ct'], 'GotoTag', 'ddqd_tag');

add_line(model, 'From_qd_ct/1', 'Computed_Torque_Controller/1');
add_line(model, 'From_dqd_ct/1', 'Computed_Torque_Controller/2');
add_line(model, 'From_ddqd_ct/1', 'Computed_Torque_Controller/3');

% 开关连线
add_line(model, 'PID_Controller/1', 'Controller_Switch/1');
add_line(model, 'Computed_Torque_Controller/1', 'Controller_Switch/2');

% Plant连线
add_line(model, 'Controller_Switch/1', 'Robot_Dynamics/1');

% 反馈q, dq到Plant和控制器
add_block('simulink/Signal Routing/Goto', [model '/Goto_q'], ...
    'Position', [1090, 270, 1150, 290]);
set_param([model '/Goto_q'], 'GotoTag', 'q_tag');

add_block('simulink/Signal Routing/Goto', [model '/Goto_dq'], ...
    'Position', [1090, 300, 1150, 320]);
set_param([model '/Goto_dq'], 'GotoTag', 'dq_tag');

% 积分器连线
add_line(model, 'Robot_Dynamics/1', 'Integrator_dq/1');
add_line(model, 'Integrator_dq/1', 'Integrator_q/1');
add_line(model, 'Integrator_q/1', 'Goto_q/1');
add_line(model, 'Integrator_dq/1', 'Goto_dq/1');

% 反馈到Plant
add_block('simulink/Signal Routing/From', [model '/From_q_plant'], ...
    'Position', [680, 310, 730, 330]);
set_param([model '/From_q_plant'], 'GotoTag', 'q_tag');

add_block('simulink/Signal Routing/From', [model '/From_dq_plant'], ...
    'Position', [680, 340, 730, 360]);
set_param([model '/From_dq_plant'], 'GotoTag', 'dq_tag');

add_line(model, 'From_q_plant/1', 'Robot_Dynamics/2');
add_line(model, 'From_dq_plant/1', 'Robot_Dynamics/3');

% 反馈到PID控制器
add_block('simulink/Signal Routing/From', [model '/From_q_pid'], ...
    'Position', [300, 265, 360, 285]);
set_param([model '/From_q_pid'], 'GotoTag', 'q_tag');

add_block('simulink/Signal Routing/From', [model '/From_dq_pid'], ...
    'Position', [300, 295, 360, 315]);
set_param([model '/From_dq_pid'], 'GotoTag', 'dq_tag');

add_line(model, 'From_q_pid/1', 'PID_Controller/3');
add_line(model, 'From_dq_pid/1', 'PID_Controller/4');

% 反馈到计算力矩控制器
add_block('simulink/Signal Routing/From', [model '/From_q_ct'], ...
    'Position', [290, 440, 360, 460]);
set_param([model '/From_q_ct'], 'GotoTag', 'q_tag');

add_block('simulink/Signal Routing/From', [model '/From_dq_ct'], ...
    'Position', [290, 465, 360, 485]);
set_param([model '/From_dq_ct'], 'GotoTag', 'dq_tag');

add_line(model, 'From_q_ct/1', 'Computed_Torque_Controller/4');
add_line(model, 'From_dq_ct/1', 'Computed_Torque_Controller/5');

%% ================================================================
%% 8. 观测与记录
%% ================================================================
% Scope
add_block('simulink/Sinks/Scope', [model '/Scope'], ...
    'Position', [1200, 260, 1240, 320]);
set_param([model '/Scope'], 'NumInputPorts', '4');

% To Workspace
add_block('simulink/Sinks/To Workspace', [model '/Log_q'], ...
    'Position', [1200, 350, 1260, 380]);
set_param([model '/Log_q'], 'VariableName', 'q_log');
set_param([model '/Log_q'], 'SaveFormat', 'Array');

add_block('simulink/Sinks/To Workspace', [model '/Log_tau'], ...
    'Position', [1200, 400, 1260, 430]);
set_param([model '/Log_tau'], 'VariableName', 'tau_log');
set_param([model '/Log_tau'], 'SaveFormat', 'Array');

% 信号到Scope和Log
add_block('simulink/Signal Routing/From', [model '/From_q_scope'], ...
    'Position', [1150, 265, 1190, 285]);
set_param([model '/From_q_scope'], 'GotoTag', 'q_tag');

add_block('simulink/Signal Routing/From', [model '/From_qd_scope'], ...
    'Position', [1150, 295, 1190, 315]);
set_param([model '/From_qd_scope'], 'GotoTag', 'qd_tag');

add_line(model, 'From_q_scope/1', 'Scope/1');
add_line(model, 'From_qd_scope/1', 'Scope/2');
add_line(model, 'From_q_scope/1', 'Log_q/1');

% 控制力矩记录
add_block('simulink/Signal Routing/Goto', [model '/Goto_tau'], ...
    'Position', [680, 270, 730, 290]);
set_param([model '/Goto_tau'], 'GotoTag', 'tau_tag');

% 需要一个分支点
% 从Controller_Switch输出分叉
add_line(model, 'Controller_Switch/1', 'Goto_tau/1', 'autorouting', 'smart');

add_block('simulink/Signal Routing/From', [model '/From_tau_scope'], ...
    'Position', [1150, 325, 1190, 345]);
set_param([model '/From_tau_scope'], 'GotoTag', 'tau_tag');

add_block('simulink/Signal Routing/From', [model '/From_tau_log'], ...
    'Position', [1150, 410, 1190, 430]);
set_param([model '/From_tau_log'], 'GotoTag', 'tau_tag');

add_line(model, 'From_tau_scope/1', 'Scope/3');
add_line(model, 'From_tau_log/1', 'Log_tau/1');

%% ================================================================
%% 9. 保存模型
%% ================================================================
slx_path = fullfile(out_dir, [model '.slx']);
save_system(model, slx_path);
% close_system(model);  % 保留打开状态供查看

fprintf('Simulink模型已保存到 output/%s.slx\n', model);
fprintf('\n模型说明:\n');
fprintf('  - 双击 Controller_Switch 可切换 PID / 计算力矩控制\n');
fprintf('  - 默认使用计算力矩控制\n');
fprintf('  - 点击 Run 运行仿真\n');
fprintf('  - Scope 显示: q(实际), q_d(期望), tau(力矩)\n');
fprintf('  - 仿真结束后工作区有 q_log, tau_log 数据\n');
fprintf('\n任务三完成!\n');
