% CREATE_SIMULINK_MODEL 创建二连杆计算力矩控制Simulink模型
% 运行此脚本将自动生成sim.slx文件
%
% 使用方法:
%   1. 在MATLAB中切换到matlab/目录
%   2. 运行: create_simulink_model
%   3. 将生成sim.slx文件

clear; clc;

model_name = 'sim';
new_system(model_name);
load_system(model_name);

%% 清空默认内容
delete_block([model_name '/Untitled']);

%% 添加模块位置配置
% 设置画布大小
% Simulink.BlockDiagram.setConfigParameter(model_name, 'ScreenUpdate', 'on');

%% 1. 期望轨迹生成
add_block('simulink/Sources/Clock', [model_name '/Clock'], ...
    'Position', [50, 50, 80, 80]);
set_param([model_name '/Clock'], 'DisplayTime', 'off');

% sin(2*t) for q1_d
add_block('simulink/User-Defined Functions/MATLAB Fcn', [model_name '/q1_d_gen'], ...
    'Position', [150, 30, 250, 60]);
set_param([model_name '/q1_d_gen'], 'MATLABFcn', 'sin(2*u)*[1; 0.25]', ...
    'OutputDimensions', '2');

% q_d signal
add_block('simulink/Sources/Constant', [model_name '/q_d_const'], ...
    'Position', [300, 40, 330, 70]);

%% 2. 积分器 (获取 q, dq)
add_block('simulink/Continuous/Integrator', [model_name '/Integrator_q'], ...
    'Position', [400, 40, 430, 70]);
add_block('simulink/Continuous/Integrator', [model_name '/Integrator_dq'], ...
    'Position', [400, 100, 430, 130]);

%% 3. 机器人Plant (MATLAB Function)
add_block('simulink/User-Defined Functions/MATLAB Function', [model_name '/RobotPlant'], ...
    'Position', [480, 50, 560, 100]);

% 设置MATLAB Function代码
plant_code = ['function [dq, ddq] = RobotPlant(tau, q, dq)\n', ...
              '% 二连杆动力学\n', ...
              'params.l1 = 1.0; params.l2 = 0.8;\n', ...
              'params.m1 = 10.0; params.m2 = 5.0;\n', ...
              'params.I1 = 0.83; params.I2 = 0.21;\n', ...
              'params.g = 9.81;\n', ...
              '[ddq, ~] = two_link_dynamics(tau, q, dq, params);\n', ...
              'dq = dq(:);'''];
set_param([model_name '/RobotPlant'], 'Function', plant_code);

%% 4. 示波器
add_block('simulink/Sinks/Scope', [model_name '/Scope'], ...
    'Position', [600, 40, 630, 70]);
set_param([model_name '/Scope'], 'NumInputPorts', '4');

%% 5. 期望轨迹 (直接用Constant输入简化)
delete_block([model_name '/Clock']);
delete_block([model_name '/q1_d_gen']);

% 简化版本: 使用Pulse Generator产生测试信号
add_block('simulink/Sources/Sine Wave', [model_name '/Sine_q1'], ...
    'Position', [50, 40, 100, 70]);
set_param([model_name '/Sine_q1'], 'Amplitude', '1', 'Frequency', '2', 'Bias', '0');

add_block('simulink/Sources/Sine Wave', [model_name '/Sine_q2'], ...
    'Position', [50, 100, 100, 130]);
set_param([model_name '/Sine_q2'], 'Amplitude', '0.5', 'Frequency', '3', 'Bias', '0');

% 合并期望信号
add_block('simulink/Math Operations/Mux', [model_name '/Mux_q_d'], ...
    'Position', [150, 60, 180, 110]);
set_param([model_name '/Mux_q_d'], 'Inputs', '2');

% 连线
add_line(model_name, 'Sine_q1/1', 'Mux_q_d/1');
add_line(model_name, 'Sine_q2/1', 'Mux_q_d/2');

%% 6. 误差计算
add_block('simulink/Math Operations/Sum', [model_name '/Sum_error'], ...
    'Position', [220, 70, 250, 100]);
set_param([model_name '/Sum_error'], 'Inputs', '|+-');

% 需要反馈回路
add_line(model_name, 'Mux_q_d/1', 'Sum_error/1');
add_line(model_name, 'Sum_error/1', 'Integrator_dq/1');

%% 7. PID控制器 (简化Gain模块)
add_block('simulink/Math Operations/Gain', [model_name '/Gain_P'], ...
    'Position', [280, 60, 310, 90]);
set_param([model_name '/Gain_P'], 'Gain', '[800; 600]');

add_block('simulink/Math Operations/Gain', [model_name '/Gain_D'], ...
    'Position', [280, 100, 310, 130]);
set_param([model_name '/Gain_D'], 'Gain', '[100; 80]');

% 需要速度误差 (简化: 直接用位置误差)
add_line(model_name, 'Sum_error/1', 'Gain_P/1');

%% 8. 手动开关 (切换PID/计算力矩)
add_block('simulink/Signal Routing/Manual Switch', [model_name '/Switch'], ...
    'Position', [350, 70, 380, 100]);

%% 保存模型
save_system(model_name);
close_system(model_name);

fprintf('Simulink模型 "%s.slx" 已创建！\n', model_name);
fprintf('注意: 此脚本创建了基本框架，需要在Simulink中手动完善连线。\n');
fprintf('建议在MATLAB中打开sim.slx进行手动调整。\n');
