% CREATE_SIM_MODEL 创建二连杆计算力矩控制Simulink模型
%
% 使用方法:
%   1. 在MATLAB中运行: cd('d:/University/Junior/2nd/code/robot-simulation/matlab')
%   2. 运行: create_sim_model
%   3. 将生成 sim.slx 文件

clear; clc;

model_name = 'sim';

% 检查是否已存在，存在则删除
if bdIsLoaded(model_name)
    close_system(model_name, 0);
end
if exist([model_name '.slx'], 'file')
    delete([model_name '.slx']);
end

% 创建新模型
new_system(model_name);
open_system(model_name);

fprintf('正在创建Simulink模型...\n');

%% 设置模型参数
set_param(model_name, 'StopTime', '10');
set_param(model_name, 'Solver', 'ode45');
set_param(model_name, 'SolverType', 'Variable-step');
set_param(model_name, 'RelTol', '1e-3');

%% 添加子系统: 期望轨迹生成
add_block('simulink/Sources/Sine Wave', [model_name '/Sine_q1'], ...
    'Position', [50, 50, 100, 80]);
set_param([model_name '/Sine_q1'], 'Amplitude', '1', 'Frequency', '2', 'Bias', '0');

add_block('simulink/Sources/Sine Wave', [model_name '/Sine_q2'], ...
    'Position', [50, 130, 100, 160]);
set_param([model_name '/Sine_q2'], 'Amplitude', '0.5', 'Frequency', '3', 'Bias', '0');

% 合并期望信号
add_block('simulink/Signal Routing/Mux', [model_name '/Mux_qd'], ...
    'Position', [150, 80, 155, 130]);
set_param([model_name '/Mux_qd'], 'Inputs', '2');

%% 添加控制器切换开关
add_block('simulink/Signal Routing/Manual Switch', [model_name '/Controller_Switch'], ...
    'Position', [250, 100, 280, 130]);

%% 添加子系统: 机器人Plant
% 使用MATLAB Function块实现动力学
add_block('simulink/User-Defined Functions/MATLAB Function', [model_name '/Robot_Dynamics'], ...
    'Position', [450, 90, 530, 150]);

% 设置MATLAB Function内容
set_param([model_name '/Robot_Dynamics'], 'FunctionName', 'robot_plant');

%% 添加积分器
add_block('simulink/Continuous/Integrator', [model_name '/Integrator_q'], ...
    'Position', [580, 90, 610, 120]);
add_block('simulink/Continuous/Integrator', [model_name '/Integrator_dq'], ...
    'Position', [580, 150, 610, 180]);

%% 添加示波器
add_block('simulink/Sinks/Scope', [model_name '/Scope'], ...
    'Position', [700, 80, 730, 110]);
set_param([model_name '/Scope'], 'NumInputPorts', '4');

add_block('simulink/Sinks/To Workspace', [model_name '/To Workspace'], ...
    'Position', [700, 130, 730, 160]);
set_param([model_name '/To Workspace'], 'VariableName', 'simout');

%% 连线
add_line(model_name, 'Sine_q1/1', 'Mux_qd/1');
add_line(model_name, 'Sine_q2/1', 'Mux_qd/2');
add_line(model_name, 'Mux_qd/1', 'Controller_Switch/1');

%% 保存模型
save_system(model_name);
close_system(model_name);

fprintf('Simulink模型 "%s.slx" 已创建！\n', model_name);
fprintf('\n注意: 此脚本创建了基本框架。\n');
fprintf('你需要在Simulink中手动完成以下步骤:\n');
fprintf('  1. 打开 sim.slx\n');
fprintf('  2. 双击 "Robot_Dynamics" 块，输入动力学代码\n');
fprintf('  3. 添加PID和计算力矩控制器子系统\n');
fprintf('  4. 完成所有连线\n');
fprintf('  5. 连接到示波器\n');
