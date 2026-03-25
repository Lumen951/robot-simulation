% SIMULINK_GUIDE Simulink模型构建指南
%
% 此脚本包含完整的Simulink模型配置说明
% 按照以下步骤在MATLAB Simulink中手动创建模型

%% ========== 步骤1: 创建新模型 ==========
% 1. 打开MATLAB
% 2. 输入: simulink
% 3. 点击 Blank Model
% 4. 保存为: sim.slx

%% ========== 步骤2: 添加模块 ==========
%
% --- 2.1 期望轨迹生成 ---
% 模块: Sine Wave (x2)
% 位置: simulink/Sources/Sine Wave
% 参数:
%   Sine_q1: Amplitude=1, Frequency=2 (rad/s), Bias=0
%   Sine_q2: Amplitude=0.5, Frequency=3 (rad/s), Bias=0
%
% --- 2.2 信号合并 ---
% 模块: Mux
% 位置: simulink/Signal Routing/Mux
% 参数: Number of inputs = 2
%
% --- 2.3 控制器子系统 ---
% 创建两个Subsystem:
%   - PID_Controller
%   - ComputedTorque_Controller
%
% --- 2.4 机器人Plant ---
% 模块: MATLAB Function
% 位置: simulink/User-Defined Functions/MATLAB Function
%
% --- 2.5 积分器 ---
% 模块: Integrator (x2)
% 位置: simulink/Continuous/Integrator
% 用途: 从加速度积分得到速度和位置
%
% --- 2.6 示波器 ---
% 模块: Scope
% 位置: simulink/Sinks/Scope
% 参数: Number of input ports = 4

%% ========== 步骤3: MATLAB Function代码 ==========
%
% 双击 MATLAB Function 块，输入以下代码:
%
% function [q, dq] = RobotPlant(tau, q_in, dq_in, params)
% % 二连杆动力学积分
% persistent q_prev dq_prev
%
% if isempty(q_prev)
%     q_prev = [0.1; 0.05];  % 初始角度
%     dq_prev = [0; 0];       % 初始角速度
% end
%
% % 参数
% l1 = 1.0; l2 = 0.8;
% m1 = 10.0; m2 = 5.0;
% I1 = 0.83; I2 = 0.21;
% g = 9.81;
%
% q = q_prev;
% dq = dq_prev;
%
% % 计算惯量矩阵
% c2 = cos(q(2)); s2 = sin(q(2));
% M11 = I1 + I2 + m1*l1^2/4 + m2*(l1^2 + l2^2/4 + l1*l2*c2);
% M12 = I2 + m2*(l2^2/4 + l1*l2*c2/2);
% M22 = I2 + m2*l2^2/4;
% M = [M11, M12; M12, M22];
%
% % 科氏力
% h = -m2*l1*l2*s2/2;
% C1 = h * dq(2) * (2*dq(1) + dq(2));
% C2 = h * dq(1)^2;
% C = [C1; C2];
%
% % 重力
% G1 = g*cos(q(1))*(m1*l1/2 + m2*l1) + g*m2*l2*cos(q(1)+q(2))/2;
% G2 = g*m2*l2*cos(q(1)+q(2))/2;
% G = [G1; G2];
%
% % 加速度
% ddq = M \ (tau - C - G);
%
% % 更新状态 (简单欧拉积分)
% dt = 0.01;
% dq = dq + ddq * dt;
% q = q + dq * dt;
%
% q_prev = q;
% dq_prev = dq;
% end

%% ========== 步骤4: PID子系统内容 ==========
%
% 输入: e (误差), de (误差导数)
% 输出: tau
%
% 内部:
% - Gain Kp: [800; 600]
% - Gain Kd: [100; 80]
% - Gain Ki: [50; 30]
% - Integrator (带抗饱和)
% - Sum: tau = Kp*e + Ki*integral + Kd*de

%% ========== 步骤5: 计算力矩子系统内容 ==========
%
% 输入: q_d, dq_d, ddq_d, q, dq
% 输出: tau
%
% 内部需要计算 M, C, G 矩阵
% 然后应用控制律: tau = M*(ddq_d + Kp*e + Kd*de) + C + G
%
% Kp = [100; 80]
% Kd = [30; 25]

%% ========== 步骤6: 模型连接结构 ==========
%
%  [Sine_q1]---\
%               [Mux]---[q_d]---\
%  [Sine_q2]---/                 \
%                                 [Controller]---[tau]---[Plant]---[q,dq]
%                     [q,dq]----/                      \       /
%                      \--------------------------------/     /
%                       (反馈回路)                          /
%                                                           /
%  [q_d, q]----[Sum: e]-----------------------------------/
%  [dq_d, dq]--[Sum: de]---------------------------------/

%% ========== 步骤7: 仿真设置 ==========
%
% Simulation > Model Configuration Parameters:
% - Stop time: 10
% - Solver: ode45 (Dormand-Prince)
% - Max step size: 0.01
% - Relative tolerance: 1e-3

%% ========== 步骤8: 运行仿真 ==========
%
% 1. 点击 Run 按钮或 Ctrl+T
% 2. 双击 Scope 查看结果
% 3. 在MATLAB命令窗口输入: simout 查看数据
%
% 或者通过脚本运行:
% sim('sim');

fprintf('Simulink模型构建指南已显示。\n');
fprintf('详细步骤请参考此文件中的注释。\n');
