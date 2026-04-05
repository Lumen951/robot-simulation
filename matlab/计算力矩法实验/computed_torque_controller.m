function [tau, dynamics_info] = computed_torque_controller(q_d, dq_d, ddq_d, q, dq, params, Kp, Kd)
% COMPUTED_TORQUE_CONTROLLER 计算力矩控制器
%
% 控制律: tau = M(q)*(ddq_d + Kp*e + Kd*de) + C(q,dq)*dq + G(q)
%
% 输入:
%   q_d   - 2x1 期望关节角度 [q1_d; q2_d] (rad)
%   dq_d  - 2x1 期望关节角速度 [dq1_d; dq2_d] (rad/s)
%   ddq_d - 2x1 期望关节角加速度 [ddq1_d; ddq2_d] (rad/s^2)
%   q     - 2x1 实际关节角度 [q1; q2] (rad)
%   dq    - 2x1 实际关节角速度 [dq1; dq2] (rad/s)
%   params - 参数结构体
%   Kp, Kd - 2x1 PD增益向量
%
% 输出:
%   tau           - 2x1 控制力矩 [tau1; tau2] (Nm)
%   dynamics_info - 信息结构体 (可选), 包含 M, C, G 用于调试

    % 提取参数
    l1 = params.l1;
    l2 = params.l2;
    m1 = params.m1;
    m2 = params.m2;
    I1 = params.I1;
    I2 = params.I2;
    g = params.g;

    % 辅助计算
    c2 = cos(q(2));
    s2 = sin(q(2));

    % 惯量矩阵 M(q)
    M11 = I1 + I2 + m1*l1^2/4 + m2*(l1^2 + l2^2/4 + l1*l2*c2);
    M12 = I2 + m2*(l2^2/4 + l1*l2*c2/2);
    M21 = M12;
    M22 = I2 + m2*l2^2/4;
    M = [M11, M12; M21, M22];

    % 科氏力和离心力向量 C(q,dq)*dq
    h = -m2*l1*l2*s2/2;
    C1 = h * dq(2) * (2*dq(1) + dq(2));
    C2 = h * dq(1)^2;
    C_vec = [C1; C2];

    % 重力向量 G(q)
    G1 = g*cos(q(1))*(m1*l1/2 + m2*l1) + g*m2*l2*cos(q(1)+q(2))/2;
    G2 = g*m2*l2*cos(q(1)+q(2))/2;
    G = [G1; G2];

    % 跟踪误差
    e = q_d - q;
    de = dq_d - dq;

    % 计算力矩控制律
    % tau = M * (ddq_d + Kp*e + Kd*de) + C + G
    tau = M * (ddq_d + Kp .* e + Kd .* de) + C_vec + G;

    % 返回调试信息（可选）
    if nargout > 1
        dynamics_info.M = M;
        dynamics_info.C = C_vec;
        dynamics_info.G = G;
    end
end
