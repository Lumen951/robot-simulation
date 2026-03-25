function [ddq, tau_info] = two_link_dynamics(tau, q, dq, params)
% TWO_LINK_DYNAMICS 二连杆机械臂动力学方程
%
% 输入:
%   tau    - 2x1 关节力矩向量 [tau1; tau2] (Nm)
%   q      - 2x1 关节角度向量 [q1; q2] (rad)
%   dq     - 2x1 关节角速度向量 [dq1; dq2] (rad/s)
%   params - 参数结构体, 包含 m1, m2, l1, l2, I1, I2, g
%
% 输出:
%   ddq    - 2x1 关节角加速度向量 [ddq1; ddq2] (rad/s^2)
%   tau_info - 信息结构体 (可选), 包含 M, C, G 用于控制器调试

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
    C = [C1; C2];

    % 重力向量 G(q)
    G1 = g*cos(q(1))*(m1*l1/2 + m2*l1) + g*m2*l2*cos(q(1)+q(2))/2;
    G2 = g*m2*l2*cos(q(1)+q(2))/2;
    G = [G1; G2];

    % 动力学方程: M(q)*ddq + C(q,dq) + G(q) = tau
    % 解得: ddq = M^{-1} * (tau - C - G)
    ddq = M \ (tau - C - G);

    % 返回调试信息（可选）
    if nargout > 1
        tau_info.M = M;
        tau_info.C = C;
        tau_info.G = G;
    end
end
