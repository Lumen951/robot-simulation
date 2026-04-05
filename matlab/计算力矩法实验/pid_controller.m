function [tau, integral_error] = pid_controller(q_d, dq_d, q, dq, kp, ki, kd, integral_error, dt)
% PID_CONTROLLER 独立关节PID控制器
%
% 输入:
%   q_d           - 2x1 期望关节角度 [q1_d; q2_d] (rad)
%   dq_d          - 2x1 期望关节角速度 [dq1_d; dq2_d] (rad/s)
%   q             - 2x1 实际关节角度 [q1; q2] (rad)
%   dq            - 2x1 实际关节角速度 [dq1; dq2] (rad/s)
%   kp, ki, kd    - 2x1 PID增益向量
%   integral_error - 2x1 积分误差累加器
%   dt            - 采样时间 (s)
%
% 输出:
%   tau           - 2x1 控制力矩 [tau1; tau2] (Nm)
%   integral_error - 更新后的积分误差累加器

    % 位置误差
    e = q_d - q;

    % 速度误差
    de = dq_d - dq;

    % 积分项累加
    integral_error = integral_error + e * dt;

    % 积分抗饱和 (简单限幅)
    for i = 1:2
        if abs(integral_error(i)) > 10  % 限制积分项范围
            integral_error(i) = 10 * sign(integral_error(i));
        end
    end

    % PID控制律: tau = Kp*e + Ki*∫e + Kd*ė
    tau = kp .* e + ki .* integral_error + kd .* de;
end
