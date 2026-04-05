# Simulink 建模操作指南

> 二连杆机械臂 PID / 计算力矩控制模型手动搭建步骤

## 准备工作

1. 打开 MATLAB
2. 命令窗口输入 `simulink`，点击 **Blank Model**
3. `Ctrl+S` 保存为 `two_link_control.slx`（保存到 `matlab/综合实例练习/output/` 目录）

---

## 第一阶段：被控对象（Plant）

> 目标：搭建动力学方程 + 双积分器，实现 τ → [q, dq] 的闭环

### 1.1 添加模块

在 Simulink Library Browser 中找到并拖入以下模块：

| 模块 | Library 路径 | 重命名为 |
|:-----|:------------|:---------|
| MATLAB Function | User-Defined Functions | `Robot Dynamics` |
| Integrator | Continuous | `Integrator dq` |
| Integrator | Continuous | `Integrator q` |

### 1.2 设置 Integrator 初始值

- 双击 `Integrator q`，Initial Condition 填 `[-0.1; -0.05]`
- 双击 `Integrator dq`，Initial Condition 填 `[0; 0]`

### 1.3 编写动力学函数

双击 `Robot Dynamics` 模块，输入以下代码：

```matlab
function ddq = f(tau, q, dq)
l1=1.0; l2=0.8; m1=10; m2=5; I1=0.83; I2=0.21; g=9.81;
c2=cos(q(2)); s2=sin(q(2));
M11=I1+I2+m1*l1^2/4+m2*(l1^2+l2^2/4+l1*l2*c2);
M12=I2+m2*(l2^2/4+l1*l2*c2/2);
M22=I2+m2*l2^2/4;
M=[M11,M12;M12,M22];
h=-m2*l1*l2*s2/2;
C=[h*dq(2)*(2*dq(1)+dq(2)); h*dq(1)^2];
G=[g*cos(q(1))*(m1*l1/2+m2*l1)+g*m2*l2*cos(q(1)+q(2))/2; g*m2*l2*cos(q(1)+q(2))/2];
ddq = M \ (tau - C - G);
```

确认函数有 **3 个输入** (tau, q, dq) 和 **1 个输出** (ddq)。
如果端口数量不对，在 MATLAB Function 编辑器中点击 "Edit Data" 调整。

### 1.4 连线（Plant 内部闭环）

```
[Robot Dynamics].ddq  →  [Integrator dq].in
[Integrator dq].out   →  [Integrator q].in
[Integrator q].out    →  分叉点 A
分叉点 A  →  [Robot Dynamics].q    (第2个输入)
[Integrator dq].out  →  分叉点 B
分叉点 B  →  [Robot Dynamics].dq   (第3个输入)
```

**分叉点操作**：在连线上按住鼠标右键拖出分支线。

### 1.5 验证 Plant

目前 Plant 还没有 tau 输入。我们先跳到第二阶段建轨迹生成器，然后再回来连接控制器。

---

## 第二阶段：期望轨迹生成

### 2.1 添加模块

| 模块 | 重命名为 |
|:-----|:---------|
| Clock | `Clock` |
| MATLAB Function | `Trajectory Gen` |

### 2.2 编写轨迹生成函数

双击 `Trajectory Gen`，输入：

```matlab
function [q_d, dq_d, ddq_d] = f(t)
q_d = [sin(2*t); 0.5*sin(3*t)];
dq_d = [2*cos(2*t); 1.5*cos(3*t)];
ddq_d = [-4*sin(2*t); -4.5*sin(3*t)];
```

确认有 **1 个输入** (t) 和 **3 个输出** (q_d, dq_d, ddq_d)。

### 2.3 连线

```
[Clock]  →  [Trajectory Gen].t
```

---

## 第三阶段：计算力矩控制器

### 3.1 添加模块

| 模块 | 重命名为 |
|:-----|:---------|
| MATLAB Function | `Computed Torque` |

### 3.2 编写控制律

双击 `Computed Torque`，输入：

```matlab
function tau = f(q_d, dq_d, ddq_d, q, dq)
l1=1.0; l2=0.8; m1=10; m2=5; I1=0.83; I2=0.21; g=9.81;
c2=cos(q(2)); s2=sin(q(2));
M11=I1+I2+m1*l1^2/4+m2*(l1^2+l2^2/4+l1*l2*c2);
M12=I2+m2*(l2^2/4+l1*l2*c2/2);
M22=I2+m2*l2^2/4;
M=[M11,M12;M12,M22];
h=-m2*l1*l2*s2/2;
C=[h*dq(2)*(2*dq(1)+dq(2)); h*dq(1)^2];
G=[g*cos(q(1))*(m1*l1/2+m2*l1)+g*m2*l2*cos(q(1)+q(2))/2; g*m2*l2*cos(q(1)+q(2))/2];
e=q_d-q; de=dq_d-dq;
Kp=[100;80]; Kd=[30;25];
tau = M*(ddq_d + Kp.*e + Kd.*de) + C + G;
```

确认 **5 个输入** (q_d, dq_d, ddq_d, q, dq)，**1 个输出** (tau)。

### 3.3 连接计算力矩控制器

```
[Trajectory Gen].q_d    →  [Computed Torque].q_d      (第1输入)
[Trajectory Gen].dq_d   →  [Computed Torque].dq_d     (第2输入)
[Trajectory Gen].ddq_d  →  [Computed Torque].ddq_d    (第3输入)
[Integrator q].out      →  [Computed Torque].q        (第4输入)
[Integrator dq].out     →  [Computed Torque].dq       (第5输入)
```

---

## 第四阶段：PID 控制器

### 4.1 添加模块

| 模块 | 重命名为 |
|:-----|:---------|
| MATLAB Function | `PID Controller` |

### 4.2 编写 PID 控制律

双击 `PID Controller`，输入：

```matlab
function tau = f(q_d, dq_d, q, dq)
Kp=[800;600]; Ki=[50;30]; Kd=[100;80];
persistent int_e
if isempty(int_e), int_e=[0;0]; end
e = q_d - q;
de = dq_d - dq;
int_e = int_e + e * 0.01;  % dt=0.01
int_e = max(min(int_e, 10), -10);  % 抗饱和
tau = Kp.*e + Ki.*int_e + Kd.*de;
```

确认 **4 个输入** (q_d, dq_d, q, dq)，**1 个输出** (tau)。

### 4.3 连接 PID 控制器

```
[Trajectory Gen].q_d    →  [PID Controller].q_d      (第1输入)
[Trajectory Gen].dq_d   →  [PID Controller].dq_d     (第2输入)
[Integrator q].out      →  [PID Controller].q        (第3输入)
[Integrator dq].out     →  [PID Controller].dq       (第4输入)
```

---

## 第五阶段：控制器切换 + 连接 Plant

### 5.1 添加模块

| 模块 | 重命名为 |
|:-----|:---------|
| Manual Switch | `Switch` |

### 5.2 连线

```
[PID Controller].tau        →  [Switch].上输入（点击上方端口）
[Computed Torque].tau       →  [Switch].下输入（点击下方端口）
[Switch].输出               →  [Robot Dynamics].tau   (第1输入)
```

**注意**：Switch 默认选中上方（PID），双击 Switch 可切换到下方（计算力矩）。

---

## 第六阶段：观测与记录

### 6.1 添加模块

| 模块 | 重命名为 | 参数 |
|:-----|:---------|:-----|
| Scope | `Scope` | 双击设置输入端口数为 3 |
| To Workspace | `Log q` | VariableName: `q_log`, Save format: Array |
| To Workspace | `Log tau` | VariableName: `tau_log`, Save format: Array |

### 6.2 连线

```
[Integrator q].out      →  [Scope]第1输入
[Trajectory Gen].q_d    →  [Scope]第2输入
[Switch].输出           →  [Scope]第3输入

[Integrator q].out      →  [Log q]
[Switch].输出           →  [Log tau]
```

---

## 第七阶段：仿真参数

菜单栏 → Modeling → Model Settings (Ctrl+E)：

- **Stop time**: `10`
- **Solver**: `ode45 (Dormand-Prince)`
- **Max step size**: `0.01`
- **Relative tolerance**: `1e-4`

---

## 运行

1. 双击 `Switch`，切换到下方（计算力矩控制）
2. 点击 **Run** 按钮
3. 双击 `Scope` 查看结果
4. 命令窗口输入 `q_log` 和 `tau_log` 查看记录的数据

## 切换测试

- 双击 `Switch` 切到上方 = PID 控制
- 双击 `Switch` 切到下方 = 计算力矩控制
- 分别运行，对比 Scope 中的跟踪效果
