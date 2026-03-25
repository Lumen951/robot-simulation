# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an educational **robot modeling and control simulation** toolkit (机器人建模与控制仿真) for a university course. The project implements fundamental robot kinematics and dynamics concepts from scratch using Python and scientific computing libraries.

**Core Topics**: Forward/inverse kinematics, DH parameters, Lagrangian/Newton-Euler dynamics, trajectory planning, Jacobian matrices, and robot visualization.

## Development Commands

### Package Management (UV)
```bash
# Install dependencies
uv sync

# Run a single script
uv run src/<script-name>.py

# Batch run all scripts and save outputs
uv run run_all.py
```

### Running Individual Scripts

**Interactive scripts** (with GUI/animation):
- `uv run src/puma560-正运动学.py` - Puma560 forward kinematics visualization
- `uv run src/puma560-逆运动学.py` - Inverse kinematics solver
- `uv run src/雅可比矩阵.py` - Jacobian matrix demonstrations
- `uv run src/puma560-轨迹规划.py` - Trajectory planning (line/circle)

**Computation scripts** (non-interactive, print output):
- `uv run src/旋转矩阵.py`
- `uv run src/齐次变换.py`
- `uv run src/单连杆动能计算.py`
- `uv run src/单连杆势能计算.py`
- `uv run src/单摆的拉格朗日方程.py`
- `uv run src/二连杆机构DH建模.py`
- `uv run src/二连杆的拉格朗日分析.py`
- `uv run src/二连杆的牛顿欧拉法分析.py`

### Batch Execution

`run_all.py` executes all scripts non-interactively and saves:
- Console output → `output/<script-name>/output.txt`
- Figures → `output/<script-name>/fig_*.png` (150 DPI)

## Architecture

### Code Organization

```
src/                     # 12 educational modules (standalone scripts)
  ├── puma560-*.py       # Puma560 robot: FK, IK, trajectory
  ├── 二连杆*.py         # Two-link manipulator: DH, Lagrangian, Newton-Euler
  ├── 单连杆*.py         # Single link: energy calculations
  ├── 单摆*.py           # Simple pendulum: Lagrangian dynamics
  ├── 旋转矩阵.py        # Rotation matrix fundamentals
  ├── 齐次变换.py        # Homogeneous transformations
  └── 雅可比矩阵.py      # Jacobian and manipulability

run_all.py               # Batch execution automation
pyproject.toml           # UV project config
docs/                    # Course materials and assignments
```

### Script Structure Pattern

Most interactive scripts follow this pattern:

```python
class RobotDemo:
    def __init__(self):
        # Physical parameters (link lengths, masses, DH params)

    def compute_something(self, ...):
        # Core computation (kinematics, dynamics, etc.)

    def visualize(self, ...):
        # Matplotlib visualization with Chinese labels

def main():
    demo = RobotDemo()
    # Interactive menu with user input (input() prompts)

if __name__ == "__main__":
    main()
```

### Key Dependencies

- **numpy**: Matrix operations, numerical methods
- **matplotlib**: 3D plotting, animation (`SimHei` font for Chinese)
- **scipy**: `solve_ivp` for ODE integration, `optimize` for inverse kinematics

## Coding Conventions

### File Naming and Language
- Scripts use **Chinese filenames** (e.g., `puma560-正运动学.py`)
- Comments and UI strings are in **Chinese**
- Variable names and code in **English**

### Matplotlib Configuration (Required)
```python
import matplotlib.pyplot as plt
plt.rcParams['font.sans-serif'] = ['SimHei']  # Chinese font support
plt.rcParams['axes.unicode_minus'] = False    # Fix minus sign display
```

### DH Parameters Implementation

DH transformation matrix (standard convention):
```python
def dh_transform(self, theta, d, a, alpha):
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0,              np.sin(alpha),                np.cos(alpha),               d],
        [0,              0,                            0,                            1]
    ])
```

### Inverse Kinematics Pattern

Use `scipy.optimize` for numerical IK:
```python
from scipy.optimize import fsolve, least_squares

def inverse_kinematics(self, target_x, target_y, target_z):
    def error_func(joint_angles):
        pos = self.forward_kinematics(*joint_angles)
        return pos - np.array([target_x, target_y, target_z])

    result = least_squares(error_func, initial_guess)
    return result.x
```

## Assignment Context

Current assignment (Week 4, Lesson 1): **UR3e Simplified Model Simulation**
- Build UR3e forward kinematics model from DH parameters
- Plan circular trajectory (10cm diameter) in 3D space
- Solve inverse kinematics numerically for each trajectory point
- Create animation showing robot motion, actual trajectory, and desired trajectory
- (Advanced) Add simple dynamics simulation for torque analysis

See `docs/第4周-第1课-作业.md` for full assignment details.

## Notes

- No external robotics libraries used (pure numpy/scipy/matplotlib)
- All implementations are from first principles for educational purposes
- Scripts are designed as interactive demonstrations, not production code
- Output from `run_all.py` is used for course materials submission
