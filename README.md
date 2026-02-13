# 🦾 POE Robot Kinematics Solver

![Python](https://img.shields.io/badge/Python-3.8%2B-blue?logo=python&logoColor=white)
![NumPy](https://img.shields.io/badge/Numpy-Powered-013243?logo=numpy&logoColor=white)
![License](https://img.shields.io/badge/License-MIT-green)

A lightweight, **Product of Exponentials (POE)** based kinematic solver for serial robot arms (N-DOF).

This Python library provides a robust implementation of **Forward Kinematics (FK)** and **Inverse Kinematics (IK)** for arbitrary serial manipulators. It allows you to define robots using standard **DH parameters**, which are internally converted to the Modern Robotics POE formulation (Screw Axes & Home Configuration).

Pre-configured setups for **Universal Robots UR5e** (6-DOF) and **Franka Emika Panda** (7-DOF) are included out of the box.

## ✨ Features

- **Product of Exponentials (POE):** Utilizes screw theory for singularity-robust kinematics ($T(\theta) = e^{[S_1]\theta_1} \dots e^{[S_n]\theta_n} M$).
- **N-DOF Support:** Works with any serial chain robot (6-DOF, 7-DOF, etc.).
- **Numerical IK Solver:** Implements a damped least-squares Newton-Raphson method for Inverse Kinematics.
- **Easy Configuration:** Define robots using familiar standard DH parameters (`a`, `d`, `alpha`).
- **Type Hinting:** Fully typed code for better development experience and autocomplete.
- **Minimal Dependencies:** Pure Python with only `numpy` required.

## 📦 Installation

### Prerequisites

You only need `numpy` installed in your environment:

```bash
pip install numpy


## 🚀 Usage

### 1. Using Built-in Robots (UR5e / Franka Panda)

You can quickly initialize the solver using the provided presets.

```python
import numpy as np
from ur5e_solver import RobotKinematicsPOE, UR5E_CONFIG, FRANKA_CONFIG

# Initialize solver for UR5e
solver = RobotKinematicsPOE(UR5E_CONFIG)
# For Franka Emika Panda: solver = RobotKinematicsPOE(FRANKA_CONFIG)

# --- Forward Kinematics ---
# Joint angles in radians (6 joints for UR5e)
# Example: Home position (all zeros)
test_joints = [0, 0, 0, 0, 0, 0]

T_matrix = solver.forward_kinematics(test_joints)

print("End-Effector Pose (4x4 Homogeneous Matrix):")
print(T_matrix)

# --- Inverse Kinematics ---
# Calculate joints required to reach T_matrix
ik_result = solver.inverse_kinematics(T_matrix)

if ik_result.converged:
    print(f"IK Solution (Joints): {np.round(ik_result.joints, 3)}")
else:
    print("IK Failed to converge within the iteration limit.")
```

---

### 2. Theory Section (이론 설명)

이 부분은 수식(`$$`)이 포함되어 있어 GitHub에서 렌더링될 때 깔끔하게 보입니다.



```markdown
## 📐 Theory: Product of Exponentials (POE)

This library implements the **Product of Exponentials (POE)** formula, which is a modern approach to robot kinematics widely used in research and advanced control.

### Forward Kinematics
Unlike traditional methods that chain local transformation matrices ($A_1 \cdot A_2 \dots$), the POE formula describes the end-effector pose $T(\theta)$ as a sequence of screw motions relative to a base frame:

$$
T(\theta) = e^{[S_1]\theta_1} e^{[S_2]\theta_2} \dots e^{[S_n]\theta_n} M
$$

Where:
- **$S_i$**: The **Screw Axis** of joint $i$, expressed in the fixed base frame. It combines the axis of rotation ($\omega$) and linear velocity ($v$).
- **$\theta_i$**: The joint angle (variable).
- **$M$**: The **Home Configuration** matrix. This is the pose of the end-effector when all joint angles are zero ($\theta = 0$).
- **$e^{[S]\theta}$**: The matrix exponential, mapping a twist to a rigid body motion in $SE(3)$.

### Why POE?
1.  **Global Reference:** All screw axes are defined in the base frame (Space form) or tool frame (Body form), avoiding the complexity of assigning local coordinate frames for every link (as in DH).
2.  **Robustness:** It elegantly handles singularities and continuous motion without the ambiguity often found in Euler angle representations.
3.  **Geometric Intuition:** It directly relates joint motions to twists in 3D space.

### Automatic Conversion
While POE is powerful, many robot datasheets still provide **Denavit-Hartenberg (DH)** parameters. This library bridges that gap by:
1.  Taking standard DH parameters (`a`, `d`, `alpha`) as input.
2.  Automatically converting them into the equivalent **Screw Axes ($S$)** and **Home Configuration ($M$)**.
3.  Using the POE formula for all subsequent FK/IK calculations.
