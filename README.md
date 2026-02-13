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