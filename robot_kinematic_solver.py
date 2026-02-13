"""Generic forward/inverse kinematics using Product of Exponentials (POE).

This module provides a POE-based kinematic solver for arbitrary N-DOF serial
robot arms.  Robot-specific parameters (DH table, tool offset) are supplied
via ``RobotConfig`` dataclasses.  Built-in presets are available for the
**UR5e** (6-DOF) and **Franka Emika Panda** (7-DOF).

Usage::

    from ur5e_solver import RobotKinematicsPOE, UR5E_CONFIG, FRANKA_CONFIG

    solver = RobotKinematicsPOE(UR5E_CONFIG)         # or FRANKA_CONFIG
    T = solver.forward_kinematics(joint_angles)
    ik = solver.inverse_kinematics(T)
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Sequence

import numpy as np
from numpy.typing import NDArray

# ---------------------------------------------------------------------------
# Type aliases
# ---------------------------------------------------------------------------
Vec3 = NDArray[np.floating]
VecN = NDArray[np.floating]
Mat3 = NDArray[np.floating]
Mat4 = NDArray[np.floating]
MatMN = NDArray[np.floating]

# ---------------------------------------------------------------------------
# Solver-wide defaults
# ---------------------------------------------------------------------------
ZERO_THRESHOLD: float = 1e-6
IK_DEFAULT_MAX_ITER: int = 50
IK_DEFAULT_TOLERANCE: float = 1e-4
IK_DAMPING_FACTOR: float = 0.01


# ---------------------------------------------------------------------------
# Robot configuration
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class RobotConfig:
    """Immutable description of a serial robot's kinematics.

    Attributes:
        name: Human-readable robot name.
        dh_params: List of (a, d, alpha) tuples – one per revolute joint.
        tool_offset: 3-D translation from the last DH frame to the tool tip.
    """

    name: str
    dh_params: list[tuple[float, float, float]]
    tool_offset: Vec3 = field(default_factory=lambda: np.zeros(3))

    @property
    def num_joints(self) -> int:
        return len(self.dh_params)


# ---------------------------------------------------------------------------
# Built-in robot presets
# ---------------------------------------------------------------------------
UR5E_CONFIG = RobotConfig(
    name="UR5e",
    dh_params=[
        (0.0, 0.1625, np.pi / 2),
        (-0.425, 0.0, 0.0),
        (-0.3922, 0.0, 0.0),
        (0.0, 0.1333, np.pi / 2),
        (0.0, 0.0997, -np.pi / 2),
        (0.0, 0.0996, 0.0),
    ],
    tool_offset=np.array([0.0, 0.0, 0.12]),
)

FRANKA_CONFIG = RobotConfig(
    name="Franka Emika Panda",
    dh_params=[
        (0.0, 0.333, 0.0),
        (0.0, 0.0, -np.pi / 2),
        (0.0, 0.316, np.pi / 2),
        (0.0825, 0.0, np.pi / 2),
        (-0.0825, 0.384, -np.pi / 2),
        (0.0, 0.0, np.pi / 2),
        (0.088, 0.107, np.pi / 2),
    ],
    tool_offset=np.array([0.0, 0.0, 0.1034]),
)


# ---------------------------------------------------------------------------
# Result container
# ---------------------------------------------------------------------------
@dataclass
class IKResult:
    """Inverse-kinematics result."""

    joints: NDArray[np.floating]
    converged: bool


# ---------------------------------------------------------------------------
# Lie-algebra helpers (module-level, stateless)
# ---------------------------------------------------------------------------
def skew(v: Vec3) -> Mat3:
    """Return the 3x3 skew-symmetric matrix of a 3-D vector (so(3))."""
    return np.array([
        [0.0, -v[2], v[1]],
        [v[2], 0.0, -v[0]],
        [-v[1], v[0], 0.0],
    ])


def matrix_exp3(omega: Vec3, theta: float) -> Mat3:
    """Rodrigues' formula: rotation matrix from unit axis *omega* and angle *theta*."""
    omega_hat = skew(omega)
    return (
        np.eye(3)
        + np.sin(theta) * omega_hat
        + (1.0 - np.cos(theta)) * (omega_hat @ omega_hat)
    )


def matrix_exp6(screw: Vec6, theta: float) -> Mat4:
    """Matrix exponential on SE(3): T = exp([S] * theta)."""
    omega = screw[:3]
    v = screw[3:]

    T = np.eye(4)

    if np.linalg.norm(omega) < ZERO_THRESHOLD:
        # Pure translation (prismatic joint)
        T[:3, 3] = v * theta
    else:
        omega_hat = skew(omega)
        T[:3, :3] = matrix_exp3(omega, theta)
        G = (
            np.eye(3) * theta
            + (1.0 - np.cos(theta)) * omega_hat
            + (theta - np.sin(theta)) * (omega_hat @ omega_hat)
        )
        T[:3, 3] = G @ v

    return T


def adjoint(T: Mat4) -> Mat6:
    """6x6 adjoint representation of a homogeneous transform *T*."""
    R = T[:3, :3]
    p_hat = skew(T[:3, 3])

    Ad = np.zeros((6, 6))
    Ad[:3, :3] = R
    Ad[3:, 3:] = R
    Ad[3:, :3] = p_hat @ R
    return Ad


def rotation_error(R: Mat3) -> Vec3:
    """Extract the rotation-error vector from a rotation matrix via the log map."""
    trace_val = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    angle = np.arccos(trace_val)

    if abs(angle) < ZERO_THRESHOLD:
        return np.zeros(3)

    axis = np.array([
        R[2, 1] - R[1, 2],
        R[0, 2] - R[2, 0],
        R[1, 0] - R[0, 1],
    ])
    return axis * (angle / (2.0 * np.sin(angle)))


def transform_to_pose(T: Mat4) -> Vec6:
    """Convert a 4x4 homogeneous transform to a 6-D pose [x, y, z, roll, pitch, yaw].

    Euler angles follow the ZYX (yaw-pitch-roll) intrinsic convention,
    equivalent to the XYZ extrinsic convention.

    Args:
        T: 4x4 homogeneous transformation matrix.

    Returns:
        (6,) array of [x, y, z, roll, pitch, yaw] in metres / radians.
    """
    x, y, z = T[:3, 3]

    # ZYX intrinsic Euler angles from the rotation sub-matrix
    sy = -T[2, 0]
    cy = np.sqrt(T[0, 0] ** 2 + T[1, 0] ** 2)

    if cy > ZERO_THRESHOLD:
        roll = np.arctan2(T[2, 1], T[2, 2])
        pitch = np.arctan2(sy, cy)
        yaw = np.arctan2(T[1, 0], T[0, 0])
    else:
        # Gimbal lock: pitch ≈ ±90°
        roll = np.arctan2(-T[1, 2], T[1, 1])
        pitch = np.arctan2(sy, cy)
        yaw = 0.0

    return np.array([x, y, z, roll, pitch, yaw])


# ---------------------------------------------------------------------------
# DH helpers
# ---------------------------------------------------------------------------
def dh_to_transform(a: float, d: float, alpha: float, theta: float = 0.0) -> Mat4:
    """Build the 4x4 homogeneous transform from standard DH parameters."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca, st * sa, a * ct],
        [st, ct * ca, -ct * sa, a * st],
        [0.0, sa, ca, d],
        [0.0, 0.0, 0.0, 1.0],
    ])


def convert_dh_to_poe(
    config: RobotConfig,
) -> tuple[NDArray[np.floating], Mat4]:
    """Derive POE screw axes (6 x N) and home configuration M from a ``RobotConfig``.

    Args:
        config: Robot configuration containing DH parameters and tool offset.

    Returns:
        S_list: (6, N) matrix whose columns are the spatial screw axes.
        M: 4x4 home configuration of the end-effector.
    """
    T_cumulative = np.eye(4)
    screws: list[VecN] = []
    local_z = np.array([0.0, 0.0, 1.0])

    for a, d, alpha in config.dh_params:
        R = T_cumulative[:3, :3]
        p = T_cumulative[:3, 3]

        omega = R @ local_z              # joint rotation axis in base frame
        v = -np.cross(omega, p)           # linear velocity component
        screws.append(np.concatenate([omega, v]))

        T_cumulative = T_cumulative @ dh_to_transform(a, d, alpha)

    # Home configuration includes tool offset
    T_tool = np.eye(4)
    T_tool[:3, 3] = config.tool_offset
    M = T_cumulative @ T_tool

    return np.array(screws).T, M


# ---------------------------------------------------------------------------
# Main solver class
# ---------------------------------------------------------------------------
class RobotKinematicsPOE:
    """Product-of-Exponentials kinematic solver for an arbitrary serial robot.

    Args:
        config: A ``RobotConfig`` describing the robot's DH parameters and
                tool offset.

    Attributes:
        config: The robot configuration used to build this solver.
        screw_axes: (6, N) spatial screw axes derived from DH parameters.
        home_config: 4x4 end-effector pose when all joints are zero.
    """

    def __init__(self, config: RobotConfig) -> None:
        self.config = config
        self.screw_axes, self.home_config = convert_dh_to_poe(config)

    # -- Forward kinematics ---------------------------------------------------

    def forward_kinematics(self, joints: Sequence[float] | NDArray) -> Mat4:
        """Compute the end-effector pose via POE.

        Formula: T = exp([S1]*q1) ... exp([Sn]*qn) * M

        Args:
            joints: Joint angles (rad), length ``config.num_joints``.

        Returns:
            4x4 homogeneous transform of the end-effector.
        """
        T = np.eye(4)
        for i, q in enumerate(joints):
            T = T @ matrix_exp6(self.screw_axes[:, i], q)
        return T @ self.home_config

    # -- Jacobian --------------------------------------------------------------

    def jacobian_space(self, joints: Sequence[float] | NDArray) -> MatMN:
        """Return the 6xN spatial Jacobian at the given joint configuration.

        Args:
            joints: Joint angles (rad), length ``config.num_joints``.
        """
        n = self.config.num_joints
        Js = np.zeros((6, n))
        Js[:, 0] = self.screw_axes[:, 0]

        T = np.eye(4)
        for i in range(1, n):
            T = T @ matrix_exp6(self.screw_axes[:, i - 1], joints[i - 1])
            Js[:, i] = adjoint(T) @ self.screw_axes[:, i]

        return Js

    # -- Inverse kinematics ----------------------------------------------------

    def inverse_kinematics(
        self,
        target: Mat4,
        initial_guess: NDArray | None = None,
        max_iter: int = IK_DEFAULT_MAX_ITER,
        tol: float = IK_DEFAULT_TOLERANCE,
    ) -> IKResult:
        """Iterative inverse kinematics using damped least-squares Newton-Raphson.

        Args:
            target: Desired 4x4 end-effector pose.
            initial_guess: Starting joint angles; defaults to zeros.
            max_iter: Maximum Newton-Raphson iterations.
            tol: Convergence tolerance on the twist-error norm.

        Returns:
            An ``IKResult`` with the solved joint angles and convergence flag.
        """
        n = self.config.num_joints
        theta = (
            np.zeros(n, dtype=float)
            if initial_guess is None
            else np.array(initial_guess, dtype=float).copy()
        )

        for _ in range(max_iter):
            T_current = self.forward_kinematics(theta)

            # Body-frame error: T_body_err = T_current^{-1} * T_target
            T_body_err = np.linalg.inv(T_current) @ target

            # Extract body twist from the error transform
            omega_err = rotation_error(T_body_err[:3, :3])
            v_err = T_body_err[:3, 3]
            twist_body = np.concatenate([omega_err, v_err])

            if np.linalg.norm(twist_body) < tol:
                theta = np.mod(theta + np.pi, 2.0 * np.pi) - np.pi
                return IKResult(joints=theta, converged=True)

            # Map body twist to spatial frame and solve with spatial Jacobian
            Js = self.jacobian_space(theta)
            twist_space = adjoint(T_current) @ twist_body

            # Damped least-squares pseudo-inverse of the spatial Jacobian
            JtJ = Js.T @ Js + IK_DAMPING_FACTOR ** 2 * np.eye(n)
            theta += np.linalg.solve(JtJ, Js.T @ twist_space)

        # Normalize to [-pi, pi]
        theta = np.mod(theta + np.pi, 2.0 * np.pi) - np.pi
        return IKResult(joints=theta, converged=False)


# ---------------------------------------------------------------------------
# Backward-compatible alias
# ---------------------------------------------------------------------------
class UR5eKinematicsPOE(RobotKinematicsPOE):
    """Convenience subclass pre-configured for the UR5e."""

    def __init__(self) -> None:
        super().__init__(UR5E_CONFIG)


# ---------------------------------------------------------------------------
# Self-test
# ---------------------------------------------------------------------------
def _run_self_test(config: RobotConfig, test_joints: list[float]) -> None:
    """Run FK → IK round-trip test for a given robot configuration."""
    solver = RobotKinematicsPOE(config)

    print(f"=== {config.name} ({config.num_joints}-DOF) ===")
    print("Home configuration (M):")
    print(np.round(solver.home_config, 3))

    fk_pose = solver.forward_kinematics(test_joints)
    print(f"\nFK result (joints={np.round(test_joints, 3).tolist()}):")
    print(np.round(fk_pose, 3))
    print(f"Pose: {np.round(transform_to_pose(fk_pose), 4)}")

    ik_result = solver.inverse_kinematics(fk_pose)
    if ik_result.converged:
        print(f"\nIK converged — joints: {np.round(ik_result.joints, 3).tolist()}")
        fk_verify = solver.forward_kinematics(ik_result.joints)
        pos_err = np.linalg.norm(fk_verify[:3, 3] - fk_pose[:3, 3])
        rot_err = np.linalg.norm(fk_verify[:3, :3] - fk_pose[:3, :3])
        print(f"Position error: {pos_err:.6e}")
        print(f"Rotation error: {rot_err:.6e}")
    else:
        print("\nIK did not converge.")
    print()


if __name__ == "__main__":
    # UR5e (6-DOF)
    _run_self_test(
        UR5E_CONFIG,
        test_joints=[0.0, -2.0, 2.0, 0.0, -1.57, -np.pi / 2],
    )

    # Franka Emika Panda (7-DOF)
    _run_self_test(
        FRANKA_CONFIG,
        test_joints=[0.0, -0.5, 0.0, -2.0, 0.0, 1.5, 0.0],
    )