"""
Forward and inverse kinematics for a serial chain read out of a URDF.

The arm's geometry lives in the URDF only. This module walks the joints from
a base link to a tip link and works out everything else numerically, so the
arm model can be swapped without touching the control code.

No ROS dependency: it takes the URDF as a string.
"""

import xml.etree.ElementTree as ET
from dataclasses import dataclass

import numpy as np


def _rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def _axis_angle_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
    x, y, z = axis
    c, s = np.cos(angle), np.sin(angle)
    k = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + s * k + (1 - c) * (k @ k)


def _floats(text: str | None, default: str) -> list[float]:
    return [float(v) for v in (text or default).split()]


@dataclass
class Joint:
    name: str
    origin: np.ndarray  # 4x4 transform from the parent link to the joint frame
    axis: np.ndarray | None  # unit axis in the joint frame; None for a fixed joint
    lower: float = 0.0
    upper: float = 0.0


def revolute_limits(urdf_xml: str) -> dict[str, tuple[float, float]]:
    """(lower, upper) of every revolute joint in the URDF, in document order."""
    limits = {}
    for joint in ET.fromstring(urdf_xml).findall("joint"):
        if joint.get("type") == "revolute":
            limit = joint.find("limit")
            limits[joint.get("name")] = (float(limit.get("lower")), float(limit.get("upper")))
    return limits


class Chain:
    """The joints between `base` and `tip`, in order from the base."""

    def __init__(self, urdf_xml: str, base: str, tip: str):
        by_child = {j.find("child").get("link"): j for j in ET.fromstring(urdf_xml).findall("joint")}
        joints = []
        link = tip
        while link != base:
            if link not in by_child:
                raise ValueError(f"no chain from {base!r} to {tip!r} in the URDF")
            element = by_child[link]
            joints.append(self._parse(element))
            link = element.find("parent").get("link")
        self.joints: list[Joint] = joints[::-1]
        self.moving = [j for j in self.joints if j.axis is not None]
        self.names = [j.name for j in self.moving]
        self.lower = np.array([j.lower for j in self.moving])
        self.upper = np.array([j.upper for j in self.moving])

    @staticmethod
    def _parse(element) -> Joint:
        origin = element.find("origin")
        transform = np.eye(4)
        if origin is not None:
            transform[:3, :3] = _rpy_matrix(*_floats(origin.get("rpy"), "0 0 0"))
            transform[:3, 3] = _floats(origin.get("xyz"), "0 0 0")
        kind = element.get("type")
        if kind == "fixed":
            return Joint(element.get("name"), transform, None)
        if kind != "revolute":
            raise ValueError(f"joint {element.get('name')!r}: only revolute and fixed joints are supported")
        axis_element = element.find("axis")
        axis = np.array(_floats(axis_element.get("xyz") if axis_element is not None else None, "1 0 0"))
        limit = element.find("limit")
        return Joint(element.get("name"), transform, axis / np.linalg.norm(axis),
                     float(limit.get("lower")), float(limit.get("upper")))

    def forward(self, q) -> tuple[np.ndarray, np.ndarray]:
        """Pose of the tip in the base frame (4x4) and the 6xN geometric Jacobian."""
        transform = np.eye(4)
        axes, points = [], []
        angles = iter(q)
        for joint in self.joints:
            transform = transform @ joint.origin
            if joint.axis is None:
                continue
            axes.append(transform[:3, :3] @ joint.axis)
            points.append(transform[:3, 3].copy())
            rotation = np.eye(4)
            rotation[:3, :3] = _axis_angle_matrix(joint.axis, next(angles))
            transform = transform @ rotation
        tip = transform[:3, 3]
        jacobian = np.zeros((6, len(axes)))
        for i, (axis, point) in enumerate(zip(axes, points)):
            jacobian[:3, i] = np.cross(axis, tip - point)
            jacobian[3:, i] = axis
        return transform, jacobian

    def ik_step(self, q, position, rotation, rest=None, damping: float = 0.02,
                max_step: float = 0.15) -> np.ndarray:
        """
        One damped-least-squares step towards a tip pose.

        The arm has one joint more than the pose needs. That spare freedom is
        spent pulling the arm towards the `rest` posture, which keeps the
        elbow from drifting over time.
        """
        q = np.asarray(q, dtype=float)
        transform, jacobian = self.forward(q)
        error = np.zeros(6)
        error[:3] = position - transform[:3, 3]
        current = transform[:3, :3]
        error[3:] = 0.5 * sum(np.cross(current[:, i], rotation[:, i]) for i in range(3))

        inverse = jacobian.T @ np.linalg.inv(jacobian @ jacobian.T + damping ** 2 * np.eye(6))
        step = inverse @ error
        if rest is not None:
            step += (np.eye(len(q)) - inverse @ jacobian) @ (0.1 * (np.asarray(rest) - q))
        largest = np.abs(step).max()
        if largest > max_step:
            step *= max_step / largest
        return np.clip(q + step, self.lower, self.upper)

    def solve(self, q, position, rotation, rest=None, iterations: int = 200) -> np.ndarray:
        """Iterate `ik_step` from the seed `q`. Used once, to find the home posture."""
        for _ in range(iterations):
            q = self.ik_step(q, position, rotation, rest)
        return q
