"""Inverse kinematics implementations for OpenArm.

This module contains various inverse kinematics solvers for calculating
joint angles from desired end-effector poses.
"""

from .ikpy import IkpyInverseKinematics

__all__ = ["IkpyInverseKinematics"]
