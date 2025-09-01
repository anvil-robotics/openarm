"""OpenArm kinematics package.

Provides forward and inverse kinematics solutions for dual-arm robotic systems.
Includes URDF-based modeling and IKPy integration for kinematic calculations.
"""

from . import inverse, models

__all__ = ["inverse", "models"]
