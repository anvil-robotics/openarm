"""Robot models and URDF descriptions for OpenArm kinematics.

This module provides paths and constants for accessing robot model files,
particularly the URDF description used for kinematic calculations.
"""

from pathlib import Path

OPENARM_URDF_PATH = Path(__file__).parent / "openarm.urdf"
