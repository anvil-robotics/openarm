"""Inverse kinematics implementation for OpenArm using IKPy library."""

import ikpy.chain
import numpy as np

from openarm.kinematics.models import OPENARM_URDF_PATH


class IkpyInverseKinematics:
    """Inverse kinematics solver for OpenArm robot using IKPy."""

    def __init__(self, urdf_path: str | None = None) -> None:
        """Initialize the inverse kinematics solver.

        Args:
            urdf_path: Path to URDF file. If None, uses default OpenArm URDF.

        """
        if urdf_path is None:
            urdf_path = OPENARM_URDF_PATH

        # Initialize chains for both arms
        self.left_chain = ikpy.chain.Chain.from_urdf_file(
            urdf_path,
            base_elements=["openarm_left_link0"],
            # TCP offset from hand to tool center point (8cm in Z-direction)
            # This matches the openarm_left_hand_tcp_joint transform in URDF
            last_link_vector=[0, 0, 0.08],
            name="openarm_left_arm",
        )

        self.right_chain = ikpy.chain.Chain.from_urdf_file(
            urdf_path,
            base_elements=["openarm_right_link0"],
            # TCP offset from hand to tool center point (8cm in Z-direction)
            # This matches the openarm_right_hand_tcp_joint transform in URDF
            last_link_vector=[0, 0, 0.08],
            name="openarm_right_arm",
        )

    def solve_left_arm(
        self,
        target_position: np.ndarray,
        target_orientation: np.ndarray | None = None,
        initial_position: np.ndarray | None = None,
    ) -> np.ndarray:
        """Solve inverse kinematics for the left arm.

        Args:
            target_position: Target position [x, y, z] in meters.
            target_orientation: Target orientation as rotation matrix (3x3)
                or None for position-only IK.
            initial_position: Initial joint angles guess. If None, uses
                current joint positions.

        Returns:
            Array of joint angles in radians

        """
        return self._solve_arm(
            self.left_chain, target_position, target_orientation, initial_position
        )

    def solve_right_arm(
        self,
        target_position: np.ndarray,
        target_orientation: np.ndarray | None = None,
        initial_position: np.ndarray | None = None,
    ) -> np.ndarray:
        """Solve inverse kinematics for the right arm.

        Args:
            target_position: Target position [x, y, z] in meters.
            target_orientation: Target orientation as rotation matrix (3x3)
                or None for position-only IK.
            initial_position: Initial joint angles guess. If None, uses
                current joint positions.

        Returns:
            Array of joint angles in radians

        """
        return self._solve_arm(
            self.right_chain, target_position, target_orientation, initial_position
        )

    def _solve_arm(
        self,
        chain: ikpy.chain.Chain,
        target_position: np.ndarray,
        target_orientation: np.ndarray | None = None,
        initial_position: np.ndarray | None = None,
    ) -> np.ndarray:
        # Convert target position to homogeneous transformation matrix
        if target_orientation is None:
            target_orientation = np.eye(3)

        target_matrix = np.eye(4)
        target_matrix[:3, :3] = target_orientation
        target_matrix[:3, 3] = target_position

        # Solve IK
        return chain.inverse_kinematics_frame(
            target_matrix,
            initial_position=initial_position,
        )
