"""Inverse kinematics implementation for OpenArm using IKPy library."""

import ikpy.chain
import numpy as np

from openarm.kinematics.models import OPENARM_URDF_PATH

# Constants for joint configuration
_NUM_REVOLUTE_JOINTS = 7
_NUM_TOTAL_LINKS = 12


class IkpyInverseKinematics:
    """Inverse kinematics solver for OpenArm robot using IKPy."""

    def __init__(self, urdf_path: str | None = None) -> None:
        """Initialize the inverse kinematics solver.

        Args:
            urdf_path: Path to URDF file. If None, uses default OpenArm URDF.

        """
        if urdf_path is None:
            urdf_path = OPENARM_URDF_PATH

        # Active links mask: Only joints 1-7 are revolute, others are fixed
        # Index 0: Base link (fixed)
        # Index 1-7: openarm_left/right_joint1-7 (revolute)
        # Index 8+: Fixed joints (joint8, hand_joint, tcp_joint, last_joint)
        active_links_mask = (
            [False]
            + [True] * _NUM_REVOLUTE_JOINTS
            + [False] * (_NUM_TOTAL_LINKS - _NUM_REVOLUTE_JOINTS - 1)
        )

        # Initialize chains for both arms
        self._left_chain = ikpy.chain.Chain.from_urdf_file(
            urdf_path,
            base_elements=["openarm_left_link0"],
            # TCP offset from hand to tool center point (8cm in Z-direction)
            # This matches the openarm_left_hand_tcp_joint transform in URDF
            last_link_vector=[0, 0, 0.08],
            name="openarm_left_arm",
            active_links_mask=active_links_mask,
        )

        self._right_chain = ikpy.chain.Chain.from_urdf_file(
            urdf_path,
            base_elements=["openarm_right_link0"],
            # TCP offset from hand to tool center point (8cm in Z-direction)
            # This matches the openarm_right_hand_tcp_joint transform in URDF
            last_link_vector=[0, 0, 0.08],
            name="openarm_right_arm",
            active_links_mask=active_links_mask,
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
            initial_position: Initial joint angles guess for 7 revolute joints.
                If None, uses current joint positions.

        Returns:
            Array of 7 joint angles in radians for revolute joints

        """
        return self._solve_arm(
            self._left_chain, target_position, target_orientation, initial_position
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
            initial_position: Initial joint angles guess for 7 revolute joints.
                If None, uses current joint positions.

        Returns:
            Array of 7 joint angles in radians for revolute joints

        """
        return self._solve_arm(
            self._right_chain, target_position, target_orientation, initial_position
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

        # Handle initial position: convert 7-element array to 12-element for IKPy
        if initial_position is not None:
            if len(initial_position) != _NUM_REVOLUTE_JOINTS:
                msg = (
                    f"initial_position must have {_NUM_REVOLUTE_JOINTS} elements "
                    f"for {_NUM_REVOLUTE_JOINTS} revolute joints, "
                    f"got {len(initial_position)}"
                )
                raise ValueError(msg)
            expanded_initial = np.zeros(_NUM_TOTAL_LINKS)
            expanded_initial[1 : 1 + _NUM_REVOLUTE_JOINTS] = initial_position
            initial_position = expanded_initial

        # Solve IK
        joint_angles = chain.inverse_kinematics_frame(
            target_matrix,
            initial_position=initial_position,
        )

        # Return only the revolute joint angles (indices 1 to 1+_NUM_REVOLUTE_JOINTS)
        return joint_angles[1 : 1 + _NUM_REVOLUTE_JOINTS]
