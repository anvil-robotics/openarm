"""OpenArm simulation wrapper and control utilities.

This module provides a complete simulation environment for the OpenArm robot,
including model loading and joint control for both left and right arms.
"""

from collections.abc import Sequence
from pathlib import Path

import mujoco

from .models import OPENARM_MODEL_PATH


class OpenArmSimulation:
    """Complete OpenArm simulation environment with joint control."""

    def __init__(self, model_path: str | Path | None = None) -> None:
        """Initialize the OpenArm simulation.

        Args:
            model_path: Path to the MuJoCo XML model file. If None,
                uses default OpenArm model.

        """
        if model_path is None:
            model_path = OPENARM_MODEL_PATH

        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)

        # Store actuator references for efficient access
        self._left_arm_actuators = [
            self.model.actuator(f"left_joint{i}_ctrl") for i in range(1, 8)
        ]
        self._right_arm_actuators = [
            self.model.actuator(f"right_joint{i}_ctrl") for i in range(1, 8)
        ]
        self._left_gripper_actuator = self.model.actuator("left_finger")
        self._right_gripper_actuator = self.model.actuator("right_finger")

    # Left arm control interface
    def get_left_arm_positions(self) -> list[float]:
        """Get current joint positions for the left arm.

        Returns:
            List of joint positions in radians for all 7 left arm joints.

        """
        return [
            self._get_actuator_position(actuator)
            for actuator in self._left_arm_actuators
        ]

    def set_left_arm_positions(self, positions: Sequence[float]) -> None:
        """Set target positions for all left arm joints.

        Args:
            positions: Sequence of 7 target positions in radians.

        Raises:
            ValueError: If positions sequence length doesn't match number of joints.

        """
        if len(positions) != len(self._left_arm_actuators):
            msg = (
                f"Expected {len(self._left_arm_actuators)} positions, got {len(positions)}"
            )
            raise ValueError(msg)

        for actuator, position in zip(self._left_arm_actuators, positions, strict=False):
            self.data.ctrl[actuator.id] = position

    # Right arm control interface
    def get_right_arm_positions(self) -> list[float]:
        """Get current joint positions for the right arm.

        Returns:
            List of joint positions in radians for all 7 right arm joints.

        """
        return [
            self._get_actuator_position(actuator)
            for actuator in self._right_arm_actuators
        ]

    def set_right_arm_positions(self, positions: Sequence[float]) -> None:
        """Set target positions for all right arm joints.

        Args:
            positions: Sequence of 7 target positions in radians.

        Raises:
            ValueError: If positions sequence length doesn't match number of joints.

        """
        if len(positions) != len(self._right_arm_actuators):
            msg = (
                f"Expected {len(self._right_arm_actuators)} positions, got {len(positions)}"
            )
            raise ValueError(msg)

        for actuator, position in zip(self._right_arm_actuators, positions, strict=False):
            self.data.ctrl[actuator.id] = position

    # Left gripper control interface
    def get_left_gripper_position(self) -> float:
        """Get current position of the left gripper.

        Returns:
            Gripper position in meters (0.0 = closed, positive = open).

        """
        return self._get_actuator_position(self._left_gripper_actuator)

    def set_left_gripper_position(self, position: float) -> None:
        """Set target position for the left gripper.

        Args:
            position: Target gripper position in meters (0.0 = closed, positive = open).

        """
        self.data.ctrl[self._left_gripper_actuator.id] = position

    # Right gripper control interface
    def get_right_gripper_position(self) -> float:
        """Get current position of the right gripper.

        Returns:
            Gripper position in meters (0.0 = closed, positive = open).

        """
        return self._get_actuator_position(self._right_gripper_actuator)

    def set_right_gripper_position(self, position: float) -> None:
        """Set target position for the right gripper.

        Args:
            position: Target gripper position in meters (0.0 = closed, positive = open).

        """
        self.data.ctrl[self._right_gripper_actuator.id] = position

    def _get_actuator_position(
        self, actuator: mujoco._structs._MjModelActuatorViews
    ) -> float:
        joint_id = self.model.actuator_trnid[actuator.id][0]
        return self.data.qpos[joint_id]

    def step(self) -> None:
        """Advance the simulation by one timestep.

        Executes one integration step of the physics simulation using MuJoCo's
        internal timestep. Should be called after setting control inputs.
        """
        mujoco.mj_step(self.model, self.data)


__all__ = ["OpenArmSimulation"]
