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

    def get_left_arm_velocities(self) -> list[float]:
        """Get current joint velocities for the left arm.

        Returns:
            List of joint velocities in rad/s for all 7 left arm joints.

        """
        return [
            self._get_actuator_velocity(actuator)
            for actuator in self._left_arm_actuators
        ]

    def set_left_arm_torques(self, torques: Sequence[float]) -> None:
        """Apply torques to all left arm joints.

        Args:
            torques: Sequence of 7 torque values in N⋅m for each joint.

        Raises:
            ValueError: If torques sequence length doesn't match number of joints.

        """
        if len(torques) != len(self._left_arm_actuators):
            msg = (
                f"Expected {len(self._left_arm_actuators)} torques, got {len(torques)}"
            )
            raise ValueError(msg)

        for actuator, torque in zip(self._left_arm_actuators, torques, strict=False):
            self._set_actuator_torque(actuator, torque)

    def set_left_arm_position_control(
        self, target_positions: Sequence[float], kp: float = 100.0, kd: float = 10.0
    ) -> None:
        """Apply PD position control to all left arm joints.

        Args:
            target_positions: Sequence of 7 target positions in radians.
            kp: Proportional gain for position error.
            kd: Derivative gain for velocity error.

        Raises:
            ValueError: If target_positions sequence length doesn't match
                number of joints.

        """
        if len(target_positions) != len(self._left_arm_actuators):
            msg = (
                f"Expected {len(self._left_arm_actuators)} positions, "
                f"got {len(target_positions)}"
            )
            raise ValueError(msg)

        for actuator, target_pos in zip(
            self._left_arm_actuators, target_positions, strict=False
        ):
            self._set_actuator_position_control(actuator, target_pos, kp, kd)

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

    def get_right_arm_velocities(self) -> list[float]:
        """Get current joint velocities for the right arm.

        Returns:
            List of joint velocities in rad/s for all 7 right arm joints.

        """
        return [
            self._get_actuator_velocity(actuator)
            for actuator in self._right_arm_actuators
        ]

    def set_right_arm_torques(self, torques: Sequence[float]) -> None:
        """Apply torques to all right arm joints.

        Args:
            torques: Sequence of 7 torque values in N⋅m for each joint.

        Raises:
            ValueError: If torques sequence length doesn't match number of joints.

        """
        if len(torques) != len(self._right_arm_actuators):
            msg = (
                f"Expected {len(self._right_arm_actuators)} torques, got {len(torques)}"
            )
            raise ValueError(msg)

        for actuator, torque in zip(self._right_arm_actuators, torques, strict=False):
            self._set_actuator_torque(actuator, torque)

    def set_right_arm_position_control(
        self, target_positions: Sequence[float], kp: float = 100.0, kd: float = 10.0
    ) -> None:
        """Apply PD position control to all right arm joints.

        Args:
            target_positions: Sequence of 7 target positions in radians.
            kp: Proportional gain for position error.
            kd: Derivative gain for velocity error.

        Raises:
            ValueError: If target_positions sequence length doesn't match
                number of joints.

        """
        if len(target_positions) != len(self._right_arm_actuators):
            msg = (
                f"Expected {len(self._right_arm_actuators)} positions, "
                f"got {len(target_positions)}"
            )
            raise ValueError(msg)

        for actuator, target_pos in zip(
            self._right_arm_actuators, target_positions, strict=False
        ):
            self._set_actuator_position_control(actuator, target_pos, kp, kd)

    # Left gripper control interface
    def get_left_gripper_position(self) -> float:
        """Get current position of the left gripper.

        Returns:
            Gripper position in meters (0.0 = closed, positive = open).

        """
        return self._get_actuator_position(self._left_gripper_actuator)

    def get_left_gripper_velocity(self) -> float:
        """Get current velocity of the left gripper.

        Returns:
            Gripper velocity in m/s.

        """
        return self._get_actuator_velocity(self._left_gripper_actuator)

    def set_left_gripper_torque(self, torque: float) -> None:
        """Apply torque to the left gripper.

        Args:
            torque: Torque value in N⋅m (positive = open, negative = close).

        """
        self._set_actuator_torque(self._left_gripper_actuator, torque)

    def set_left_gripper_position_control(
        self, target_position: float, kp: float = 100.0, kd: float = 10.0
    ) -> None:
        """Apply PD position control to the left gripper.

        Args:
            target_position: Target gripper position in meters.
            kp: Proportional gain for position error.
            kd: Derivative gain for velocity error.

        """
        self._set_actuator_position_control(
            self._left_gripper_actuator, target_position, kp, kd
        )

    # Right gripper control interface
    def get_right_gripper_position(self) -> float:
        """Get current position of the right gripper.

        Returns:
            Gripper position in meters (0.0 = closed, positive = open).

        """
        return self._get_actuator_position(self._right_gripper_actuator)

    def get_right_gripper_velocity(self) -> float:
        """Get current velocity of the right gripper.

        Returns:
            Gripper velocity in m/s.

        """
        return self._get_actuator_velocity(self._right_gripper_actuator)

    def set_right_gripper_torque(self, torque: float) -> None:
        """Apply torque to the right gripper.

        Args:
            torque: Torque value in N⋅m (positive = open, negative = close).

        """
        self._set_actuator_torque(self._right_gripper_actuator, torque)

    def set_right_gripper_position_control(
        self, target_position: float, kp: float = 100.0, kd: float = 10.0
    ) -> None:
        """Apply PD position control to the right gripper.

        Args:
            target_position: Target gripper position in meters.
            kp: Proportional gain for position error.
            kd: Derivative gain for velocity error.

        """
        self._set_actuator_position_control(
            self._right_gripper_actuator, target_position, kp, kd
        )

    def _get_actuator_position(
        self, actuator: mujoco._structs._MjModelActuatorViews
    ) -> float:
        joint_id = self.model.actuator_trnid[actuator.id][0]
        return self.data.qpos[joint_id]

    def _get_actuator_velocity(
        self, actuator: mujoco._structs._MjModelActuatorViews
    ) -> float:
        joint_id = self.model.actuator_trnid[actuator.id][0]
        return self.data.qvel[joint_id]

    def _set_actuator_torque(
        self, actuator: mujoco._structs._MjModelActuatorViews, torque: float
    ) -> None:
        self.data.ctrl[actuator.id] = torque

    def _set_actuator_position_control(
        self,
        actuator: mujoco._structs._MjModelActuatorViews,
        target_position: float,
        kp: float,
        kd: float,
    ) -> None:
        current_pos = self._get_actuator_position(actuator)
        current_vel = self._get_actuator_velocity(actuator)

        pos_error = target_position - current_pos
        vel_error = -current_vel  # Target velocity is 0

        torque = kp * pos_error + kd * vel_error
        self._set_actuator_torque(actuator, torque)

    def step(self) -> None:
        """Advance the simulation by one timestep.

        Executes one integration step of the physics simulation using MuJoCo's
        internal timestep. Should be called after setting control inputs.
        """
        mujoco.mj_step(self.model, self.data)


__all__ = ["OpenArmSimulation"]
