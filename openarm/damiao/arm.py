"""Arm control module for coordinating multiple Damiao motors."""

import asyncio

from .encoding import (
    ControlMode,
    MitControlParams,
    MotorState,
    PosForceControlParams,
    PosVelControlParams,
    SaveResponse,
    VelControlParams,
)
from .motor import Motor


class Arm:
    """Coordinate control of multiple Damiao motors as a single arm."""

    def __init__(self, motors: list[Motor]) -> None:
        """Initialize Arm with a list of motors.

        Args:
            motors: List of Motor instances to control as a group.

        """
        self.motors = motors

    async def get_control_mode(self) -> list[ControlMode]:
        """Get control mode for all motors."""
        return await asyncio.gather(
            *[motor.get_control_mode() for motor in self.motors]
        )

    async def set_control_mode(self, mode: ControlMode) -> list[ControlMode]:
        """Set control mode for all motors."""
        return await asyncio.gather(
            *[motor.set_control_mode(mode) for motor in self.motors]
        )

    async def control_mit(
        self,
        *,
        kp: float | list[float],
        kd: float | list[float],
        q: float | list[float],
        dq: float | list[float],
        tau: float | list[float],
    ) -> list[MotorState]:
        """Control all motors in MIT mode.

        Args:
            kp: Proportional gain (single value or list per motor)
            kd: Derivative gain (single value or list per motor)
            q: Position (single value or list per motor)
            dq: Velocity (single value or list per motor)
            tau: Torque (single value or list per motor)

        Returns:
            List of MotorState from all motors

        """
        return await asyncio.gather(
            *[
                motor.control_mit(
                    MitControlParams(
                        kp=kp[i] if isinstance(kp, list) else kp,
                        kd=kd[i] if isinstance(kd, list) else kd,
                        q=q[i] if isinstance(q, list) else q,
                        dq=dq[i] if isinstance(dq, list) else dq,
                        tau=tau[i] if isinstance(tau, list) else tau,
                    )
                )
                for i, motor in enumerate(self.motors)
            ]
        )

    async def control_pos_vel(
        self,
        *,
        position: float | list[float],
        velocity: float | list[float],
    ) -> list[MotorState]:
        """Control all motors in position/velocity mode.

        Args:
            position: Target position (single value or list per motor)
            velocity: Target velocity (single value or list per motor)

        Returns:
            List of MotorState from all motors

        """
        return await asyncio.gather(
            *[
                motor.control_pos_vel(
                    PosVelControlParams(
                        position=position[i]
                        if isinstance(position, list)
                        else position,
                        velocity=velocity[i]
                        if isinstance(velocity, list)
                        else velocity,
                    )
                )
                for i, motor in enumerate(self.motors)
            ]
        )

    async def control_vel(
        self,
        *,
        velocity: float | list[float],
    ) -> list[MotorState]:
        """Control all motors in velocity mode.

        Args:
            velocity: Target velocity (single value or list per motor)

        Returns:
            List of MotorState from all motors

        """
        return await asyncio.gather(
            *[
                motor.control_vel(
                    VelControlParams(
                        velocity=velocity[i]
                        if isinstance(velocity, list)
                        else velocity,
                    )
                )
                for i, motor in enumerate(self.motors)
            ]
        )

    async def control_pos_force(
        self,
        *,
        position: float | list[float],
        torque: float | list[float],
    ) -> list[MotorState]:
        """Control all motors in position/force mode.

        Args:
            position: Target position (single value or list per motor)
            torque: Target torque (single value or list per motor)

        Returns:
            List of MotorState from all motors

        """
        return await asyncio.gather(
            *[
                motor.control_pos_force(
                    PosForceControlParams(
                        position=position[i]
                        if isinstance(position, list)
                        else position,
                        torque=torque[i] if isinstance(torque, list) else torque,
                    )
                )
                for i, motor in enumerate(self.motors)
            ]
        )

    async def enable(self) -> list[MotorState]:
        """Enable all motors."""
        return await asyncio.gather(*[motor.enable() for motor in self.motors])

    async def disable(self) -> list[MotorState]:
        """Disable all motors."""
        return await asyncio.gather(*[motor.disable() for motor in self.motors])

    async def set_zero_position(self) -> list[MotorState]:
        """Set zero position for all motors."""
        return await asyncio.gather(
            *[motor.set_zero_position() for motor in self.motors]
        )

    async def save_parameters(self) -> list[SaveResponse]:
        """Save parameters to flash for all motors."""
        return await asyncio.gather(*[motor.save_parameters() for motor in self.motors])

    async def refresh_status(self) -> list[MotorState]:
        """Refresh status for all motors."""
        return await asyncio.gather(*[motor.refresh_status() for motor in self.motors])
