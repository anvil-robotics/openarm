"""Damiao motor control package for OpenArm.

High-level interface for controlling Damiao motors through CAN bus communication.
This module provides the main Motor class that orchestrates encode/decode operations
from the low-level encoding.py implementation.

Reference: README.md High-Level Motor Class section for architecture details.
"""

from collections.abc import Coroutine
from enum import StrEnum
from typing import Any

from openarm.bus import Bus

from .encoding import (
    AckResponse,
    ControlMode,
    MitControlParams,
    MotorLimits,
    MotorState,
    PosForceControlParams,
    PosVelControlParams,
    RegisterAddress,
    RegisterResponse,
    VelControlParams,
    decode_acknowledgment,
    decode_motor_state,
    decode_register_value,
    encode_control_mit,
    encode_control_pos_force,
    encode_control_pos_vel,
    encode_control_vel,
    encode_disable_motor,
    encode_enable_motor,
    encode_enable_motor_legacy,
    encode_read_register,
    encode_refresh_status,
    encode_save_parameters,
    encode_set_control_mode,
    encode_set_zero_position,
    encode_write_register_float,
    encode_write_register_int,
)

__version__ = "0.1.0"


class MotorType(StrEnum):
    """Enumeration of Damiao motor types.

    Reference: DM_CAN.py DM_Motor_Type enum and Limit_Param array lines 65-69
    """

    DM4310 = "DM4310"
    DM4310_48V = "DM4310_48V"
    DM4340 = "DM4340"
    DM4340_48V = "DM4340_48V"
    DM6006 = "DM6006"
    DM8006 = "DM8006"
    DM8009 = "DM8009"
    DM10010L = "DM10010L"
    DM10010 = "DM10010"
    DMH3510 = "DMH3510"
    DMH6215 = "DMH6215"
    DMG6220 = "DMG6220"


# Motor limit configurations for all Damiao motor types
# Reference: DM_CAN.py Limit_Param array structure lines 65-69
MOTOR_LIMITS = {
    MotorType.DM4310: MotorLimits(q_max=12.5, dq_max=30.0, tau_max=10.0),
    MotorType.DM4310_48V: MotorLimits(q_max=12.5, dq_max=50.0, tau_max=10.0),
    MotorType.DM4340: MotorLimits(q_max=12.5, dq_max=8.0, tau_max=28.0),
    MotorType.DM4340_48V: MotorLimits(q_max=12.5, dq_max=10.0, tau_max=28.0),
    MotorType.DM6006: MotorLimits(q_max=12.5, dq_max=45.0, tau_max=20.0),
    MotorType.DM8006: MotorLimits(q_max=12.5, dq_max=45.0, tau_max=40.0),
    MotorType.DM8009: MotorLimits(q_max=12.5, dq_max=45.0, tau_max=54.0),
    MotorType.DM10010L: MotorLimits(q_max=12.5, dq_max=25.0, tau_max=200.0),
    MotorType.DM10010: MotorLimits(q_max=12.5, dq_max=20.0, tau_max=200.0),
    MotorType.DMH3510: MotorLimits(q_max=12.5, dq_max=280.0, tau_max=1.0),
    MotorType.DMH6215: MotorLimits(q_max=12.5, dq_max=45.0, tau_max=10.0),
    MotorType.DMG6220: MotorLimits(q_max=12.5, dq_max=45.0, tau_max=10.0),
}


class Motor:
    """High-level interface for controlling a Damiao motor.

    This class combines encode/decode functions from encoding.py to provide
    a convenient interface that follows the request-response pattern.

    Reference: README.md High-Level Motor Class section lines 320-345
    """

    def __init__(self, bus: Bus, motor_id: int, motor_type: MotorType) -> None:
        """Initialize a Motor instance.

        Args:
            bus: CAN bus instance for message transmission and reception
            motor_id: Motor slave ID for CAN communication
            motor_type: Motor type enum for automatic limit configuration

        Reference: README.md Motor class constructor pattern lines 330-332

        """
        self.bus = bus
        self.motor_id = motor_id
        self.motor_type = motor_type
        self.motor_limits = MOTOR_LIMITS[motor_type]


    def set_control_mode(
        self, mode: ControlMode
    ) -> Coroutine[Any, Any, RegisterResponse]:
        """Set motor control mode. Returns coroutine to be awaited.

        Args:
            mode: Control mode to set (MIT, POS_VEL, VEL, TORQUE_POS)

        Returns:
            Coroutine that yields RegisterResponse when awaited

        Reference: README.md Motor class method pattern lines 334-340

        """
        # Encode control mode and send request
        encode_set_control_mode(self.bus, self.motor_id, mode)

        # Return coroutine from asynchronous decode function
        return decode_register_value(self.bus)


    def control_mit(self, params: MitControlParams) -> Coroutine[Any, Any, MotorState]:
        """Control motor in MIT mode. Returns coroutine to be awaited.

        Args:
            params: MIT control parameters dataclass

        Returns:
            Coroutine that yields MotorState when awaited

        Reference: README.md Motor class method pattern lines 334-340

        """
        # Encode MIT control and send request
        encode_control_mit(self.bus, self.motor_id, self.motor_limits, params)

        # Return coroutine from asynchronous decode function
        return decode_motor_state(self.bus, self.motor_id, self.motor_limits)


    def control_pos_vel(
        self, params: PosVelControlParams
    ) -> Coroutine[Any, Any, MotorState]:
        """Control motor in position/velocity mode. Returns coroutine to be awaited.

        Args:
            params: Position and velocity control parameters dataclass

        Returns:
            Coroutine that yields MotorState when awaited

        Reference: README.md Motor class method pattern lines 334-340

        """
        # Encode position/velocity control and send request
        encode_control_pos_vel(self.bus, self.motor_id, params)

        # Return coroutine from asynchronous decode function
        return decode_motor_state(self.bus, self.motor_id, self.motor_limits)


    def control_vel(self, params: VelControlParams) -> Coroutine[Any, Any, MotorState]:
        """Control motor in velocity mode. Returns coroutine to be awaited.

        Args:
            params: Velocity control parameters dataclass

        Returns:
            Coroutine that yields MotorState when awaited

        Reference: README.md Motor class method pattern lines 334-340

        """
        # Encode velocity control and send request
        encode_control_vel(self.bus, self.motor_id, params)

        # Return coroutine from asynchronous decode function
        return decode_motor_state(self.bus, self.motor_id, self.motor_limits)


    def control_pos_force(
        self, params: PosForceControlParams
    ) -> Coroutine[Any, Any, MotorState]:
        """Control motor in position/force mode. Returns coroutine to be awaited.

        Args:
            params: Position and force control parameters dataclass

        Returns:
            Coroutine that yields MotorState when awaited

        Reference: README.md Motor class method pattern lines 334-340

        """
        # Encode position/force control and send request
        encode_control_pos_force(self.bus, self.motor_id, params)

        # Return coroutine from asynchronous decode function
        return decode_motor_state(self.bus, self.motor_id, self.motor_limits)


    def enable(self) -> Coroutine[Any, Any, AckResponse]:
        """Enable motor. Returns coroutine to be awaited.

        Returns:
            Coroutine that yields AckResponse when awaited

        Reference: README.md Motor class method pattern lines 334-340

        """
        # Encode enable command and send request
        encode_enable_motor(self.bus, self.motor_id)

        # Return coroutine from asynchronous decode function
        return decode_acknowledgment(self.bus)


    def enable_legacy(
        self, control_mode: ControlMode
    ) -> Coroutine[Any, Any, AckResponse]:
        """Enable motor with legacy firmware. Returns coroutine to be awaited.

        Args:
            control_mode: Control mode for legacy enable calculation

        Returns:
            Coroutine that yields AckResponse when awaited

        Reference: README.md Motor class method pattern lines 334-340

        """
        # Encode legacy enable command and send request
        encode_enable_motor_legacy(self.bus, self.motor_id, control_mode)

        # Return coroutine from asynchronous decode function
        return decode_acknowledgment(self.bus)


    def disable(self) -> Coroutine[Any, Any, AckResponse]:
        """Disable motor. Returns coroutine to be awaited.

        Returns:
            Coroutine that yields AckResponse when awaited

        Reference: README.md Motor class method pattern lines 334-340

        """
        # Encode disable command and send request
        encode_disable_motor(self.bus, self.motor_id)

        # Return coroutine from asynchronous decode function
        return decode_acknowledgment(self.bus)


    def set_zero_position(self) -> Coroutine[Any, Any, AckResponse]:
        """Set motor zero position. Returns coroutine to be awaited.

        Returns:
            Coroutine that yields AckResponse when awaited

        Reference: README.md Motor class method pattern lines 334-340

        """
        # Encode set zero position command and send request
        encode_set_zero_position(self.bus, self.motor_id)

        # Return coroutine from asynchronous decode function
        return decode_acknowledgment(self.bus)


    def read_register(
        self, register_address: RegisterAddress
    ) -> Coroutine[Any, Any, RegisterResponse]:
        """Read motor register value. Returns coroutine to be awaited.

        Args:
            register_address: Register address to read

        Returns:
            Coroutine that yields RegisterResponse when awaited

        Reference: README.md Motor class method pattern lines 334-340

        """
        # Encode read register command and send request
        encode_read_register(self.bus, self.motor_id, register_address)

        # Return coroutine from asynchronous decode function
        return decode_register_value(self.bus)


    def write_register_int(
        self, register_address: RegisterAddress, value: int
    ) -> Coroutine[Any, Any, RegisterResponse]:
        """Write motor register value as integer. Returns coroutine to be awaited.

        Args:
            register_address: Register address to write
            value: Integer value to write to register

        Returns:
            Coroutine that yields RegisterResponse when awaited

        Reference: README.md Motor class method pattern lines 334-340

        """
        # Encode write register integer command and send request
        encode_write_register_int(self.bus, self.motor_id, register_address, value)

        # Return coroutine from asynchronous decode function
        return decode_register_value(self.bus)


    def write_register_float(
        self, register_address: RegisterAddress, value: float
    ) -> Coroutine[Any, Any, RegisterResponse]:
        """Write motor register value as float. Returns coroutine to be awaited.

        Args:
            register_address: Register address to write
            value: Float value to write to register

        Returns:
            Coroutine that yields RegisterResponse when awaited

        Reference: README.md Motor class method pattern lines 334-340

        """
        # Encode write register float command and send request
        encode_write_register_float(self.bus, self.motor_id, register_address, value)

        # Return coroutine from asynchronous decode function
        return decode_register_value(self.bus)


    def save_parameters(self) -> Coroutine[Any, Any, AckResponse]:
        """Save motor parameters to flash. Returns coroutine to be awaited.

        Returns:
            Coroutine that yields AckResponse when awaited

        Reference: README.md Motor class method pattern lines 334-340

        """
        # Encode save parameters command and send request
        encode_save_parameters(self.bus, self.motor_id)

        # Return coroutine from asynchronous decode function
        return decode_acknowledgment(self.bus)


    def refresh_status(self) -> Coroutine[Any, Any, MotorState]:
        """Refresh motor status. Returns coroutine to be awaited.

        Returns:
            Coroutine that yields MotorState when awaited

        Reference: README.md Motor class method pattern lines 334-340

        """
        # Encode refresh status command and send request
        encode_refresh_status(self.bus, self.motor_id)

        # Return coroutine from asynchronous decode function
        return decode_motor_state(self.bus, self.motor_id, self.motor_limits)


__all__ = [
    "MOTOR_LIMITS",
    "AckResponse",
    "ControlMode",
    "MitControlParams",
    "Motor",
    "MotorLimits",
    "MotorState",
    "MotorType",
    "PosForceControlParams",
    "PosVelControlParams",
    "RegisterAddress",
    "RegisterResponse",
    "VelControlParams",
]
