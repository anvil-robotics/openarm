"""Damiao sub-package for OpenArm.

This module provides helper functions to construct CAN messages
for enabling and disabling Damiao motors, reading registers, and
decoding responses from the motors.
"""

import struct
from enum import IntEnum

import can

__version__ = "0.1.0"

_READ_REGISTER_CODE = 0x33


class RegisterAddress(IntEnum):
    """Enumeration of Damiao motor register addresses."""

    UV_VALUE = 0
    KT_VALUE = 1
    OT_VALUE = 2
    OC_VALUE = 3
    ACC = 4
    DEC = 5
    MAX_SPD = 6
    MST_ID = 7
    ESC_ID = 8
    TIMEOUT = 9
    CTRL_MODE = 10
    DAMP = 11
    INERTIA = 12
    HW_VER = 13
    SW_VER = 14
    SN = 15
    NPP = 16
    RS = 17
    LS = 18
    FLUX = 19
    GR = 20
    PMAX = 21
    VMAX = 22
    TMAX = 23
    I_BW = 24
    KP_ASR = 25
    KI_ASR = 26
    KP_APR = 27
    KI_APR = 28
    OV_VALUE = 29
    GREF = 30
    DETA = 31
    V_BW = 32
    IQ_C1 = 33
    VL_C1 = 34
    CAN_BAR = 35
    SUB_VER = 36
    U_OFF = 50
    V_OFF = 51
    K1 = 52
    K2 = 53
    M_OFF = 54
    DIR = 55
    P_M = 80
    XOUT = 81


def enable_command(slave_id: int) -> can.Message:
    """Create a CAN message to enable a Damiao motor.

    Args:
        slave_id (int): Slave ID for the target motor.

    Returns:
        can.Message: A CAN message that, when sent, enables the motor.

    """
    return can.Message(
        arbitration_id=slave_id,
        data=[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC],
        is_extended_id=False,
    )


def disable_command(slave_id: int) -> can.Message:
    """Create a CAN message to disable a Damiao motor.

    Args:
        slave_id (int): Slave ID for the target motor.

    Returns:
        can.Message: A CAN message that, when sent, disables the motor.

    """
    return can.Message(
        arbitration_id=slave_id,
        data=[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD],
        is_extended_id=False,
    )


def read_register_command(slave_id: int, address: RegisterAddress) -> can.Message:
    """Create a CAN message to read a specific register from a Damiao motor.

    Args:
        slave_id (int): Slave ID for the target motor.
        address (RegisterAddress): The register address to read.

    Returns:
        can.Message: A CAN message that, when sent, requests the register value.

    """
    return can.Message(
        arbitration_id=0x7FF,
        data=[
            slave_id & 0xFF,
            (slave_id >> 8) & 0xFF,
            _READ_REGISTER_CODE,
            address,
            0x00,
            0x00,
            0x00,
            0x00,
        ],
        is_extended_id=False,
    )


class Response:
    """Base class for responses from Damiao motors."""

    def __init__(self, msg: can.Message) -> None:
        """Initialize a Response object from a CAN message.

        Args:
            msg (can.Message): The CAN message received from the motor.

        """
        self.master_id = msg.arbitration_id


class UnknownResponse(Response):
    """Represents an unknown response from a Damiao motor."""

    def __init__(self, msg: can.Message) -> None:
        """Initialize an UnknownResponse object from a CAN message.

        Args:
            msg (can.Message): The CAN message received from the motor.

        """
        super().__init__(msg)
        self.data = msg.data


class RegisterResponse(Response):
    """Represents a response containing register data from a Damiao motor."""

    def __init__(self, msg: can.Message) -> None:
        """Initialize a RegisterResponse object from a CAN message.

        Args:
            msg (can.Message): The CAN message received from the motor.

        """
        super().__init__(msg)
        self.slave_id = msg.data[0] | (msg.data[1] << 8)
        self.register = RegisterAddress(msg.data[3])
        self.data = bytes(msg.data[4:8])

    def as_int(self) -> int:
        """Interpret the register data as a 32-bit unsigned integer.

        Returns:
            int: The register value as an integer.

        """
        return struct.unpack("<I", self.data)[0]

    def as_float(self) -> float:
        """Interpret the register data as a 32-bit float.

        Returns:
            float: The register value as a float.

        """
        return struct.unpack("<f", self.data)[0]


def decode_response(msg: can.Message) -> Response:
    """Decode a CAN message into the appropriate Response subclass.

    Args:
        msg (can.Message): The CAN message to decode.

    Returns:
        Response: Either a RegisterResponse or UnknownResponse depending on the command.

    """
    if msg.data[2] == _READ_REGISTER_CODE:
        return RegisterResponse(msg)
    return UnknownResponse(msg)


__all__ = [
    "RegisterAddress",
    "RegisterResponse",
    "Response",
    "UnknownResponse",
    "decode_response",
    "disable_command",
    "enable_command",
    "read_register_command",
]
