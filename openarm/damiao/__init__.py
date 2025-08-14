"""Damiao subpackage for OpenArm.

Provides utilities for constructing CAN messages to control Damiao motors,
including commands for enabling/disabling motors, reading registers, and
decoding motor responses.
"""

import struct
from enum import IntEnum, StrEnum
from typing import NamedTuple

import can

__version__ = "0.1.0"

_STATE_CODE = 0x11
_READ_REGISTER_CODE = 0x33
_WRITE_REGISTER_CODE = 0x55


class MotorType(StrEnum):
    """Enumeration of Damiao motor types."""

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


class _MotorLimit(NamedTuple):
    q_max: float
    dq_max: float
    tau_max: float


_MOTOR_LIMITS = {
    MotorType.DM4310: _MotorLimit(q_max=12.5, dq_max=30, tau_max=10),
    MotorType.DM4310_48V: _MotorLimit(q_max=12.5, dq_max=50, tau_max=10),
    MotorType.DM4340: _MotorLimit(q_max=12.5, dq_max=8, tau_max=28),
    MotorType.DM4340_48V: _MotorLimit(q_max=12.5, dq_max=10, tau_max=28),
    MotorType.DM6006: _MotorLimit(q_max=12.5, dq_max=45, tau_max=20),
    MotorType.DM8006: _MotorLimit(q_max=12.5, dq_max=45, tau_max=40),
    MotorType.DM8009: _MotorLimit(q_max=12.5, dq_max=45, tau_max=54),
    MotorType.DM10010L: _MotorLimit(q_max=12.5, dq_max=25, tau_max=200),
    MotorType.DM10010: _MotorLimit(q_max=12.5, dq_max=20, tau_max=200),
    MotorType.DMH3510: _MotorLimit(q_max=12.5, dq_max=280, tau_max=1),
    MotorType.DMH6215: _MotorLimit(q_max=12.5, dq_max=45, tau_max=10),
    MotorType.DMG6220: _MotorLimit(q_max=12.5, dq_max=45, tau_max=10),
}


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


def set_zero_command(slave_id: int) -> can.Message:
    """Create a CAN message to set the current position of a Damiao motor as zero.

    Args:
        slave_id (int): The slave ID of the target motor.

    Returns:
        can.Message: A CAN message that, when sent, sets the motor's current position
            as its zero reference.

    """
    return can.Message(
        arbitration_id=slave_id,
        data=[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE],
        is_extended_id=False,
    )


def refresh_command(slave_id: int) -> can.Message:
    """Create a CAN message to refresh the state of a Damiao motor.

    Args:
        slave_id (int): Slave ID for the target motor.

    Returns:
        can.Message: A CAN message that, when sent, prompts the motor
            to send its current state.

    """
    return can.Message(
        arbitration_id=0x7FF,
        data=[
            slave_id & 0xFF,
            (slave_id >> 8) & 0xFF,
            0xCC,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
        ],
        is_extended_id=False,
    )


def _map_float_to_uint(val: float, min_val: float, max_val: float, bits: int) -> int:
    if val < min_val:
        val = min_val
    elif val > max_val:
        val = max_val

    norm = (val - min_val) / (max_val - min_val)
    return int(norm * ((1 << bits) - 1))


def control_mit_command(
    motor_type: MotorType,
    slave_id: int,
    kp: float,
    kd: float,
    q: float,
    dq: float,
    tau: float,
) -> can.Message:
    """Create a CAN message to control a Damiao motor in MIT mode.

    Args:
        motor_type (MotorType): The type of the motor.
        slave_id (int): Slave ID for the target motor.
        kp (float): Proportional gain.
        kd (float): Derivative gain.
        q (float): Desired position (radians).
        dq (float): Desired velocity (radians/second).
        tau (float): Desired torque (Nm).

    Returns:
        can.Message: A CAN message with MIT-mode control parameters.

    """
    lim = _MOTOR_LIMITS[motor_type]

    kp_uint = _map_float_to_uint(kp, 0, 500, 12)
    kd_uint = _map_float_to_uint(kd, 0, 500, 12)
    q_uint = _map_float_to_uint(q, -lim.q_max, lim.q_max, 16)
    dq_uint = _map_float_to_uint(dq, -lim.dq_max, lim.dq_max, 12)
    tau_uint = _map_float_to_uint(tau, -lim.tau_max, lim.tau_max, 12)

    return can.Message(
        arbitration_id=slave_id,
        data=[
            (q_uint >> 8) & 0xFF,
            q_uint & 0xFF,
            dq_uint >> 4,
            ((dq_uint & 0xF) << 4) | ((kp_uint >> 8) & 0xF),
            kp_uint & 0xFF,
            kd_uint >> 4,
            ((kd_uint & 0xF) << 4) | ((tau_uint >> 8) & 0xF),
            tau_uint & 0xFF,
        ],
        is_extended_id=False,
    )


def control_pos_vel_command(
    slave_id: int,
    pos: float,
    vel: float,
) -> can.Message:
    """Create a CAN message to control a Damiao motor in position and velocity mode.

    Args:
        slave_id (int): Slave ID for the target motor.
        pos (float): Desired position (radians).
        vel (float): Desired velocity (radians/second).

    Returns:
        can.Message: A CAN message containing position and velocity control parameters.

    """
    return can.Message(
        arbitration_id=0x100 + slave_id,
        data=[*struct.pack("<f", float(pos)), *struct.pack("<f", float(vel))],
        is_extended_id=False,
    )


def control_vel_command(
    slave_id: int,
    vel: float,
) -> can.Message:
    """Create a CAN message to control a Damiao motor in velocity mode.

    Args:
        slave_id (int): Slave ID for the target motor.
        vel (float): Desired velocity (radians/second).

    Returns:
        can.Message: A CAN message containing velocity control parameters.

    """
    return can.Message(
        arbitration_id=0x200 + slave_id,
        data=[*struct.pack("<f", float(vel)), 0x00, 0x00, 0x00, 0x00],
        is_extended_id=False,
    )


def control_pos_force_command(
    slave_id: int,
    pos: float,
    vel: float,
    i_norm: float,
) -> can.Message:
    """Create a CAN message to control a Damiao motor in force-position hybrid mode.

    Args:
        slave_id (int): Slave ID for the target motor.
        pos (float): Desired position (radians).
        vel (float): Desired velocity (radians/second).
        i_norm (float): Desired normalized current in the range 0-1,
            where 1 corresponds to the motor's maximum current.

    Returns:
        can.Message: A CAN message containing the position, velocity, and current
            parameters.

    """
    if i_norm < 0:
        i_norm = 0
    elif i_norm > 1:
        i_norm = 1

    vel_uint = int(vel * 100)
    i_uint = int(i_norm * 10000)

    return can.Message(
        arbitration_id=0x300 + slave_id,
        data=[
            *struct.pack("<f", float(pos)),
            vel_uint & 0xFF,
            vel_uint >> 8,
            i_uint & 0xFF,
            i_uint >> 8,
        ],
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


def write_register_command(
    slave_id: int, address: RegisterAddress, value: float, *, as_float: bool = False
) -> can.Message:
    """Create a CAN message to write a value to a specific register of a Damiao motor.

    Args:
        slave_id (int): Slave ID of the target motor.
        address (RegisterAddress): The register address to write.
        value (float): The value to write to the register.
        as_float (bool, optional): If True, the value is written as a 32-bit float.

    Returns:
        can.Message: A CAN message that, when sent, writes the value to the register.

    """
    return can.Message(
        arbitration_id=0x7FF,
        data=[
            slave_id & 0xFF,
            (slave_id >> 8) & 0xFF,
            _WRITE_REGISTER_CODE,
            address,
            *(
                struct.pack("<f", float(value))
                if as_float
                else struct.pack("<I", int(value))
            ),
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


class MotorState(NamedTuple):
    """Represents a state data from a Damiao motor."""

    q: float
    dq: float
    tau: float
    t_mos: int
    t_rotor: int


def _map_uint_to_float(val: int, min_val: float, max_val: float, bits: int) -> float:
    norm = val / ((1 << bits) - 1)
    return min_val + norm * (max_val - min_val)


class StateResponse(Response):
    """Represents a response containing state data from a Damiao motor."""

    def __init__(self, msg: can.Message) -> None:
        """Initialize a StateResponse object from a CAN message.

        Args:
            msg (can.Message): The CAN message received from the motor.

        """
        super().__init__(msg)
        self.q = (msg.data[1] << 8) | msg.data[2]
        self.dq = (msg.data[3] << 4) | (msg.data[4] >> 4)
        self.tau = ((msg.data[4] & 0xF) << 8) | msg.data[5]
        self.t_mos = msg.data[6]
        self.t_rotor = msg.data[7]

    def as_motor(self, motor_type: MotorType) -> MotorState:
        """Convert the raw state values to scaled motor units.

        Args:
            motor_type (MotorType): The type of the motor.

        Returns:
            MotorState: A scaled motor state.

        """
        lim = _MOTOR_LIMITS[motor_type]
        return MotorState(
            q=_map_uint_to_float(self.q, -lim.q_max, lim.q_max, 16),
            dq=_map_uint_to_float(self.dq, -lim.dq_max, lim.dq_max, 12),
            tau=_map_uint_to_float(self.tau, -lim.tau_max, lim.tau_max, 12),
            t_mos=self.t_mos,
            t_rotor=self.t_rotor,
        )


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
    if msg.data[0] == _STATE_CODE:
        return StateResponse(msg)
    if msg.data[2] == _READ_REGISTER_CODE or msg.data[2] == _WRITE_REGISTER_CODE:
        return RegisterResponse(msg)

    return UnknownResponse(msg)


__all__ = [
    "MotorState",
    "MotorType",
    "RegisterAddress",
    "RegisterResponse",
    "Response",
    "StateResponse",
    "UnknownResponse",
    "control_mit_command",
    "control_pos_force_command",
    "control_pos_vel_command",
    "control_vel_command",
    "decode_response",
    "disable_command",
    "enable_command",
    "read_register_command",
    "refresh_command",
    "set_zero_command",
    "write_register_command",
]
