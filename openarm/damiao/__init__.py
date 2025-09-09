"""Damiao motor control package for OpenArm.

High-level interface for controlling Damiao motors through CAN bus communication.
"""

from .arm import Arm
from .detect import detect_motors
from .encoding import (
    ControlMode,
    MitControlParams,
    MotorLimits,
    MotorState,
    PosForceControlParams,
    PosVelControlParams,
    RegisterAddress,
    SaveResponse,
    VelControlParams,
)
from .motor import MOTOR_LIMITS, Motor, MotorType

__version__ = "0.1.0"

__all__ = [
    "Arm",
    "MOTOR_LIMITS",
    "ControlMode",
    "MitControlParams",
    "Motor",
    "MotorLimits",
    "MotorState",
    "MotorType",
    "PosForceControlParams",
    "PosVelControlParams",
    "RegisterAddress",
    "SaveResponse",
    "VelControlParams",
    "detect_motors",
]
