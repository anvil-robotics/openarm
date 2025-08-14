"""Damiao sub-package for OpenArm.

This module provides helper functions to construct CAN messages
for enabling and disabling Damiao motors.
"""

import can

__version__ = "0.1.0"


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


__all__ = ["disable_command", "enable_command"]
