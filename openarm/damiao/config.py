from __future__ import annotations

"""Motor configuration and shared constants for Damiao motors."""

from dataclasses import dataclass
from math import pi

from .motor import MotorType


@dataclass
class MotorConfig:
    """Configuration for a single motor."""

    name: str
    slave_id: int  # Target slave ID
    master_id: int  # Target master ID
    type: MotorType
    inverted: bool  # Whether to negate position in mirror mode
    min_angle_left: float  # Minimum angle in degrees for left arm
    max_angle_left: float  # Maximum angle in degrees for left arm
    min_angle_right: float  # Minimum angle in degrees for right arm
    max_angle_right: float  # Maximum angle in degrees for right arm


# Motor configurations
MOTOR_CONFIGS: list[MotorConfig] = [
    MotorConfig(
        "J1",
        slave_id=0x01,
        master_id=0x11,
        type=MotorType.DM8009,
        inverted=True,
        min_angle_left=-200.0,
        max_angle_left=80.0,
        min_angle_right=-80.0,
        max_angle_right=200.0,
    ),
    MotorConfig(
        "J2",
        slave_id=0x02,
        master_id=0x12,
        type=MotorType.DM8009,
        inverted=True,
        min_angle_left=-190.0,
        max_angle_left=10.0,
        min_angle_right=-10.0,
        max_angle_right=190.0,
    ),
    MotorConfig(
        "J3",
        slave_id=0x03,
        master_id=0x13,
        type=MotorType.DM4340,
        inverted=True,
        min_angle_left=-90.0,
        max_angle_left=90.0,
        min_angle_right=-90.0,
        max_angle_right=90.0,
    ),
    MotorConfig(
        "J4",
        slave_id=0x04,
        master_id=0x14,
        type=MotorType.DM4340,
        inverted=False,
        min_angle_left=0.0,
        max_angle_left=140.0,
        min_angle_right=0.0,
        max_angle_right=140.0,
    ),
    MotorConfig(
        "J5",
        slave_id=0x05,
        master_id=0x15,
        type=MotorType.DM4310,
        inverted=True,
        min_angle_left=-90.0,
        max_angle_left=90.0,
        min_angle_right=-90.0,
        max_angle_right=90.0,
    ),
    MotorConfig(
        "J6",
        slave_id=0x06,
        master_id=0x16,
        type=MotorType.DM4310,
        inverted=True,
        min_angle_left=-45.0,
        max_angle_left=45.0,
        min_angle_right=-45.0,
        max_angle_right=45.0,
    ),
    MotorConfig(
        "J7",
        slave_id=0x07,
        master_id=0x17,
        type=MotorType.DM4310,
        inverted=True,
        min_angle_left=-90.0,
        max_angle_left=90.0,
        min_angle_right=-90.0,
        max_angle_right=90.0,
    ),
    MotorConfig(
        "J8",
        slave_id=0x08,
        master_id=0x18,
        type=MotorType.DM4310,
        inverted=False,
        min_angle_left=-45.0,
        max_angle_left=0.0,
        min_angle_right=-45.0,
        max_angle_right=0.0,
    ),
]


# Per-joint (kp, kd) gains for J1-J8, used in homing, waypoints, and teleop.
JOINT_GAINS = [
    (300.0, 30.0),   # Joint 0
    (150.0, 50.0),   # Joint 1
    (150.0, 100.0),  # Joint 2
    (200.0, 50.0),   # Joint 3
    (40.0, 3.0),     # Joint 4
    (40.0, 3.0),     # Joint 5
    (40.0, 3.0),     # Joint 6
    (4.0, 1.0),      # Joint 7
]

# Startup homing waypoints executed in series before teleop begins.
# Each entry is (joint_angles_rad [J1-J8], duration_seconds).
HOME_WAYPOINTS: list[tuple[list[float], float]] = [
    ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0.5),
    ([0.0, 0.0, 0.0, pi / 2, 0.0, 0.0, 0.0, 0.0], 0.5),
]

FRAME_GAP = 0.0003  # seconds between CAN send/recv pairs

# Hardware button remap: hardware [1,2,3,4] -> logical [1,4,3,2]
BTN_REMAP = [0, 3, 2, 1]
