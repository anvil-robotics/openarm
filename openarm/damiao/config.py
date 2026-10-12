"""Motor configuration for Damiao motors."""

from dataclasses import dataclass

from .motor import MotorType


@dataclass
class MotorConfig:
    """Configuration for a single motor."""

    name: str
    slave_id: int  # Target slave ID
    master_id: int  # Target master ID
    type: MotorType
    min_angle_left: float  # Minimum angle in degrees for left arm
    max_angle_left: float  # Maximum angle in degrees for left arm
    min_angle_right: float  # Minimum angle in degrees for right arm
    max_angle_right: float  # Maximum angle in degrees for right arm
    inverted: bool = False  # Whether to negate position in mirror mode


# Motor configurations
MOTOR_CONFIGS: list[MotorConfig] = [
    MotorConfig(
        "J1",
        slave_id=0x01,
        master_id=0x11,
        type=MotorType.DM8009,
        inverted=True,
        min_angle_left=-135.0,
        max_angle_left=135.0,
        min_angle_right=-135.0,
        max_angle_right=135.0,
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

OPENARM2_MOTOR_CONFIGS: list[MotorConfig] = [
    MotorConfig(
        "J1",
        slave_id=0x01,
        master_id=0x11,
        type=MotorType.DM8009,
        inverted=True,
        min_angle_left=-135.0,
        max_angle_left=135.0,
        min_angle_right=-135.0,
        max_angle_right=135.0,
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
        max_angle_left=70.0,
        min_angle_right=-70.0,
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


# Openyam motor configurations (J1-J6 + J8, symmetric — left == right).
OPENYAM_MOTOR_CONFIGS: list[MotorConfig] = [
    MotorConfig(
        "J1",
        slave_id=0x01,
        master_id=0x11,
        type=MotorType.DM4340,
        min_angle_left=-225.0,
        max_angle_left=90.0,
        min_angle_right=-225.0,
        max_angle_right=90.0,
    ),
    MotorConfig(
        "J2",
        slave_id=0x02,
        master_id=0x12,
        type=MotorType.DM4340,
        min_angle_left=0.0,
        max_angle_left=210.0,
        min_angle_right=0.0,
        max_angle_right=210.0,
    ),
    MotorConfig(
        "J3",
        slave_id=0x03,
        master_id=0x13,
        type=MotorType.DM4340,
        min_angle_left=0.0,
        max_angle_left=230.0,
        min_angle_right=0.0,
        max_angle_right=230.0,
    ),
    MotorConfig(
        "J4",
        slave_id=0x04,
        master_id=0x14,
        type=MotorType.DM4310,
        min_angle_left=-95.0,
        max_angle_left=95.0,
        min_angle_right=-95.0,
        max_angle_right=95.0,
    ),
    MotorConfig(
        "J5",
        slave_id=0x05,
        master_id=0x15,
        type=MotorType.DM4310,
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
        min_angle_left=-135.0,
        max_angle_left=105.0,
        min_angle_right=-135.0,
        max_angle_right=105.0,
    ),
    MotorConfig(
        "J8",
        slave_id=0x08,
        master_id=0x18,
        type=MotorType.DM4310,
        min_angle_left=-45.0,
        max_angle_left=0.0,
        min_angle_right=-45.0,
        max_angle_right=0.0,
    ),
]


# Openyam 2 motor configurations (J1-J6 + J8, symmetric — left == right).
OPENYAM2_MOTOR_CONFIGS: list[MotorConfig] = [
    MotorConfig(
        "J1",
        slave_id=0x01,
        master_id=0x11,
        type=MotorType.DM4340,
        min_angle_left=-150.0,
        max_angle_left=180.0,
        min_angle_right=-150.0,
        max_angle_right=180.0,
    ),
    MotorConfig(
        "J2",
        slave_id=0x02,
        master_id=0x12,
        type=MotorType.DM4340,
        min_angle_left=0.0,
        max_angle_left=210.0,
        min_angle_right=0.0,
        max_angle_right=210.0,
    ),
    MotorConfig(
        "J3",
        slave_id=0x03,
        master_id=0x13,
        type=MotorType.DM4340,
        min_angle_left=0.0,
        max_angle_left=230.0,
        min_angle_right=0.0,
        max_angle_right=230.0,
    ),
    MotorConfig(
        "J4",
        slave_id=0x04,
        master_id=0x14,
        type=MotorType.DM4310,
        min_angle_left=-95.0,
        max_angle_left=95.0,
        min_angle_right=-95.0,
        max_angle_right=95.0,
    ),
    MotorConfig(
        "J5",
        slave_id=0x05,
        master_id=0x15,
        type=MotorType.DM4310,
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
        min_angle_left=-135.0,
        max_angle_left=105.0,
        min_angle_right=-135.0,
        max_angle_right=105.0,
    ),
    MotorConfig(
        "J8",
        slave_id=0x08,
        master_id=0x18,
        type=MotorType.DM4310,
        min_angle_left=-45.0,
        max_angle_left=0.0,
        min_angle_right=-45.0,
        max_angle_right=0.0,
    ),
]


# Map robot name to its motor list.
ROBOT_MOTOR_CONFIGS: dict[str, list[MotorConfig]] = {
    "openarm": MOTOR_CONFIGS,
    "openarm2": OPENARM2_MOTOR_CONFIGS,
    "openyam": OPENYAM_MOTOR_CONFIGS,
    "openyam2": OPENYAM2_MOTOR_CONFIGS,
}
