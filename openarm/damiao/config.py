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
    inverted: bool = False  # Whether to negate position in mirror mode


# Motor configurations
MOTOR_CONFIGS: list[MotorConfig] = [
    MotorConfig(
        "J1", slave_id=0x01, master_id=0x11, type=MotorType.DM8009, inverted=True
    ),
    MotorConfig(
        "J2", slave_id=0x02, master_id=0x12, type=MotorType.DM8009, inverted=True
    ),
    MotorConfig(
        "J3", slave_id=0x03, master_id=0x13, type=MotorType.DM4340, inverted=True
    ),
    MotorConfig(
        "J4", slave_id=0x04, master_id=0x14, type=MotorType.DM4340, inverted=False
    ),
    MotorConfig(
        "J5", slave_id=0x05, master_id=0x15, type=MotorType.DM4310, inverted=True
    ),
    MotorConfig(
        "J6", slave_id=0x06, master_id=0x16, type=MotorType.DM4310, inverted=True
    ),
    MotorConfig(
        "J7", slave_id=0x07, master_id=0x17, type=MotorType.DM4310, inverted=True
    ),
    MotorConfig(
        "J8", slave_id=0x08, master_id=0x18, type=MotorType.DM4310, inverted=False
    ),
]
