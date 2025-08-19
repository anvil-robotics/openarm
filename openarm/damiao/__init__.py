"""Damiao motor control package for OpenArm.

High-level interface for controlling Damiao motors through CAN bus communication.
This module provides the main Motor class that orchestrates encode/decode operations
from the low-level encoding.py implementation.

Reference: README.md High-Level Motor Class section for architecture details.
"""

from collections.abc import Coroutine
from enum import Enum
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
    decode_register_float,
    decode_register_int,
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
    encode_set_zero_position,
    encode_write_register_float,
    encode_write_register_int,
)

__version__ = "0.1.0"


class MotorType(str, Enum):
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
    ) -> Coroutine[Any, Any, int]:
        """Set motor control mode. Returns coroutine to be awaited.

        Args:
            mode: Control mode to set (MIT, POS_VEL, VEL, TORQUE_POS)

        Returns:
            Coroutine that yields int when awaited

        Reference: README.md Motor class method pattern lines 334-340

        """
        # Write control mode to register 10 as integer value
        # Reference: DM_CAN.py switchControlMode using __write_motor_param with RID=10
        encode_write_register_int(
            self.bus, self.motor_id, RegisterAddress.CTRL_MODE, int(mode)
        )

        # Return coroutine from asynchronous decode function
        return decode_register_int(self.bus)


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

    # Voltage Protection Parameters
    def get_under_voltage(self) -> Coroutine[Any, Any, float]:
        """Get motor under-voltage protection value. Returns coroutine to be awaited.

        Returns:
            Coroutine that yields float when awaited

        Range: (10.0, 3.4E38] volts

        """
        encode_read_register(self.bus, self.motor_id, RegisterAddress.UV_VALUE)
        return decode_register_float(self.bus)

    def set_under_voltage(
        self, value: float
    ) -> Coroutine[Any, Any, float]:
        """Set motor under-voltage protection value. Returns coroutine to be awaited.

        Args:
            value: Under-voltage threshold in volts (10.0, 3.4E38]

        Returns:
            Coroutine that yields float when awaited

        """
        encode_write_register_float(
            self.bus, self.motor_id, RegisterAddress.UV_VALUE, value
        )
        return decode_register_float(self.bus)

    def get_over_voltage(self) -> Coroutine[Any, Any, float]:
        """Get motor over-voltage protection value. Returns coroutine to be awaited.

        Returns:
            Coroutine that yields float when awaited

        """
        encode_read_register(self.bus, self.motor_id, RegisterAddress.OV_VALUE)
        return decode_register_float(self.bus)

    def set_over_voltage(
        self, value: float
    ) -> Coroutine[Any, Any, float]:
        """Set motor over-voltage protection value. Returns coroutine to be awaited.

        Args:
            value: Over-voltage threshold in volts

        Returns:
            Coroutine that yields float when awaited

        """
        encode_write_register_float(
            self.bus, self.motor_id, RegisterAddress.OV_VALUE, value
        )
        return decode_register_float(self.bus)

    # Motor Characteristics
    def get_torque_coefficient(self) -> Coroutine[Any, Any, float]:
        """Get motor torque coefficient. Returns coroutine to be awaited.

        Returns:
            Coroutine that yields float when awaited

        Range: [0.0, 3.4E38]

        """
        encode_read_register(self.bus, self.motor_id, RegisterAddress.KT_VALUE)
        return decode_register_float(self.bus)

    def set_torque_coefficient(
        self, value: float
    ) -> Coroutine[Any, Any, float]:
        """Set motor torque coefficient. Returns coroutine to be awaited.

        Args:
            value: Torque coefficient [0.0, 3.4E38]

        Returns:
            Coroutine that yields float when awaited

        """
        encode_write_register_float(
            self.bus, self.motor_id, RegisterAddress.KT_VALUE, value
        )
        return decode_register_float(self.bus)

    def get_gear_efficiency(self) -> Coroutine[Any, Any, float]:
        """Get gear torque efficiency. Returns coroutine to be awaited.

        Returns:
            Coroutine that yields float when awaited

        Range: (0.0, 1.0]

        """
        encode_read_register(self.bus, self.motor_id, RegisterAddress.GREF)
        return decode_register_float(self.bus)

    def set_gear_efficiency(
        self, value: float
    ) -> Coroutine[Any, Any, float]:
        """Set gear torque efficiency. Returns coroutine to be awaited.

        Args:
            value: Gear efficiency factor (0.0, 1.0]

        Returns:
            Coroutine that yields float when awaited

        """
        encode_write_register_float(
            self.bus, self.motor_id, RegisterAddress.GREF, value
        )
        return decode_register_float(self.bus)

    # Protection Limits
    def get_over_temperature(self) -> Coroutine[Any, Any, float]:
        """Get motor over-temperature protection value. Returns coroutine to be awaited.

        Returns:
            Coroutine that yields float when awaited

        Range: [80.0, 200) degrees Celsius

        """
        encode_read_register(self.bus, self.motor_id, RegisterAddress.OT_VALUE)
        return decode_register_float(self.bus)

    def set_over_temperature(
        self, value: float
    ) -> Coroutine[Any, Any, float]:
        """Set motor over-temperature protection value. Returns coroutine to be awaited.

        Args:
            value: Over-temperature threshold in degrees Celsius [80.0, 200)

        Returns:
            Coroutine that yields float when awaited

        """
        encode_write_register_float(
            self.bus, self.motor_id, RegisterAddress.OT_VALUE, value
        )
        return decode_register_float(self.bus)

    def get_over_current(self) -> Coroutine[Any, Any, float]:
        """Get motor over-current protection value. Returns coroutine to be awaited.

        Returns:
            Coroutine that yields float when awaited

        Range: (0.0, 1.0) normalized current

        """
        encode_read_register(self.bus, self.motor_id, RegisterAddress.OC_VALUE)
        return decode_register_float(self.bus)

    def set_over_current(
        self, value: float
    ) -> Coroutine[Any, Any, float]:
        """Set motor over-current protection value. Returns coroutine to be awaited.

        Args:
            value: Over-current threshold as normalized current (0.0, 1.0)

        Returns:
            Coroutine that yields float when awaited

        """
        encode_write_register_float(
            self.bus, self.motor_id, RegisterAddress.OC_VALUE, value
        )
        return decode_register_float(self.bus)

    # Mapping Limits
    def get_position_limit(self) -> Coroutine[Any, Any, float]:
        """Get motor position mapping maximum value. Returns coroutine to be awaited.

        Returns:
            Coroutine that yields float when awaited

        Range: (0.0, 3.4E38] radians

        """
        encode_read_register(self.bus, self.motor_id, RegisterAddress.PMAX)
        return decode_register_float(self.bus)

    def set_position_limit(
        self, value: float
    ) -> Coroutine[Any, Any, float]:
        """Set motor position mapping maximum value. Returns coroutine to be awaited.

        Args:
            value: Position limit in radians (0.0, 3.4E38]

        Returns:
            Coroutine that yields float when awaited

        """
        encode_write_register_float(
            self.bus, self.motor_id, RegisterAddress.PMAX, value
        )
        return decode_register_float(self.bus)

    def get_velocity_limit(self) -> Coroutine[Any, Any, float]:
        """Get motor velocity mapping maximum value. Returns coroutine to be awaited.

        Returns:
            Coroutine that yields float when awaited

        Range: (0.0, 3.4E38] rad/s

        """
        encode_read_register(self.bus, self.motor_id, RegisterAddress.VMAX)
        return decode_register_float(self.bus)

    def set_velocity_limit(
        self, value: float
    ) -> Coroutine[Any, Any, float]:
        """Set motor velocity mapping maximum value. Returns coroutine to be awaited.

        Args:
            value: Velocity limit in rad/s (0.0, 3.4E38]

        Returns:
            Coroutine that yields float when awaited

        """
        encode_write_register_float(
            self.bus, self.motor_id, RegisterAddress.VMAX, value
        )
        return decode_register_float(self.bus)

    def get_torque_limit(self) -> Coroutine[Any, Any, float]:
        """Get motor torque mapping maximum value. Returns coroutine to be awaited.

        Returns:
            Coroutine that yields float when awaited

        Range: (0.0, 3.4E38] Nm

        """
        encode_read_register(self.bus, self.motor_id, RegisterAddress.TMAX)
        return decode_register_float(self.bus)

    def set_torque_limit(
        self, value: float
    ) -> Coroutine[Any, Any, float]:
        """Set motor torque mapping maximum value. Returns coroutine to be awaited.

        Args:
            value: Torque limit in Nm (0.0, 3.4E38]

        Returns:
            Coroutine that yields float when awaited

        """
        encode_write_register_float(
            self.bus, self.motor_id, RegisterAddress.TMAX, value
        )
        return decode_register_float(self.bus)

    # Control Loop Parameters
    def get_velocity_kp(self) -> Coroutine[Any, Any, float]:
        """Get velocity loop proportional gain. Returns coroutine to be awaited.

        Returns:
            Coroutine that yields float when awaited

        Range: [0.0, 3.4E38]

        """
        encode_read_register(self.bus, self.motor_id, RegisterAddress.KP_ASR)
        return decode_register_float(self.bus)

    def set_velocity_kp(
        self, value: float
    ) -> Coroutine[Any, Any, float]:
        """Set velocity loop proportional gain. Returns coroutine to be awaited.

        Args:
            value: Velocity loop Kp [0.0, 3.4E38]

        Returns:
            Coroutine that yields float when awaited

        """
        encode_write_register_float(
            self.bus, self.motor_id, RegisterAddress.KP_ASR, value
        )
        return decode_register_float(self.bus)

    def get_velocity_ki(self) -> Coroutine[Any, Any, float]:
        """Get velocity loop integral gain. Returns coroutine to be awaited.

        Returns:
            Coroutine that yields float when awaited

        Range: [0.0, 3.4E38]

        """
        encode_read_register(self.bus, self.motor_id, RegisterAddress.KI_ASR)
        return decode_register_float(self.bus)

    def set_velocity_ki(
        self, value: float
    ) -> Coroutine[Any, Any, float]:
        """Set velocity loop integral gain. Returns coroutine to be awaited.

        Args:
            value: Velocity loop Ki [0.0, 3.4E38]

        Returns:
            Coroutine that yields float when awaited

        """
        encode_write_register_float(
            self.bus, self.motor_id, RegisterAddress.KI_ASR, value
        )
        return decode_register_float(self.bus)

    def get_position_kp(self) -> Coroutine[Any, Any, float]:
        """Get position loop proportional gain. Returns coroutine to be awaited.

        Returns:
            Coroutine that yields float when awaited

        Range: [0.0, 3.4E38]

        """
        encode_read_register(self.bus, self.motor_id, RegisterAddress.KP_APR)
        return decode_register_float(self.bus)

    def set_position_kp(
        self, value: float
    ) -> Coroutine[Any, Any, float]:
        """Set position loop proportional gain. Returns coroutine to be awaited.

        Args:
            value: Position loop Kp [0.0, 3.4E38]

        Returns:
            Coroutine that yields float when awaited

        """
        encode_write_register_float(
            self.bus, self.motor_id, RegisterAddress.KP_APR, value
        )
        return decode_register_float(self.bus)

    def get_position_ki(self) -> Coroutine[Any, Any, float]:
        """Get position loop integral gain. Returns coroutine to be awaited.

        Returns:
            Coroutine that yields float when awaited

        Range: [0.0, 3.4E38]

        """
        encode_read_register(self.bus, self.motor_id, RegisterAddress.KI_APR)
        return decode_register_float(self.bus)

    def set_position_ki(
        self, value: float
    ) -> Coroutine[Any, Any, float]:
        """Set position loop integral gain. Returns coroutine to be awaited.

        Args:
            value: Position loop Ki [0.0, 3.4E38]

        Returns:
            Coroutine that yields float when awaited

        """
        encode_write_register_float(
            self.bus, self.motor_id, RegisterAddress.KI_APR, value
        )
        return decode_register_float(self.bus)

    # Read-Only Motor Information
    def get_hardware_version(self) -> Coroutine[Any, Any, int]:
        """Get motor hardware version. Returns coroutine to be awaited.

        Returns:
            Coroutine that yields int when awaited

        """
        encode_read_register(self.bus, self.motor_id, RegisterAddress.HW_VER)
        return decode_register_int(self.bus)

    def get_software_version(self) -> Coroutine[Any, Any, int]:
        """Get motor software version. Returns coroutine to be awaited.

        Returns:
            Coroutine that yields int when awaited

        """
        encode_read_register(self.bus, self.motor_id, RegisterAddress.SW_VER)
        return decode_register_int(self.bus)

    def get_serial_number(self) -> Coroutine[Any, Any, int]:
        """Get motor serial number. Returns coroutine to be awaited.

        Returns:
            Coroutine that yields int when awaited

        """
        encode_read_register(self.bus, self.motor_id, RegisterAddress.SN)
        return decode_register_int(self.bus)

    def get_gear_ratio(self) -> Coroutine[Any, Any, float]:
        """Get motor gear reduction ratio. Returns coroutine to be awaited.

        Returns:
            Coroutine that yields float when awaited

        """
        encode_read_register(self.bus, self.motor_id, RegisterAddress.GR)
        return decode_register_float(self.bus)

    # Motion Parameters
    def get_acceleration(self) -> Coroutine[Any, Any, float]:
        """Get motor acceleration parameter. Returns coroutine to be awaited.

        Returns:
            Coroutine that yields float when awaited

        Range: (0.0, 3.4E38)

        """
        encode_read_register(self.bus, self.motor_id, RegisterAddress.ACC)
        return decode_register_float(self.bus)

    def set_acceleration(
        self, value: float
    ) -> Coroutine[Any, Any, float]:
        """Set motor acceleration parameter. Returns coroutine to be awaited.

        Args:
            value: Acceleration parameter (0.0, 3.4E38)

        Returns:
            Coroutine that yields float when awaited

        """
        encode_write_register_float(
            self.bus, self.motor_id, RegisterAddress.ACC, value
        )
        return decode_register_float(self.bus)

    def get_deceleration(self) -> Coroutine[Any, Any, float]:
        """Get motor deceleration parameter. Returns coroutine to be awaited.

        Returns:
            Coroutine that yields float when awaited

        Range: [-3.4E38, 0.0)

        """
        encode_read_register(self.bus, self.motor_id, RegisterAddress.DEC)
        return decode_register_float(self.bus)

    def set_deceleration(
        self, value: float
    ) -> Coroutine[Any, Any, float]:
        """Set motor deceleration parameter. Returns coroutine to be awaited.

        Args:
            value: Deceleration parameter [-3.4E38, 0.0)

        Returns:
            Coroutine that yields float when awaited

        """
        encode_write_register_float(
            self.bus, self.motor_id, RegisterAddress.DEC, value
        )
        return decode_register_float(self.bus)

    def get_max_speed(self) -> Coroutine[Any, Any, float]:
        """Get motor maximum speed parameter. Returns coroutine to be awaited.

        Returns:
            Coroutine that yields float when awaited

        Range: (0.0, 3.4E38] rad/s

        """
        encode_read_register(self.bus, self.motor_id, RegisterAddress.MAX_SPD)
        return decode_register_float(self.bus)

    def set_max_speed(
        self, value: float
    ) -> Coroutine[Any, Any, float]:
        """Set motor maximum speed parameter. Returns coroutine to be awaited.

        Args:
            value: Maximum speed in rad/s (0.0, 3.4E38]

        Returns:
            Coroutine that yields float when awaited

        """
        encode_write_register_float(
            self.bus, self.motor_id, RegisterAddress.MAX_SPD, value
        )
        return decode_register_float(self.bus)

    # Communication Parameters
    def get_master_id(self) -> Coroutine[Any, Any, int]:
        """Get motor feedback ID (Master ID). Returns coroutine to be awaited.

        Returns:
            Coroutine that yields int when awaited

        Range: [0, 0x7FF]

        """
        encode_read_register(self.bus, self.motor_id, RegisterAddress.MST_ID)
        return decode_register_int(self.bus)

    def set_master_id(self, value: int) -> Coroutine[Any, Any, int]:
        """Set motor feedback ID (Master ID). Returns coroutine to be awaited.

        Args:
            value: Master ID [0, 0x7FF]

        Returns:
            Coroutine that yields int when awaited

        """
        encode_write_register_int(
            self.bus, self.motor_id, RegisterAddress.MST_ID, value
        )
        return decode_register_int(self.bus)

    def get_slave_id(self) -> Coroutine[Any, Any, int]:
        """Get motor receive ID (Slave ID). Returns coroutine to be awaited.

        Returns:
            Coroutine that yields int when awaited

        Range: [0, 0x7FF]

        """
        encode_read_register(self.bus, self.motor_id, RegisterAddress.ESC_ID)
        return decode_register_int(self.bus)

    def set_slave_id(self, value: int) -> Coroutine[Any, Any, int]:
        """Set motor receive ID (Slave ID). Returns coroutine to be awaited.

        Args:
            value: Slave ID [0, 0x7FF]

        Returns:
            Coroutine that yields int when awaited

        """
        encode_write_register_int(
            self.bus, self.motor_id, RegisterAddress.ESC_ID, value
        )
        return decode_register_int(self.bus)

    def get_timeout(self) -> Coroutine[Any, Any, int]:
        """Get motor timeout alarm time. Returns coroutine to be awaited.

        Returns:
            Coroutine that yields int when awaited

        Range: [0, 2^32-1]

        """
        encode_read_register(self.bus, self.motor_id, RegisterAddress.TIMEOUT)
        return decode_register_int(self.bus)

    def set_timeout(self, value: int) -> Coroutine[Any, Any, int]:
        """Set motor timeout alarm time. Returns coroutine to be awaited.

        Args:
            value: Timeout value [0, 2^32-1]

        Returns:
            Coroutine that yields int when awaited

        """
        encode_write_register_int(
            self.bus, self.motor_id, RegisterAddress.TIMEOUT, value
        )
        return decode_register_int(self.bus)

    def get_can_baudrate(self) -> Coroutine[Any, Any, int]:
        """Get CAN baud rate code. Returns coroutine to be awaited.

        Returns:
            Coroutine that yields int when awaited

        Range: [0, 4]

        """
        encode_read_register(self.bus, self.motor_id, RegisterAddress.CAN_BR)
        return decode_register_int(self.bus)

    def set_can_baudrate(
        self, value: int
    ) -> Coroutine[Any, Any, int]:
        """Set CAN baud rate code. Returns coroutine to be awaited.

        Args:
            value: CAN baud rate code [0, 4]

        Returns:
            Coroutine that yields int when awaited

        """
        encode_write_register_int(
            self.bus, self.motor_id, RegisterAddress.CAN_BR, value
        )
        return decode_register_int(self.bus)

    # Read-Only Calibration and Position Methods
    def get_phase_u_offset(self) -> Coroutine[Any, Any, float]:
        """Get U phase offset calibration value. Returns coroutine to be awaited.

        Returns:
            Coroutine that yields float when awaited

        Read-Only Parameter

        """
        encode_read_register(self.bus, self.motor_id, RegisterAddress.U_OFF)
        return decode_register_float(self.bus)

    def get_phase_v_offset(self) -> Coroutine[Any, Any, float]:
        """Get V phase offset calibration value. Returns coroutine to be awaited.

        Returns:
            Coroutine that yields float when awaited

        Read-Only Parameter

        """
        encode_read_register(self.bus, self.motor_id, RegisterAddress.V_OFF)
        return decode_register_float(self.bus)

    def get_compensation_factor_1(self) -> Coroutine[Any, Any, float]:
        """Get compensation factor 1 calibration value. Returns coroutine to be awaited.

        Returns:
            Coroutine that yields float when awaited

        Read-Only Parameter

        """
        encode_read_register(self.bus, self.motor_id, RegisterAddress.K1)
        return decode_register_float(self.bus)

    def get_compensation_factor_2(self) -> Coroutine[Any, Any, float]:
        """Get compensation factor 2 calibration value. Returns coroutine to be awaited.

        Returns:
            Coroutine that yields float when awaited

        Read-Only Parameter

        """
        encode_read_register(self.bus, self.motor_id, RegisterAddress.K2)
        return decode_register_float(self.bus)

    def get_angle_offset(self) -> Coroutine[Any, Any, float]:
        """Get angle offset calibration value. Returns coroutine to be awaited.

        Returns:
            Coroutine that yields float when awaited

        Read-Only Parameter

        """
        encode_read_register(self.bus, self.motor_id, RegisterAddress.M_OFF)
        return decode_register_float(self.bus)

    def get_direction(self) -> Coroutine[Any, Any, float]:
        """Get motor direction calibration value. Returns coroutine to be awaited.

        Returns:
            Coroutine that yields float when awaited

        Read-Only Parameter

        """
        encode_read_register(self.bus, self.motor_id, RegisterAddress.DIR)
        return decode_register_float(self.bus)

    def get_motor_position(self) -> Coroutine[Any, Any, float]:
        """Get motor position. Returns coroutine to be awaited.

        Returns:
            Coroutine that yields float when awaited

        Read-Only Parameter

        """
        encode_read_register(self.bus, self.motor_id, RegisterAddress.P_M)
        return decode_register_float(self.bus)

    def get_output_shaft_position(self) -> Coroutine[Any, Any, float]:
        """Get output shaft position. Returns coroutine to be awaited.

        Returns:
            Coroutine that yields float when awaited

        Read-Only Parameter

        """
        encode_read_register(self.bus, self.motor_id, RegisterAddress.XOUT)
        return decode_register_float(self.bus)

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
