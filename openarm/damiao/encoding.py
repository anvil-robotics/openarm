"""Damiao motor encoding/decoding functions.

Provides utilities for constructing CAN messages to control Damiao motors,
including commands for setting control mode and MIT control.
"""

import struct
from dataclasses import dataclass
from enum import IntEnum

import can

from openarm.bus import Bus


class ControlMode(IntEnum):
    """Enumeration of Damiao motor control modes."""

    MIT = 1
    POS_VEL = 2
    VEL = 3
    TORQUE_POS = 4


class RegisterAddress(IntEnum):
    """Enumeration of Damiao motor register addresses.

    Reference: DM_CAN.py DM_variable enum
    """

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
    CAN_BR = 35
    SUB_VER = 36
    U_OFF = 50
    V_OFF = 51
    K1 = 52
    K2 = 53
    M_OFF = 54
    DIR = 55
    P_M = 80
    XOUT = 81


_WRITE_REGISTER_CODE = 0x55
_READ_REGISTER_CODE = 0x33
_STATE_CODE = 0x11


def _float_to_uint(value: float, min_val: float, max_val: float, bits: int) -> int:
    """Convert float value to unsigned integer with clamping and scaling.

    Args:
        value: Float value to convert
        min_val: Minimum value in float range
        max_val: Maximum value in float range
        bits: Number of bits for the unsigned integer

    Returns:
        Scaled unsigned integer value

    Reference: DM_CAN.py float_to_uint function lines 494-498

    """
    # Clamp value to valid range
    value = max(min_val, min(value, max_val))

    # Scale to unsigned integer range
    scale = (1 << bits) - 1
    normalized = (value - min_val) / (max_val - min_val)
    return int(normalized * scale)


def _uint_to_float(value: int, min_val: float, max_val: float, bits: int) -> float:
    """Convert unsigned integer to float value with scaling.

    Args:
        value: Unsigned integer value to convert
        min_val: Minimum value in float range
        max_val: Maximum value in float range
        bits: Number of bits for the unsigned integer

    Returns:
        Scaled float value

    Reference: DM_CAN.py uint_to_float function lines 494-498

    """
    # Scale from unsigned integer range to normalized [0, 1]
    scale = (1 << bits) - 1
    normalized = value / scale

    # Scale to actual float range
    return normalized * (max_val - min_val) + min_val


@dataclass
class MotorLimits:
    """Motor physical limits for parameter scaling.

    Reference: DM_CAN.py Limit_Param array structure
    """

    q_max: float  # Maximum position in radians
    dq_max: float  # Maximum velocity in radians/second
    tau_max: float  # Maximum torque in Nm


@dataclass
class MitControlParams:
    """MIT control parameters for Damiao motor control.

    Reference: DM_CAN.py controlMIT function parameters lines 90-99
    """

    kp: float  # Proportional gain (0-500)
    kd: float  # Derivative gain (0-5)
    q: float  # Desired position in radians
    dq: float  # Desired velocity in radians/second
    tau: float  # Desired torque in Nm


@dataclass
class MotorState:
    """Motor state response data from Damiao motor.

    Reference: DM_CAN.py MotorState structure and recv_data function
    """

    slave_id: int  # Motor slave ID that responded
    position: float  # Motor position in radians
    velocity: float  # Motor velocity in radians/second
    torque: float  # Motor torque in Nm
    temp_mos: int  # MOS temperature
    temp_rotor: int  # Rotor temperature


@dataclass
class SaveResponse:
    """Save parameters response data from Damiao motor.

    Reference: Save parameters response format - custom protocol
    """

    slave_id: int  # Motor slave ID that responded
    success: bool  # Whether the save operation was successful


@dataclass
class PosVelControlParams:
    """Position and velocity control parameters for Damiao motor control.

    Reference: DM_CAN.py control_Pos_Vel function parameters lines 130-135
    """

    position: float  # Desired position in radians
    velocity: float  # Desired velocity in radians/second


@dataclass
class VelControlParams:
    """Velocity control parameters for Damiao motor control.

    Reference: DM_CAN.py control_Vel function parameters lines 150-153
    """

    velocity: float  # Desired velocity in radians/second


@dataclass
class PosForceControlParams:
    """Position and force control parameters for Damiao motor EMIT control.

    Reference: DM_CAN.py control_pos_force function parameters lines 165-170
    """

    position: float  # Desired position in radians
    velocity: float  # Desired velocity in radians/second
    current_norm: float  # Normalized current 0-1


async def decode_register_int(bus: Bus, master_id: int) -> int:
    """Decode register response with integer value. Waits for confirmation response.

    Args:
        bus: CAN bus instance for message reception
        master_id: Motor master ID to wait for response from

    Returns:
        int: Register value

    Reference: DM_CAN.py __process_set_param_packet function lines 291-315

    """
    # Wait for register response with master arbitration ID
    # Timeout to prevent indefinite blocking
    # Reference: Register response handling in DM_CAN.py switchControlMode retry loop
    message = bus.recv(master_id, timeout=0.1)

    # Unpack register response data in single operation
    # Format: '<HBBI' = slave_id(H) + command_code(B) + register_id(B) + value(I)
    # Reference: Register response format in DM_CAN.py __process_set_param_packet
    _, _, _, register_value = struct.unpack("<HBBI", message.data)

    return register_value


async def decode_register_float(bus: Bus, master_id: int) -> float:
    """Decode register response with float value. Waits for confirmation response.

    Args:
        bus: CAN bus instance for message reception
        master_id: Motor master ID to wait for response from

    Returns:
        float: Register value

    Reference: DM_CAN.py __process_set_param_packet function lines 291-315

    """
    # Wait for register response with master arbitration ID
    # Timeout to prevent indefinite blocking
    # Reference: Register response handling in DM_CAN.py switchControlMode retry loop
    message = bus.recv(master_id, timeout=0.1)

    # Unpack register response data in single operation
    # Format: '<HBBf' = slave_id(H) + command_code(B) + register_id(B) + value(f)
    # Reference: Register response format in DM_CAN.py __process_set_param_packet
    _, _, _, register_value = struct.unpack("<HBBf", message.data)

    return register_value


def encode_control_mit(
    bus: Bus, slave_id: int, motor_limits: MotorLimits, params: MitControlParams
) -> None:
    """Control motor in MIT mode. Sends CAN message with MIT control parameters.

    Args:
        bus: CAN bus instance for message transmission
        slave_id: Motor slave ID
        motor_limits: Motor physical limits dataclass for parameter scaling
        params: MIT control parameters dataclass

    Decode with: decode_motor_state(bus, master_id, motor_limits)

    Reference: DM_CAN.py controlMIT function lines 90-123

    """
    # Get motor limits for scaling
    # Reference: DM_CAN.py Limit_Param array access in controlMIT
    q_max, dq_max, tau_max = (
        motor_limits.q_max,
        motor_limits.dq_max,
        motor_limits.tau_max,
    )

    # Convert float parameters to unsigned integers using scaling
    # Reference: DM_CAN.py float_to_uint function calls in controlMIT lines 104-112
    kp_uint = _float_to_uint(params.kp, 0, 500, 12)
    kd_uint = _float_to_uint(params.kd, 0, 5, 12)
    q_uint = _float_to_uint(params.q, -q_max, q_max, 16)
    dq_uint = _float_to_uint(params.dq, -dq_max, dq_max, 12)
    tau_uint = _float_to_uint(params.tau, -tau_max, tau_max, 12)

    # Pack data according to MIT control protocol bit layout
    # Format: '<HHHH' = little-endian 4 unsigned 16-bit values with bit manipulation
    # Reference: DM_CAN.py controlMIT data_buf packing lines 114-121
    # Pack as: q(16-bit), dq(12-bit)|kp_high(4-bit), kp_low(8-bit)|kd_high(8-bit),
    # kd_low(4-bit)|tau(12-bit)
    word1 = q_uint  # q: full 16 bits
    word2 = (dq_uint << 4) | ((kp_uint >> 8) & 0xF)  # dq(12) + kp_high(4)
    word3 = (kp_uint & 0xFF) | ((kd_uint & 0xFF0) << 4)  # kp_low(8) + kd_high(8)
    word4 = ((kd_uint & 0xF) << 12) | (tau_uint & 0xFFF)  # kd_low(4) + tau(12)

    data = struct.pack("<HHHH", word1, word2, word3, word4)

    # Send directly to motor's slave ID for MIT control
    # Reference: DM_CAN.py __send_data call with DM_Motor.SlaveID in controlMIT
    message = can.Message(arbitration_id=slave_id, data=data, is_extended_id=False)
    bus.send(message)


async def decode_motor_state(
    bus: Bus, master_id: int, motor_limits: MotorLimits
) -> MotorState:
    """Decode motor state response. Waits for motor state feedback.

    Args:
        bus: CAN bus instance for message reception
        master_id: Motor master ID to wait for response from
            (motor sends responses on MasterID)
        motor_limits: Motor physical limits dataclass for parameter scaling back
            to engineering units

    Returns:
        MotorState: Motor state dataclass with scaled engineering values

    Reference: DM_CAN.py __process_packet function lines 260-288

    """
    # Wait for motor state response with motor's master ID
    # Timeout to prevent indefinite blocking
    # Reference: Motor state response handling in DM_CAN.py recv calls after controlMIT
    message = bus.recv(master_id, timeout=0.1)

    # Unpack motor state response data according to protocol format
    # Format: '<BHHBB' = little-endian: slave_id(B) + packed_data(2*H) + temps(2*B)
    # Reference: Motor state format in DM_CAN.py __process_packet CMD==0x11 handling
    slave_id, word1, word2, t_mos, t_rotor = struct.unpack("<BHHBB", message.data[:7])

    # Extract packed motor state values using bit operations
    # Reference: DM_CAN.py __process_packet lines 264-266 bit unpacking
    q_uint = word1  # position: full 16 bits
    dq_uint = (word2 >> 4) & 0xFFF  # velocity: high 12 bits
    tau_uint = ((word2 & 0xF) << 8) | (message.data[5] & 0xFF)  # torque: low 4 bits

    # Get motor limits for scaling back to engineering units
    # Reference: DM_CAN.py uint_to_float function calls in __process_packet
    q_max, dq_max, tau_max = (
        motor_limits.q_max,
        motor_limits.dq_max,
        motor_limits.tau_max,
    )

    # Convert unsigned integers back to float engineering values using helper
    # Reference: DM_CAN.py uint_to_float function lines 494-498
    position = _uint_to_float(q_uint, -q_max, q_max, 16)
    velocity = _uint_to_float(dq_uint, -dq_max, dq_max, 12)
    torque = _uint_to_float(tau_uint, -tau_max, tau_max, 12)

    return MotorState(
        slave_id=slave_id,
        position=position,
        velocity=velocity,
        torque=torque,
        temp_mos=t_mos,
        temp_rotor=t_rotor,
    )


async def decode_save_response(bus: Bus, master_id: int) -> SaveResponse:
    """Decode save parameters response. Waits for save confirmation.

    Args:
        bus: CAN bus instance for message reception
        master_id: Motor master ID to wait for response from

    Returns:
        SaveResponse: Save response dataclass with slave_id and success status

    Reference: Save parameters response format - custom protocol

    """
    # Wait for save response with motor's master ID
    # Timeout to prevent indefinite blocking
    message = bus.recv(master_id, timeout=0.1)

    # Unpack save response data
    # Format: '<HBB' = little-endian: slave_id(H) + command(B) + status(B)
    # slave_id: first 2 bytes, command: byte 3 (0xAA), status: byte 4 (0x01=success)
    slave_id, command, status = struct.unpack("<HBB", message.data[:4])

    # Check if valid save response (0xAA cmd) and if it succeeded (0x01 status)
    save_cmd = 0xAA  # Save command identifier
    success_status = 0x01  # Success status code
    success = command == save_cmd and status == success_status

    return SaveResponse(
        slave_id=slave_id,
        success=success,
    )


def encode_read_register(
    bus: Bus, slave_id: int, register_address: RegisterAddress
) -> None:
    """Read motor register value. Sends CAN message to read register.

    Args:
        bus: CAN bus instance for message transmission
        slave_id: Motor slave ID
        register_address: Register address to read

    Decode with: decode_register_int(bus) or decode_register_float(bus)

    Reference: DM_CAN.py __read_RID_param function lines 381-385

    """
    # Pack register read command as structured data
    # Format: '<HBBBBBB' = little-endian: slave_id(H=uint16) + command_bytes(6*B)
    # Reference: Register read format in DM_CAN.py __read_RID_param function
    data = struct.pack(
        "<HBBBBBB",
        slave_id,  # slave_id as 16-bit value
        _READ_REGISTER_CODE,  # 0x33 read register command
        int(register_address),  # register address to read
        0x00,
        0x00,
        0x00,
        0x00,  # padding bytes
    )

    # Send to master arbitration ID 0x7FF for register operations
    # Reference: DM_CAN.py __send_data calls with 0x7FF for register reads
    message = can.Message(arbitration_id=0x7FF, data=data, is_extended_id=False)
    bus.send(message)


def encode_write_register_int(
    bus: Bus, slave_id: int, register_address: RegisterAddress, value: int
) -> None:
    """Write motor register value as integer. Sends CAN message to write register.

    Args:
        bus: CAN bus instance for message transmission
        slave_id: Motor slave ID
        register_address: Register address to write
        value: Integer value to write to register

    Decode with: decode_register_int(bus) or decode_register_float(bus)

    Reference: DM_CAN.py __write_motor_param function lines 387-397 (int path)

    """
    # Pack as uint32 (4 bytes)
    # Format: '<HBBI' = slave_id(H) + command(B) + address(B) + value(I=uint32)
    # Reference: Integer register write format in DM_CAN.py __write_motor_param
    data = struct.pack(
        "<HBBI",
        slave_id,  # slave_id as 16-bit value
        _WRITE_REGISTER_CODE,  # 0x55 write register command
        int(register_address),  # register address to write
        int(value),  # value as 32-bit unsigned integer
    )

    # Send to master arbitration ID 0x7FF for register operations
    # Reference: DM_CAN.py __send_data calls with 0x7FF for register writes
    message = can.Message(arbitration_id=0x7FF, data=data, is_extended_id=False)
    bus.send(message)


def encode_write_register_float(
    bus: Bus, slave_id: int, register_address: RegisterAddress, value: float
) -> None:
    """Write motor register value as float. Sends CAN message to write register.

    Args:
        bus: CAN bus instance for message transmission
        slave_id: Motor slave ID
        register_address: Register address to write
        value: Float value to write to register

    Decode with: decode_register_int(bus) or decode_register_float(bus)

    Reference: DM_CAN.py __write_motor_param function lines 387-397 (float path)

    """
    # Pack as float (4 bytes)
    # Format: '<HBBf' = slave_id(H) + commands(2*B) + value(f=float)
    # Reference: Float register write format in DM_CAN.py __write_motor_param
    data = struct.pack(
        "<HBBf",
        slave_id,  # slave_id as 16-bit value
        _WRITE_REGISTER_CODE,  # 0x55 write register command
        int(register_address),  # register address to write
        float(value),  # value as 32-bit float
    )

    # Send to master arbitration ID 0x7FF for register operations
    # Reference: DM_CAN.py __send_data calls with 0x7FF for register writes
    message = can.Message(arbitration_id=0x7FF, data=data, is_extended_id=False)
    bus.send(message)


def encode_save_parameters(bus: Bus, slave_id: int) -> None:
    """Save motor parameters to flash. Sends CAN message to save all parameters.

    Args:
        bus: CAN bus instance for message transmission
        slave_id: Motor slave ID

    Decode with: decode_save_response(bus, master_id)

    Reference: DM_CAN.py save_motor_param function lines 420-431

    """
    # Pack save parameters command as structured data
    # Format: '<HBBBBBB' = little-endian: slave_id(H=uint16) + command_bytes(6*B)
    # Reference: Save command format in DM_CAN.py save_motor_param function
    data = struct.pack(
        "<HBBBBBB",
        slave_id,  # slave_id as 16-bit value
        0xAA,  # 0xAA save parameters command
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,  # padding bytes
    )

    # Send to master arbitration ID 0x7FF for register operations
    # Reference: DM_CAN.py __send_data calls with 0x7FF for save operations
    message = can.Message(arbitration_id=0x7FF, data=data, is_extended_id=False)
    bus.send(message)


def encode_refresh_status(bus: Bus, slave_id: int) -> None:
    """Refresh motor status. Sends CAN message to get current motor state.

    Args:
        bus: CAN bus instance for message transmission
        slave_id: Motor slave ID

    Decode with: decode_motor_state(bus, master_id, motor_limits)

    Reference: DM_CAN.py refresh_motor_status function lines 446-455

    """
    # Pack refresh status command as structured data
    # Format: '<HBBBBBB' = little-endian: slave_id(H=uint16) + command_bytes(6*B)
    # Reference: Refresh command format in DM_CAN.py refresh_motor_status function
    data = struct.pack(
        "<HBBBBBB",
        slave_id,  # slave_id as 16-bit value
        0xCC,  # 0xCC refresh status command
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,  # padding bytes
    )

    # Send to master arbitration ID 0x7FF for register operations
    # Reference: DM_CAN.py __send_data calls with 0x7FF for refresh operations
    message = can.Message(arbitration_id=0x7FF, data=data, is_extended_id=False)
    bus.send(message)


def encode_control_pos_vel(
    bus: Bus, slave_id: int, params: PosVelControlParams
) -> None:
    """Control motor in position and velocity mode. Sends CAN message with params.

    Args:
        bus: CAN bus instance for message transmission
        slave_id: Motor slave ID
        params: Position and velocity control parameters dataclass

    Decode with: decode_motor_state(bus, master_id, motor_limits)

    Reference: DM_CAN.py control_Pos_Vel function lines 130-148

    """
    # Pack position and velocity as two 32-bit little-endian floats
    # Format: '<ff' = little-endian position(f=float) + velocity(f=float)
    # Reference: Position/velocity packing in DM_CAN.py control_Pos_Vel lines 142-145
    data = struct.pack("<ff", params.position, params.velocity)

    # Send to motor ID + 0x100 offset for position/velocity control mode
    # Reference: DM_CAN.py control_Pos_Vel motorid calculation line 140
    message = can.Message(
        arbitration_id=0x100 + slave_id, data=data, is_extended_id=False
    )
    bus.send(message)


def encode_control_vel(bus: Bus, slave_id: int, params: VelControlParams) -> None:
    """Control motor in velocity mode. Sends CAN message with velocity parameters.

    Args:
        bus: CAN bus instance for message transmission
        slave_id: Motor slave ID
        params: Velocity control parameters dataclass

    Decode with: decode_motor_state(bus, master_id, motor_limits)

    Reference: DM_CAN.py control_Vel function lines 150-163

    """
    # Pack velocity as 32-bit little-endian float with 4 bytes of padding
    # Format: '<fBBBB' = little-endian velocity(f=float) + padding(4*B=4 bytes)
    # Reference: Velocity packing in DM_CAN.py control_Vel lines 160-161
    data = struct.pack("<fBBBB", params.velocity, 0x00, 0x00, 0x00, 0x00)

    # Send to motor ID + 0x200 offset for velocity control mode
    # Reference: DM_CAN.py control_Vel motorid calculation line 158
    message = can.Message(
        arbitration_id=0x200 + slave_id, data=data, is_extended_id=False
    )
    bus.send(message)


def encode_control_torque_pos(
    bus: Bus, slave_id: int, params: PosForceControlParams
) -> None:
    """Control motor in position+force mode (EMIT). Sends CAN message with params.

    Args:
        bus: CAN bus instance for message transmission
        slave_id: Motor slave ID
        params: Position and force control parameters dataclass

    Decode with: decode_motor_state(bus, master_id, motor_limits)

    Reference: DM_CAN.py control_pos_force function lines 165-186

    """
    # Clamp current_norm to valid range [0, 1]
    # Reference: DM_CAN.py control_pos_force current range documentation lines 169-170
    current_norm = max(0.0, min(params.current_norm, 1.0))

    # Scale velocity and current according to protocol requirements
    # Reference: DM_CAN.py control_pos_force scaling lines 179-180
    vel_scaled = int(params.velocity * 100)  # Velocity scaled by 100
    current_scaled = int(current_norm * 10000)  # Current scaled to 0-10000 range

    # Pack position as float + velocity as uint16 + current as uint16
    # Format: '<fHH' = little-endian position(f=float) + velocity(H=uint16) +
    # current(H=uint16)
    # Reference: Position/force packing in DM_CAN.py control_pos_force lines 177-184
    data = struct.pack("<fHH", params.position, vel_scaled, current_scaled)

    # Send to motor ID + 0x300 offset for position/force control mode
    # Reference: DM_CAN.py control_pos_force motorid calculation line 175
    message = can.Message(
        arbitration_id=0x300 + slave_id, data=data, is_extended_id=False
    )
    bus.send(message)


def encode_enable_motor(bus: Bus, slave_id: int) -> None:
    """Enable motor. Sends CAN message to enable motor operation.

    Args:
        bus: CAN bus instance for message transmission
        slave_id: Motor slave ID

    Decode with: decode_motor_state(bus, master_id, motor_limits)

    Reference: DM_CAN.py enable function lines 188-195

    """
    # Pack enable command with 0xFC command code
    # Format: '<BBBBBBBB' = 8 bytes with 0xFF padding + 0xFC command
    # Reference: Enable command format in DM_CAN.py __control_cmd lines 309-311
    data = struct.pack("<BBBBBBBB", 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC)

    # Send directly to motor's slave ID for control commands
    # Reference: DM_CAN.py __control_cmd usage in enable function line 193
    message = can.Message(arbitration_id=slave_id, data=data, is_extended_id=False)
    bus.send(message)


def encode_disable_motor(bus: Bus, slave_id: int) -> None:
    """Disable motor. Sends CAN message to disable motor operation.

    Args:
        bus: CAN bus instance for message transmission
        slave_id: Motor slave ID

    Decode with: decode_motor_state(bus, master_id, motor_limits)

    Reference: DM_CAN.py disable function lines 209-215

    """
    # Pack disable command with 0xFD command code
    # Format: '<BBBBBBBB' = 8 bytes with 0xFF padding + 0xFD command
    # Reference: Disable command format in DM_CAN.py __control_cmd lines 309-311
    data = struct.pack("<BBBBBBBB", 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD)

    # Send directly to motor's slave ID for control commands
    # Reference: DM_CAN.py __control_cmd usage in disable function line 213
    message = can.Message(arbitration_id=slave_id, data=data, is_extended_id=False)
    bus.send(message)


def encode_set_zero_position(bus: Bus, slave_id: int) -> None:
    """Set motor zero position. Sends CAN message to set current position as zero ref.

    Args:
        bus: CAN bus instance for message transmission
        slave_id: Motor slave ID

    Decode with: decode_motor_state(bus, master_id, motor_limits)

    Reference: DM_CAN.py set_zero_position function lines 217-223

    """
    # Pack set zero command with 0xFE command code
    # Format: '<BBBBBBBB' = 8 bytes with 0xFF padding + 0xFE command
    # Reference: Set zero command format in DM_CAN.py __control_cmd lines 309-311
    data = struct.pack("<BBBBBBBB", 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE)

    # Send directly to motor's slave ID for control commands
    # Reference: DM_CAN.py __control_cmd usage in set_zero_position function line 221
    message = can.Message(arbitration_id=slave_id, data=data, is_extended_id=False)
    bus.send(message)
