# Motor Testing Guide

This guide provides step-by-step instructions for testing Damiao motors.

## Prerequisites

- Motors connected to CAN bus interface (e.g. `leader_l`, `leader_r`, `follower_l`, `follower_r`)
- Motor IDs and types configured in `openarm/damiao/config.py`

## Testing Steps

### 1. Detect Motors

Scan for connected motors on the CAN bus:

```bash
python -m openarm.damiao.detect --interface socketcan
```

This command will:

- Detect all available CAN buses
- Scan for motors on each bus
- Display motor status with slave ID and master ID
- Verify motors match expected configuration

### 2. Automated Motor Testing (Recommended)

Use the automated test script to check all motors sequentially.

**General command:**

```bash
python -m openarm.damiao.arm_motor_check --iface <interface>  --side {left | right}
```

Replace <interface> with your actual CAN bus device name (e.g., `follower_l`, `follower_r`) and {left | right} with the arm side.

> [!CAUTION]
> Please ensure the side is correct because the test positions are different for left and right arms.

**Examples:**

```bash
# For left arm
python -m openarm.damiao.arm_motor_check --iface follower_l --side left

# For right arm
python -m openarm.damiao.arm_motor_check --iface follower_r --side right
```

This automated script will:

- Verify all motors are present on the CAN bus
- Test each motor sequentially through a safe movement sequence
- Use side-specific test positions to avoid collisions
- Automatically handle enable/disable for each motor
- Provide a summary report with pass/fail status

### 3. Manual Motor Testing (Advanced)

For troubleshooting specific motors or testing individual joints, you can use the manual commands below.

The test sequence for each motor is as follows:

1. Enable motor
2. Set control mode to POS_VEL (mode 2)
3. Move to zero position (0.0 rad)
4. Wait 2 seconds
5. Move to test position (adjust based on motor and safety constraints)
6. Wait 2 seconds
7. Move back to zero position (0.0 rad)
8. Wait 2 seconds
9. Disable motor

```bash
python -m openarm.damiao enable --motor-type <MOTOR_TYPE> --iface <INTERFACE> <SLAVE_ID> <MASTER_ID>
python -m openarm.damiao param set --motor-type <MOTOR_TYPE> --iface <INTERFACE> <SLAVE_ID> <MASTER_ID> control_mode 2
python -m openarm.damiao control pos_vel --motor-type <MOTOR_TYPE> --iface <INTERFACE> <SLAVE_ID> <MASTER_ID> 0.0 0.2
sleep 2
python -m openarm.damiao control pos_vel --motor-type <MOTOR_TYPE> --iface <INTERFACE> <SLAVE_ID> <MASTER_ID> <TEST_POSITION> 0.2
sleep 2
python -m openarm.damiao control pos_vel --motor-type <MOTOR_TYPE> --iface <INTERFACE> <SLAVE_ID> <MASTER_ID> 0.0 0.2
sleep 2
python -m openarm.damiao disable --motor-type <MOTOR_TYPE> --iface <INTERFACE> <SLAVE_ID> <MASTER_ID>
```

**Parameters:**

- `<MOTOR_TYPE>`: Motor model (DM8009, DM4340, or DM4310)
- `<INTERFACE>`: CAN bus interface (e.g., `follower_l`, `follower_r`, `can0`)
- `<SLAVE_ID>`: Motor slave ID (1-8 for J1-J8)
- `<MASTER_ID>`: Motor master ID (17-24 for J1-J8, typically `slave_id + 16`)
- `<TEST_POSITION>`: Target position in radians (e.g., 0.3, -0.15)

**Motor Configuration Reference:**

**Left arm:**

| Motor | Type   | Slave ID | Master ID | Min Angle     | Max Angle     |
| ----- | ------ | -------- | --------- | ------------- | ------------- |
| J1    | DM8009 | 1 (0x01) | 17 (0x11) | -200° (-3.49) | +80° (+1.39)  |
| J2    | DM8009 | 2 (0x02) | 18 (0x12) | -190° (-3.31) | +10° (+0.17)  |
| J3    | DM4340 | 3 (0x03) | 19 (0x13) | -90° (-1.57)  | +90° (+1.57)  |
| J4    | DM4340 | 4 (0x04) | 20 (0x14) | 0° (0.00)     | +140° (+2.44) |
| J5    | DM4310 | 5 (0x05) | 21 (0x15) | -90° (-1.57)  | +90° (+1.57)  |
| J6    | DM4310 | 6 (0x06) | 22 (0x16) | -45° (-0.78)  | +45° (+0.78)  |
| J7    | DM4310 | 7 (0x07) | 23 (0x17) | -90° (-1.57)  | +90° (+1.57)  |
| J8    | DM4310 | 8 (0x08) | 24 (0x18) | -45° (-0.78)  | 0° (0.00)     |

**Right arm:**

| Motor | Type   | Slave ID | Master ID | Min Angle    | Max Angle     |
| ----- | ------ | -------- | --------- | ------------ | ------------- |
| J1    | DM8009 | 1 (0x01) | 17 (0x11) | -80° (-1.39) | +200° (+3.49) |
| J2    | DM8009 | 2 (0x02) | 18 (0x12) | -10° (-0.17) | +190° (+3.31) |
| J3    | DM4340 | 3 (0x03) | 19 (0x13) | -90° (-1.57) | +90° (+1.57)  |
| J4    | DM4340 | 4 (0x04) | 20 (0x14) | 0° (0.00)    | +140° (+2.44) |
| J5    | DM4310 | 5 (0x05) | 21 (0x15) | -90° (-1.57) | +90° (+1.57)  |
| J6    | DM4310 | 6 (0x06) | 22 (0x16) | -45° (-0.78) | +45° (+0.78)  |
| J7    | DM4310 | 7 (0x07) | 23 (0x17) | -90° (-1.57) | +90° (+1.57)  |
| J8    | DM4310 | 8 (0x08) | 24 (0x18) | -45° (-0.78) | 0° (0.00)     |

_Note: Radian values are shown in parentheses. Always ensure test positions are within these limits._

#### Example Command for J2 (DM8009)

```bash
python -m openarm.damiao enable --motor-type DM8009 --iface follower_l 2 18
python -m openarm.damiao param set --motor-type DM8009 --iface follower_l 2 18 control_mode 2
python -m openarm.damiao control pos_vel --motor-type DM8009 --iface follower_l 2 18 0.0 0.2
sleep 2
# Safety Note: please ensure the arm is moving away from the pedestal for safety
python -m openarm.damiao control pos_vel --motor-type DM8009 --iface follower_l 2 18 -0.15 0.2
sleep 2
python -m openarm.damiao control pos_vel --motor-type DM8009 --iface follower_l 2 18 0.0 0.2
sleep 2
python -m openarm.damiao disable --motor-type DM8009 --iface follower_l 2 18
```

> [!CAUTION]
> J2 movement direction is critical to avoid collision with the pedestal.
>
> Left arm (`follower_l`, `robot_l`): Use -0.15 rad to move away from pedestal (safe direction)
>
> Right arm (`follower_r`, `robot_r`): Use +0.15 rad to move away from pedestal (safe direction)

## Safety Notes

- Always start with small position values when testing
- Monitor motor temperature during testing
- Keep velocity values reasonable (1.0-5.0 rad/s for initial tests)
- Ensure the motor has clearance to move before sending commands
- Have emergency stop procedures ready
