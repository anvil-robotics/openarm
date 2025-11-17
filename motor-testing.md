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

### 2. Test Each Motor

Test motors individually using the commands below. Each motor follows this sequence:

1. Enable motor
2. Set control mode to POS_VEL
3. Move to zero position (0.0 rad)
4. Wait 2 seconds
5. Move to test position (+0.3 rad)
6. Wait 2 seconds
7. Move back to zero position (0.0 rad)
8. Wait 2 seconds
9. Disable motor

#### J1 (DM8009)

```bash
python -m openarm.damiao enable --motor-type DM8009 --iface follower_l 1 17
python -m openarm.damiao param set --motor-type DM8009 --iface follower_l 1 17 control_mode 2
python -m openarm.damiao control pos_vel --motor-type DM8009 --iface follower_l 1 17 0.0 0.2
sleep 2
python -m openarm.damiao control pos_vel --motor-type DM8009 --iface follower_l 1 17 0.3 0.2
sleep 2
python -m openarm.damiao control pos_vel --motor-type DM8009 --iface follower_l 1 17 0.0 0.2
sleep 2
python -m openarm.damiao disable --motor-type DM8009 --iface follower_l 1 17
```

#### J2 (DM8009)

> [!CAUTION]
> J2 movement direction is critical to avoid collision with the pedestal.
>
> Left arm (`follower_l`, `robot_l`): Use -0.15 rad to move away from pedestal (safe direction)
> Right arm (`follower_r`, `robot_r`): Use +0.15 rad to move away from pedestal (safe direction)

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

#### J3 (DM4340)

```bash
python -m openarm.damiao enable --motor-type DM4340 --iface follower_l 3 19
python -m openarm.damiao param set --motor-type DM4340 --iface follower_l 3 19 control_mode 2
python -m openarm.damiao control pos_vel --motor-type DM4340 --iface follower_l 3 19 0.0 0.2
sleep 2
python -m openarm.damiao control pos_vel --motor-type DM4340 --iface follower_l 3 19 0.3 0.2
sleep 2
python -m openarm.damiao control pos_vel --motor-type DM4340 --iface follower_l 3 19 0.0 0.2
sleep 2
python -m openarm.damiao disable --motor-type DM4340 --iface follower_l 3 19
```

#### J4 (DM4340)

```bash
python -m openarm.damiao enable --motor-type DM4340 --iface follower_l 4 20
python -m openarm.damiao param set --motor-type DM4340 --iface follower_l 4 20 control_mode 2
python -m openarm.damiao control pos_vel --motor-type DM4340 --iface follower_l 4 20 0.0 0.2
sleep 2
python -m openarm.damiao control pos_vel --motor-type DM4340 --iface follower_l 4 20 0.3 0.2
sleep 2
python -m openarm.damiao control pos_vel --motor-type DM4340 --iface follower_l 4 20 0.0 0.2
sleep 2
python -m openarm.damiao disable --motor-type DM4340 --iface follower_l 4 20
```

#### J5 (DM4310)

```bash
python -m openarm.damiao enable --motor-type DM4310 --iface follower_l 5 21
python -m openarm.damiao param set --motor-type DM4310 --iface follower_l 5 21 control_mode 2
python -m openarm.damiao control pos_vel --motor-type DM4310 --iface follower_l 5 21 0.0 0.2
sleep 2
python -m openarm.damiao control pos_vel --motor-type DM4310 --iface follower_l 5 21 0.3 0.2
sleep 2
python -m openarm.damiao control pos_vel --motor-type DM4310 --iface follower_l 5 21 0.0 0.2
sleep 2
python -m openarm.damiao disable --motor-type DM4310 --iface follower_l 5 21
```

#### J6 (DM4310)

```bash
python -m openarm.damiao enable --motor-type DM4310 --iface follower_l 6 22
python -m openarm.damiao param set --motor-type DM4310 --iface follower_l 6 22 control_mode 2
python -m openarm.damiao control pos_vel --motor-type DM4310 --iface follower_l 6 22 0.0 0.2
sleep 2
python -m openarm.damiao control pos_vel --motor-type DM4310 --iface follower_l 6 22 0.3 0.2
sleep 2
python -m openarm.damiao control pos_vel --motor-type DM4310 --iface follower_l 6 22 0.0 0.2
sleep 2
python -m openarm.damiao disable --motor-type DM4310 --iface follower_l 6 22
```

#### J7 (DM4310)

```bash
python -m openarm.damiao enable --motor-type DM4310 --iface follower_l 7 23
python -m openarm.damiao param set --motor-type DM4310 --iface follower_l 7 23 control_mode 2
python -m openarm.damiao control pos_vel --motor-type DM4310 --iface follower_l 7 23 0.0 0.2
sleep 2
python -m openarm.damiao control pos_vel --motor-type DM4310 --iface follower_l 7 23 0.3 0.2
sleep 2
python -m openarm.damiao control pos_vel --motor-type DM4310 --iface follower_l 7 23 0.0 0.2
sleep 2
python -m openarm.damiao disable --motor-type DM4310 --iface follower_l 7 23
```

#### J8 (DM4310)

```bash
python -m openarm.damiao enable --motor-type DM4310 --iface follower_l 8 24
python -m openarm.damiao param set --motor-type DM4310 --iface follower_l 8 24 control_mode 2
python -m openarm.damiao control pos_vel --motor-type DM4310 --iface follower_l 8 24 0.0 0.2
sleep 2
python -m openarm.damiao control pos_vel --motor-type DM4310 --iface follower_l 8 24 -0.3 0.2
sleep 2
python -m openarm.damiao control pos_vel --motor-type DM4310 --iface follower_l 8 24 0.0 0.2
sleep 2
python -m openarm.damiao disable --motor-type DM4310 --iface follower_l 8 24
```

## Safety Notes

- Always start with small position values when testing
- Monitor motor temperature during testing
- Keep velocity values reasonable (1.0-5.0 rad/s for initial tests)
- Ensure the motor has clearance to move before sending commands
- Have emergency stop procedures ready

## Reference Motor Configuration

| Motor | Type   | Slave ID | Master ID | Min Angle | Max Angle | Notes        |
| ----- | ------ | -------- | --------- | --------- | --------- | ------------ |
| J1    | DM8009 | 1 (0x01) | 17 (0x11) | -200°     | +80°      | Inverted     |
| J2    | DM8009 | 2 (0x02) | 18 (0x12) | -190°     | +10°      | Inverted     |
| J3    | DM4340 | 3 (0x03) | 19 (0x13) | -90°      | +90°      | Inverted     |
| J4    | DM4340 | 4 (0x04) | 20 (0x14) | 0°        | +140°     | Not inverted |
| J5    | DM4310 | 5 (0x05) | 21 (0x15) | -90°      | +90°      | Inverted     |
| J6    | DM4310 | 6 (0x06) | 22 (0x16) | -45°      | +45°      | Inverted     |
| J7    | DM4310 | 7 (0x07) | 23 (0x17) | -90°      | +90°      | Inverted     |
| J8    | DM4310 | 8 (0x08) | 24 (0x18) | -45°      | 0°        | Not inverted |
