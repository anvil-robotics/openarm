# Motor Testing Guide

This guide provides step-by-step instructions for testing Damiao motors.

## Prerequisites

- Motors connected to CAN bus interface (e.g., `can0`)
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

### 2. Enable Motors

Enable motors before controlling them:

**J1 (DM8009):**
```bash
python -m openarm.damiao enable --motor-type DM8009 --iface can0 1 17
```

**J2 (DM8009):**
```bash
python -m openarm.damiao enable --motor-type DM8009 --iface can0 2 18
```

**J3 (DM4340):**
```bash
python -m openarm.damiao enable --motor-type DM4340 --iface can0 3 19
```

**J4 (DM4340):**
```bash
python -m openarm.damiao enable --motor-type DM4340 --iface can0 4 20
```

**J5 (DM4310):**
```bash
python -m openarm.damiao enable --motor-type DM4310 --iface can0 5 21
```

**J6 (DM4310):**
```bash
python -m openarm.damiao enable --motor-type DM4310 --iface can0 6 22
```

**J7 (DM4310):**
```bash
python -m openarm.damiao enable --motor-type DM4310 --iface can0 7 23
```

**J8 (DM4310):**
```bash
python -m openarm.damiao enable --motor-type DM4310 --iface can0 8 24
```

### 3. Set Control Mode to Position/Velocity

Set each motor to position-velocity control mode:

**J1 (DM8009):**
```bash
python -m openarm.damiao param set --motor-type DM8009 --iface can0 1 17 control_mode POS_VEL
```

**J2 (DM8009):**
```bash
python -m openarm.damiao param set --motor-type DM8009 --iface can0 2 18 control_mode POS_VEL
```

**J3 (DM4340):**
```bash
python -m openarm.damiao param set --motor-type DM4340 --iface can0 3 19 control_mode POS_VEL
```

**J4 (DM4340):**
```bash
python -m openarm.damiao param set --motor-type DM4340 --iface can0 4 20 control_mode POS_VEL
```

**J5 (DM4310):**
```bash
python -m openarm.damiao param set --motor-type DM4310 --iface can0 5 21 control_mode POS_VEL
```

**J6 (DM4310):**
```bash
python -m openarm.damiao param set --motor-type DM4310 --iface can0 6 22 control_mode POS_VEL
```

**J7 (DM4310):**
```bash
python -m openarm.damiao param set --motor-type DM4310 --iface can0 7 23 control_mode POS_VEL
```

**J8 (DM4310):**
```bash
python -m openarm.damiao param set --motor-type DM4310 --iface can0 8 24 control_mode POS_VEL
```

### 4. Move Motors to Zero Position

Move each motor to zero position (0.0 radians):

**J1 (DM8009):**
```bash
python -m openarm.damiao control pos_vel --motor-type DM8009 --iface can0 1 17 0.0 2.0
```

**J2 (DM8009):**
```bash
python -m openarm.damiao control pos_vel --motor-type DM8009 --iface can0 2 18 0.0 2.0
```

**J3 (DM4340):**
```bash
python -m openarm.damiao control pos_vel --motor-type DM4340 --iface can0 3 19 0.0 2.0
```

**J4 (DM4340):**
```bash
python -m openarm.damiao control pos_vel --motor-type DM4340 --iface can0 4 20 0.0 2.0
```

**J5 (DM4310):**
```bash
python -m openarm.damiao control pos_vel --motor-type DM4310 --iface can0 5 21 0.0 2.0
```

**J6 (DM4310):**
```bash
python -m openarm.damiao control pos_vel --motor-type DM4310 --iface can0 6 22 0.0 2.0
```

**J7 (DM4310):**
```bash
python -m openarm.damiao control pos_vel --motor-type DM4310 --iface can0 7 23 0.0 2.0
```

**J8 (DM4310):**
```bash
python -m openarm.damiao control pos_vel --motor-type DM4310 --iface can0 8 24 0.0 2.0
```

### 5. Move Motors to Small Positions Around Zero

Test motor movement with small position values (±0.1 radians ≈ ±5.7 degrees):

**J1 (DM8009) - Move to +0.1 rad:**
```bash
python -m openarm.damiao control pos_vel --motor-type DM8009 --iface can0 1 17 0.1 1.0
```

**J2 (DM8009) - Move to +0.1 rad:**
```bash
python -m openarm.damiao control pos_vel --motor-type DM8009 --iface can0 2 18 0.1 1.0
```

**J3 (DM4340) - Move to +0.1 rad:**
```bash
python -m openarm.damiao control pos_vel --motor-type DM4340 --iface can0 3 19 0.1 1.0
```

**J4 (DM4340) - Move to +0.1 rad:**
```bash
python -m openarm.damiao control pos_vel --motor-type DM4340 --iface can0 4 20 0.1 1.0
```

**J5 (DM4310) - Move to +0.1 rad:**
```bash
python -m openarm.damiao control pos_vel --motor-type DM4310 --iface can0 5 21 0.1 1.0
```

**J6 (DM4310) - Move to +0.1 rad:**
```bash
python -m openarm.damiao control pos_vel --motor-type DM4310 --iface can0 6 22 0.1 1.0
```

**J7 (DM4310) - Move to +0.1 rad:**
```bash
python -m openarm.damiao control pos_vel --motor-type DM4310 --iface can0 7 23 0.1 1.0
```

**J8 (DM4310) - Move to +0.1 rad:**
```bash
python -m openarm.damiao control pos_vel --motor-type DM4310 --iface can0 8 24 0.1 1.0
```

### 6. Disable Motors (Optional)

After testing, disable each motor:

**J1 (DM8009):**
```bash
python -m openarm.damiao disable --motor-type DM8009 --iface can0 1 17
```

**J2 (DM8009):**
```bash
python -m openarm.damiao disable --motor-type DM8009 --iface can0 2 18
```

**J3 (DM4340):**
```bash
python -m openarm.damiao disable --motor-type DM4340 --iface can0 3 19
```

**J4 (DM4340):**
```bash
python -m openarm.damiao disable --motor-type DM4340 --iface can0 4 20
```

**J5 (DM4310):**
```bash
python -m openarm.damiao disable --motor-type DM4310 --iface can0 5 21
```

**J6 (DM4310):**
```bash
python -m openarm.damiao disable --motor-type DM4310 --iface can0 6 22
```

**J7 (DM4310):**
```bash
python -m openarm.damiao disable --motor-type DM4310 --iface can0 7 23
```

**J8 (DM4310):**
```bash
python -m openarm.damiao disable --motor-type DM4310 --iface can0 8 24
```

## Safety Notes

- Always start with small position values when testing
- Monitor motor temperature during testing
- Keep velocity values reasonable (1.0-5.0 rad/s for initial tests)
- Ensure the motor has clearance to move before sending commands
- Have emergency stop procedures ready

## Reference Motor Configuration

| Motor | Type   | Slave ID | Master ID | Min Angle | Max Angle | Notes |
|-------|--------|----------|-----------|-----------|-----------|-------|
| J1    | DM8009 | 1 (0x01) | 17 (0x11) | -200°     | +80°      | Inverted |
| J2    | DM8009 | 2 (0x02) | 18 (0x12) | -190°     | +10°      | Inverted |
| J3    | DM4340 | 3 (0x03) | 19 (0x13) | -90°      | +90°      | Inverted |
| J4    | DM4340 | 4 (0x04) | 20 (0x14) | 0°        | +140°     | Not inverted |
| J5    | DM4310 | 5 (0x05) | 21 (0x15) | -90°      | +90°      | Inverted |
| J6    | DM4310 | 6 (0x06) | 22 (0x16) | -45°      | +45°      | Inverted |
| J7    | DM4310 | 7 (0x07) | 23 (0x17) | -90°      | +90°      | Inverted |
| J8    | DM4310 | 8 (0x08) | 24 (0x18) | -45°      | 0°        | Not inverted |
