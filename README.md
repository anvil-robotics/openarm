# OpenArm

A Python package for robotic arm control and automation.

## Installation

### Prerequisites

- Python 3.11 or higher

### Setup

1. **Create a virtual environment** (recommended):

   ```bash
   # Using venv
   python -m venv .venv
   source .venv/bin/activate  # On Windows: .venv\Scripts\activate

   # Or using conda/mamba
   conda create -n openarm python=3.11
   conda activate openarm
   ```

2. **Install OpenArm in development mode**:

   ```bash
   pip install -e .
   ```

   This installs all required dependencies including:
   - `python-can` for CAN bus communication
   - `ikpy` for inverse kinematics
   - `mujoco` for physics simulation

## Usage

OpenArm provides multiple tools for controlling and monitoring Damiao servo motors. All commands work with any CAN interface and automatically detect available motor configurations.

### Motor Configuration

Configure Damiao motor IDs, set zero positions, and save parameters:

```bash
# Configure motor with specific master and slave IDs
python -m openarm.damiao.configure --channel can0 --set-master 0x11 --set-slave 0x01

# Configure motor as a predefined joint (J1-J8)
python -m openarm.damiao.configure --channel can0 --set-motor J1

# Set both custom IDs and zero position, then save
python -m openarm.damiao.configure --channel can0 --set-master 0x12 --set-slave 0x02 --set-zero --save

# Configure multiple motors (requires --allow-multiple)
python -m openarm.damiao.configure --channel can0 --set-motor J3 --allow-multiple --save

# Override motor type IDs while using predefined configuration
python -m openarm.damiao.configure --channel can0 --set-motor J1 --set-master 0x20
```

**Arguments:**
- `--channel CHANNEL`: CAN channel (required, e.g., can0, can1)
- `--interface INTERFACE`: CAN interface type (default: socketcan)
- `--set-master ID`: Set master ID (hex: 0x11 or decimal: 17)
- `--set-slave ID`: Set slave ID (hex: 0x01 or decimal: 1)
- `--set-motor MOTOR`: Configure as predefined motor (J1-J8)
- `--set-zero`: Set current position as zero
- `--save`: Save parameters to motor flash memory
- `--allow-multiple`: Allow configuring multiple motors at once

**Notes:**
- When using `--set-motor`, both master and slave IDs are set from predefined configuration
- `--set-master` or `--set-slave` override the predefined values from `--set-motor`
- Motors are auto-detected on the specified channel
- The script assumes DM8009 motor type by default


### CAN Interface Setup (Linux only)

Before using any motor control commands, set up the CAN interfaces:

```bash
sudo ./scripts/setup_can.sh
```

This script configures the CAN devices with proper interface names (leader_left, leader_right, follower_left, follower_right) and sets the bitrate to 1 Mbps.

### Zero Position Calibration

Set all motors to zero position automatically across all detected CAN buses:

```bash
# Set zero position for all motors on all buses
python -m openarm.damiao.set_zero

# Specify custom CAN interface (Linux only)
python -m openarm.damiao.set_zero --interface can1
```

### Motor Monitoring

Monitor motor angles in real-time across all detected CAN buses:

```bash
# Monitor all motors (passive mode - motors disabled)
python -m openarm.damiao.monitor

# Specify custom CAN interface (Linux only)
python -m openarm.damiao.monitor --interface can1
```

### Direct Motor Control

Low-level control of individual motors for testing and debugging:

```bash
# Enable a motor
python -m openarm.damiao enable --motor-type DM4310 --iface can0 1 1

# Control motor with MIT parameters
python -m openarm.damiao control mit --motor-type DM4310 --iface can0 1 1 \
    50 0.3 0 0 0  # kp kd q dq tau

# Position/velocity control
python -m openarm.damiao control pos_vel --motor-type DM4310 --iface can0 1 1 \
    1.57 2.0  # position(rad) velocity(rad/s)

# Get motor status
python -m openarm.damiao refresh --motor-type DM4310 --iface can0 1 1

# Get/set motor parameters
python -m openarm.damiao param get --motor-type DM4310 --iface can0 1 1 over_voltage
python -m openarm.damiao param set --motor-type DM4310 --iface can0 1 1 max_speed 10.0

# Disable motor safely
python -m openarm.damiao disable --motor-type DM4310 --iface can0 1 1
```

**Common Arguments:**

- `--motor-type TYPE`: Motor type (`DM4310`, `DM4340`, `DM6006`, `DM8006`, `DM8009`)
- `--iface INTERFACE`: CAN interface (default: `can0`)
- `slave_id`: Motor ID for commands
- `master_id`: Master ID for responses

**Available Commands:**

- `enable/disable`: Motor power control
- `set-zero`: Set current position as zero
- `refresh`: Get current motor state
- `control`: MIT, position/velocity, velocity, or position/force control
- `param get/set`: Read/write motor parameters
- `save`: Save parameters to flash memory

### Gravity Compensation

Run physics-based gravity compensation on specific arms:

```bash
# Single arm gravity compensation
python -m openarm.damiao.gravity --port can0:left

# Multiple arms simultaneously
python -m openarm.damiao.gravity --port can0:left --port can1:right
```

**Options:**

- `--port INTERFACE:POSITION`: Specify CAN interface and arm position
  - `POSITION` must be `left` or `right`
  - Can specify multiple `--port` arguments

### Teleoperation

Enable teleoperation where master arms control slave arms in real-time:

```bash
# Basic teleoperation (first bus masters others)
python -m openarm.damiao.monitor --teleop

# Custom master-slave with automatic mirror detection
python -m openarm.damiao.monitor -t --follow can0:left:can1:right

# Multiple master-slave pairs
python -m openarm.damiao.monitor -t --follow can1:left:can2:right --follow can0:right:can3:left

# With gravity compensation and custom velocity
python -m openarm.damiao.monitor -t --follow can1:left:can2:right --gravity --velocity 5
```

**Options:**

- `-t, --teleop`: Enable teleoperation mode
- `--follow MASTER:POS:SLAVE:POS`: Define master-slave mappings
  - `POS` is `left` or `right` (determines arm position)
  - Mirror mode auto-enabled when positions differ (e.g., `left` master → `right` slave)
- `-g, --gravity`: Enable gravity compensation for master arms
- `-v, --velocity SPEED`: Velocity for slave motors (default: 1.0)
