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
