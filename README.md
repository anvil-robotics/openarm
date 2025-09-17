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

### Motor Monitoring

Monitor motor angles in real-time:

```bash
python -m openarm.damiao.monitor
```

### Teleoperation

Enable teleoperation mode where one arm (master) controls other arms (slaves):

```bash
# Basic teleoperation (first bus is master, others are slaves)
python -m openarm.damiao.monitor --teleop

# Custom master-slave configuration with mirror mode
python -m openarm.damiao.monitor -t --follow can0:left:can1:right

# Multiple master-slave pairs
python -m openarm.damiao.monitor -t --follow can1:left:can2:right --follow can0:right:can3:left

# With gravity compensation and custom velocity
python -m openarm.damiao.monitor -t --follow can1:left:can2:right --gravity --velocity 5
```

Options:

- `-t, --teleop`: Enable teleoperation mode
- `--follow MASTER:POS:SLAVE:POS`: Define master-slave mappings (POS is 'left' or 'right')
  - Mirror mode is automatic when positions differ
- `-g, --gravity`: Enable gravity compensation for master arms
- `-v, --velocity`: Set velocity for slave motors (default: 1.0)

### Gravity Compensation

Run gravity compensation on specific arms:

```bash
# Single arm
python -m openarm.damiao.gravity --port can0:left

# Multiple arms
python -m openarm.damiao.gravity --port can0:left --port can1:right
```
