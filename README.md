# OpenArm

A Python package for robotic arm control and automation.

## Installation

```bash
pip install -e .
```

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

# Planned Architecture

## Project Structure

Single Python package with submodules organized as subdirectories.

## Core Modules

### `bus`

CAN bus abstraction layer enabling asynchronous read operations while sending CAN frames.

### `damiao`

Damiao motor communication and control:

- CAN message protocols
- CLI tools for motor management (register read/write, ID changes, testing)

### `openarm`

High-level arm control abstraction:

- Joint position control across all motors
- Gripper control
- Whole-arm commands (e.g., zero position setting)
- CLI tools for arm-level operations

#### Submodules

- **`ros`**: ROS 2 wrapper exposing OpenArm functionality
- **`description`**: Robot URDF files
- **`simulation`**: MuJoCo-based simulation with complete robot model and interactive environment
- **`ik`**: Inverse kinematics solvers (starting with IKFast)
- **`gravity_comp`**: Gravity compensation controllers (starting with MuJoCo KDL)

## Additional Tools

### `netcan`

Network CAN bridge exposing CAN bus over TCP/WebSocket for remote device communication.
