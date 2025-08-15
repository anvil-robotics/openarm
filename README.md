# OpenArm

A Python package for robotic arm control and automation.

```bash
pip install -e .
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
- **`sim`**: MuJoCo-based simulation
- **`ik`**: Inverse kinematics solvers (starting with IKFast)
- **`gravity_comp`**: Gravity compensation controllers (starting with MuJoCo KDL)

## Additional Tools

### `netcan`
Network CAN bridge exposing CAN bus over TCP/WebSocket for remote device communication.
