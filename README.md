# OpenArm

A Python package for robotic arm control and automation.

```bash
pip install -e .
```

# Planned Architecture

Simple modular architecture focused on getting basic arm control working first.

```
openarm/
├── transport/        # CAN bus communication
├── motors/           # Motor control (Damiao)
├── control/          # High-level arm control
├── kinematics/       # Forward/inverse kinematics & gravity compensation
└── simulation/       # Physics simulation
```

## Core Modules

### `transport`
CAN bus communication layer. Handles SocketCAN, virtual CAN, and network bridges.

### `motors`
Motor control starting with Damiao servos. Extensible to other motor types.

### `control`
High-level arm control. Coordinates multiple motors as unified arm.

### `kinematics`
Forward/inverse kinematics and gravity compensation. Robot pose calculations and dynamics.

### `simulation`
Physics simulation using MuJoCo for testing and development.
