# OpenArm Simulation

Physics simulation using MuJoCo for testing and development without hardware.

## Features

- Complete OpenArm robot model with dual 7-DOF arms and grippers
- High-fidelity visual and collision meshes
- MuJoCo XML model definition (`models/openarm.xml`)
- Interactive simulation environment with position and torque control
- Python API for programmatic control and data collection

## Quick Start

```python
from openarm.simulation import OpenArmSimulation

# Create simulation instance
sim = OpenArmSimulation()

# Get current joint positions
left_positions = sim.get_left_arm_positions()
right_positions = sim.get_right_arm_positions()

# Apply position control
sim.set_left_arm_position_control([0.0, 0.5, 0.0, -1.0, 0.0, 0.5, 0.0])
sim.set_right_arm_position_control([0.0, -0.5, 0.0, 1.0, 0.0, -0.5, 0.0])

# Step simulation
sim.step()
```

## Models

Robot models are stored in the `models/` directory:

- `openarm.xml` - Main MuJoCo model definition
- `meshes/visual/` - High-quality visual meshes (.obj files)
- `meshes/collision/` - Simplified collision meshes (.stl files)

The model includes:

- Dual 7-DOF robotic arms (links 0-7 each)
- Body base link
- Two grippers with finger articulation
