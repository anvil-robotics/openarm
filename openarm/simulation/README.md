# OpenArm Simulation

Physics simulation using MuJoCo for testing and development without hardware.

## Quick Start

```python
from openarm.simulation import Simulator

# Load simulation
sim = Simulator("openarm_7dof.xml")

# Create virtual arm
arm = sim.create_arm()

# Control in simulation
arm.move_to([0, 0.5, 0, -1.5, 0, 0])

# Run simulation
sim.run(duration=10.0)
```

## Models

Robot models will be stored in `models/` directory:
- `openarm_7dof.xml` - 7-DOF arm model
- `workspace.xml` - Table and objects
