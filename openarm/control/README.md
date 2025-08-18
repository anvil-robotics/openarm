# OpenArm Control

High-level arm control that coordinates multiple motors as a unified robotic arm.

## Quick Start

```python
from openarm.control import Arm
from openarm.transport import CANBus

# Create arm with 6 motors
bus = CANBus('can0')
arm = Arm(bus, motor_ids=[1, 2, 3, 4, 5, 6])

# Enable and home
arm.enable()
arm.home()

# Move to position
arm.move_to([0, 0.5, 0, -1.5, 0, 0])  # Joint angles in radians

# Get state
state = arm.get_state()
print(f"Position: {state.joint_positions}")
```

## Features

- [x] Multi-motor coordination
- [x] Joint space control
- [ ] Cartesian space control
- [ ] Trajectory execution
- [ ] Safety limits
- [ ] Collision detection

## Configuration

```python
# Custom arm configuration
arm = Arm(
    bus,
    motors=[
        (1, "DM8009"),   # Base
        (2, "DM4340"),   # Shoulder
        (3, "DM4310"),   # Elbow
        # ...
    ]
)
```

## Examples

See `examples/control/` for usage examples.