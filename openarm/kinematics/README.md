# OpenArm Kinematics

Forward/inverse kinematics and gravity compensation for robotic arms.

## Quick Start

```python
from openarm.kinematics import ForwardKinematics, InverseKinematics, GravityCompensator

# Load robot model
fk = ForwardKinematics.from_urdf("openarm_6dof.urdf")
ik = InverseKinematics.from_urdf("openarm_6dof.urdf")
gravity = GravityCompensator.from_urdf("openarm_6dof.urdf")

# Forward kinematics
pose = fk.compute([0, 0.5, 0, -1.5, 0, 0])
print(f"End-effector pose: {pose}")

# Inverse kinematics
joint_angles = ik.solve(target_pose)

# Gravity compensation
joint_positions = [0, 0.5, 0, -1.5, 0, 0]
gravity_torques = gravity.compute_torques(joint_positions)
```
