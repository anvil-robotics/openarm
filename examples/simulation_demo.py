"""OpenArm simulation demonstration script.

This example demonstrates basic usage of the OpenArm simulation environment
by creating a simple oscillating motion on the first joint of both arms
using position control while visualizing the result in MuJoCo's viewer.
"""

import math

import mujoco.viewer

from openarm.simulation import OpenArmSimulation

# Create simulation instance with default OpenArm model
sim = OpenArmSimulation()

with mujoco.viewer.launch_passive(sim.model, sim.data) as viewer:
    step_count = 0
    while viewer.is_running():
        # Generate sinusoidal motion for demonstration
        amplitude = 0.5  # Joint angle amplitude in radians
        frequency = 0.01  # Oscillation frequency (cycles per step)
        first_joint_angle = amplitude * math.sin(step_count * frequency)

        # Read current joint positions for both arms
        left_positions = sim.get_left_arm_positions()
        right_positions = sim.get_right_arm_positions()

        # Apply sinusoidal motion to the first joint of each arm
        left_positions[0] = first_joint_angle
        right_positions[0] = first_joint_angle

        # Set target positions for both arms
        sim.set_left_arm_positions(left_positions)
        sim.set_right_arm_positions(right_positions)

        # Advance physics simulation and update viewer
        sim.step()
        viewer.sync()
        step_count += 1
