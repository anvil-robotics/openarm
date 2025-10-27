"""OpenArm simulation inverse kinematics demonstration script.

This example demonstrates inverse kinematics functionality in simulation
by moving the left arm in circular motion.
"""

import math
import sys
import os

import mujoco.viewer
import numpy as np

from openarm.simulation import OpenArmSimulation
from openarm_ikfast import compute_ik

def rotation_matrix_from_euler(roll, pitch, yaw):
    """Create rotation matrix from roll, pitch, yaw angles (in radians)"""
    # Roll (rotation around x-axis)
    R_x = np.array([
        [1, 0, 0],
        [0, math.cos(roll), -math.sin(roll)],
        [0, math.sin(roll), math.cos(roll)]
    ])

    # Pitch (rotation around y-axis)
    R_y = np.array([
        [math.cos(pitch), 0, math.sin(pitch)],
        [0, 1, 0],
        [-math.sin(pitch), 0, math.cos(pitch)]
    ])

    # Yaw (rotation around z-axis)
    R_z = np.array([
        [math.cos(yaw), -math.sin(yaw), 0],
        [math.sin(yaw), math.cos(yaw), 0],
        [0, 0, 1]
    ])

    # Combined rotation matrix (R = R_z * R_y * R_x)
    return R_z @ R_y @ R_x

# Create simulation instance with default OpenArm model
sim = OpenArmSimulation()
sim.model.opt.gravity[:] = [0, 0, 0]


with mujoco.viewer.launch_passive(sim.model, sim.data) as viewer:
    t = 0.0
    while viewer.is_running():
        # Create circular motion in x-axis for left arm
        radius = 0.1
        center_x = 0.2
        center_y = 0.1
        center_z = 0.1

        left_target = np.array([
            center_x,  # X: circular motion
            center_y + radius * math.cos(t),                         # Y: left side (constant)
            center_z + radius * math.sin(t)   # Z: circular motion
        ])

        t += 0.02  # Increment time for animation

        # Target orientation using roll, pitch, yaw (in radians)
        roll = math.pi / 2 # 90 degrees
        pitch = 0
        yaw = math.pi / 2  # 90 degrees

        forward_orientation = rotation_matrix_from_euler(roll, pitch, yaw)

        try:
            # Get current joint positions as initial guess
            current_left = sim.get_left_arm_positions()

            # Call ikfast Python binding directly
            solution = compute_ik(left_target, forward_orientation, current_left)
            if solution is not None:
                sim.set_left_arm_positions(solution + [0.0])

        except Exception as e:
            print(f"IK solution failed: {e}")
            # Keep current positions if IK fails
            pass

        # Advance physics simulation and update viewer
        sim.step()
        viewer.sync()
