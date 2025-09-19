import asyncio
import websockets
import json
import argparse
import sys
import os
import math
import threading
from datetime import datetime

import mujoco.viewer
import numpy as np

from openarm.simulation import OpenArmSimulation
from openarm_ikfast import compute_ik

def rotation_matrix_from_euler(roll, pitch, yaw):
    """Create rotation matrix from roll, pitch, yaw angles (in radians)"""
    R_x = np.array([
        [1, 0, 0],
        [0, math.cos(roll), -math.sin(roll)],
        [0, math.sin(roll), math.cos(roll)]
    ])

    R_y = np.array([
        [math.cos(pitch), 0, math.sin(pitch)],
        [0, 1, 0],
        [-math.sin(pitch), 0, math.cos(pitch)]
    ])

    R_z = np.array([
        [math.cos(yaw), -math.sin(yaw), 0],
        [math.sin(yaw), math.cos(yaw), 0],
        [0, 0, 1]
    ])

    return R_z @ R_y @ R_x

def quaternion_to_rotation_matrix(quat):
    """Convert quaternion [x, y, z, w] to 3x3 rotation matrix"""
    x, y, z, w = quat
    return np.array([
        [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
        [2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
        [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]
    ])

def rotation_matrix_to_euler(R):
    """Convert rotation matrix to roll, pitch, yaw angles"""
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return x, y, z

class VRTracker:
    def __init__(self):
        self.latest_data = {
            "head": None,
            "left": None,
            "right": None
        }
        self.data_lock = threading.Lock()

    def update_data(self, data):
        with self.data_lock:
            for key in ["head", "left", "right"]:
                if key in data:
                    self.latest_data[key] = data[key]

    def get_head_data(self):
        with self.data_lock:
            return self.latest_data["head"]

    def get_left_data(self):
        with self.data_lock:
            return self.latest_data["left"]

async def listen_websocket(url, tracker):
    try:
        async with websockets.connect(url) as websocket:
            print(f"Connected to {url}")
            async for message in websocket:
                try:
                    data = json.loads(message)
                    tracker.update_data(data)
                except json.JSONDecodeError:
                    print(f"Invalid JSON received: {message}")
    except Exception as e:
        print(f"Connection error: {e}")

def run_websocket_in_thread(url, tracker):
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(listen_websocket(url, tracker))

def main():
    parser = argparse.ArgumentParser(description="VR tracking simulation demo - connects to WebSocket stream")
    parser.add_argument("url", help="WebSocket URL to connect to")

    args = parser.parse_args()

    tracker = VRTracker()

    print(f"Starting VR tracking simulation...")
    print(f"Connecting to: {args.url}")

    sim = OpenArmSimulation()
    sim.model.opt.gravity[:] = [0, 0, 0]

    websocket_thread = threading.Thread(target=run_websocket_in_thread, args=(args.url, tracker))
    websocket_thread.daemon = True
    websocket_thread.start()

    with mujoco.viewer.launch_passive(sim.model, sim.data) as viewer:
        while viewer.is_running():
            head_data = tracker.get_head_data()
            left_data = tracker.get_left_data()

            if head_data and left_data:
                try:
                    left_pos = np.array([
                        head_data["pos"][2] - left_data["pos"][2],
                        head_data["pos"][1] - left_data["pos"][1],
                        head_data["pos"][0] - left_data["pos"][0]
                    ])

                    left_quat = left_data["rot"]
                    vr_rotation = quaternion_to_rotation_matrix([
                        -left_quat[3], -left_quat[2], -left_quat[1], -left_quat[0]
                    ])

                    vr_roll, vr_pitch, vr_yaw = rotation_matrix_to_euler(vr_rotation)

                    final_roll = vr_pitch + math.pi / 2
                    final_pitch = vr_yaw
                    final_yaw = vr_roll - math.pi / 4

                    left_rotation = rotation_matrix_from_euler(final_roll, final_pitch, final_yaw)

                    current_left = sim.get_left_arm_positions()

                    solution = compute_ik(left_pos, left_rotation, current_left)
                    if solution is not None:
                        sim.set_left_arm_positions(solution + [0.0])

                except Exception as e:
                    pass

            sim.step()
            viewer.sync()

if __name__ == "__main__":
    main()
