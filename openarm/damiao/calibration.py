"""Hand-eye calibration data collection.

Moves both slave arms through a series of waypoints (loaded from a file),
captures images from two ROS 2 camera topics, and records arm_T_ee poses
(FK from encoder readings) at each waypoint.

Waypoint file format (plain text, one waypoint per line):
  # Lines starting with '#' are comments.
  # 16 values = unique angles per arm (left_j0..left_j7  right_j0..right_j7)
  # 8 values  = same angles sent to both arms
  0.0 0.0 0.0 1.5708 0.0 0.0 0.0 0.0
  0.1 -0.2 0.0 1.5708 0.1 0.0 0.0 0.0  0.1 -0.2 0.0 1.5708 -0.1 0.0 0.0 0.0

Output (YAML):
  calibration.yaml
  left/images/0000.png ...
  right/images/0000.png ...
"""

from __future__ import annotations

import asyncio
import sys
import threading
import time
from pathlib import Path
from typing import TYPE_CHECKING

import cv2
import numpy as np
import yaml

from .config import FRAME_GAP, JOINT_GAINS, MOTOR_CONFIGS
from .encoding import (
    MitControlParams,
    decode_motor_state_sync,
    encode_control_mit,
)
from .transform_utils import T_inv, pose_to_T

if TYPE_CHECKING:
    from .gravity import GravityCompensator
    from .monitor import Arm

try:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from sensor_msgs.msg import Image

    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False

N_JOINTS_PER_ARM = 8


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _T_to_list(T: np.ndarray) -> list[list[float]]:
    """Convert a 4x4 numpy transform to a nested list (YAML-friendly)."""
    return [[round(float(v), 8) for v in row] for row in T]


# ---------------------------------------------------------------------------
# ROS 2 image subscriber
# ---------------------------------------------------------------------------

class _ImageListener:
    """Subscribe to two ROS 2 Image topics and keep the latest frame from each."""

    def __init__(self, left_topic: str, right_topic: str) -> None:
        self._lock = threading.Lock()
        self._left: np.ndarray | None = None
        self._right: np.ndarray | None = None
        self._node = None

        if not HAS_ROS2:
            sys.stderr.write("ROS 2 not available – image capture disabled.\n")
            return

        if not rclpy.ok():
            rclpy.init()

        self._node = rclpy.create_node("calib_image_listener")
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=3,
        )
        self._left_sub = self._node.create_subscription(
            Image, left_topic, self._left_cb, sensor_qos
        )
        self._right_sub = self._node.create_subscription(
            Image, right_topic, self._right_cb, sensor_qos
        )
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()
        sys.stdout.write(
            f"Subscribed to images: {left_topic}, {right_topic}\n"
        )

    @staticmethod
    def _image_msg_to_cv(msg: "Image") -> np.ndarray:
        """Convert a sensor_msgs/Image to a numpy BGR array."""
        h, w = msg.height, msg.width
        if msg.encoding in ("rgb8", "RGB8"):
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
            return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        if msg.encoding in ("bgr8", "BGR8"):
            return np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
        if msg.encoding in ("mono8", "8UC1"):
            gray = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w)
            return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        if msg.encoding in ("16UC1", "mono16"):
            raw = np.frombuffer(msg.data, dtype=np.uint16).reshape(h, w)
            scaled = (raw >> 8).astype(np.uint8)
            return cv2.cvtColor(scaled, cv2.COLOR_GRAY2BGR)
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, -1)
        return arr[:, :, :3]

    def _left_cb(self, msg: "Image") -> None:
        with self._lock:
            self._left = self._image_msg_to_cv(msg)

    def _right_cb(self, msg: "Image") -> None:
        with self._lock:
            self._right = self._image_msg_to_cv(msg)

    def _spin(self) -> None:
        try:
            self._executor.spin()
        except Exception:
            pass

    def grab(self) -> tuple[np.ndarray | None, np.ndarray | None]:
        """Return the latest (left_image, right_image) pair."""
        with self._lock:
            left = self._left.copy() if self._left is not None else None
            right = self._right.copy() if self._right is not None else None
        return left, right

    def wait_for_images(self, timeout: float = 15.0) -> bool:
        """Block until at least one image arrives on each topic (or timeout)."""
        if self._node is None:
            return False
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._lock:
                got_left = self._left is not None
                got_right = self._right is not None
            if got_left and got_right:
                return True
            time.sleep(0.1)
        with self._lock:
            return self._left is not None and self._right is not None

    def shutdown(self) -> None:
        if hasattr(self, "_executor") and self._executor is not None:
            self._executor.shutdown()
        if self._node is not None:
            self._node.destroy_node()


# ---------------------------------------------------------------------------
# Waypoint file I/O
# ---------------------------------------------------------------------------

def load_waypoints(
    path: str,
) -> list[dict[str, list[float]]]:
    """Load waypoints from a text file.

    Each line is either 8 floats (same for both arms) or 16 floats
    (first 8 = left, last 8 = right).  Returns a list of dicts
    ``{"left": [...], "right": [...]}``.
    """
    waypoints: list[dict[str, list[float]]] = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            vals = [float(v) for v in line.split()]
            if len(vals) == 2 * N_JOINTS_PER_ARM:
                waypoints.append({
                    "left": vals[:N_JOINTS_PER_ARM],
                    "right": vals[N_JOINTS_PER_ARM:],
                })
            elif len(vals) == N_JOINTS_PER_ARM:
                waypoints.append({"left": vals, "right": vals})
            else:
                sys.stderr.write(
                    f"WARNING: skipping line with {len(vals)} values "
                    f"(expected {N_JOINTS_PER_ARM} or {2 * N_JOINTS_PER_ARM})\n"
                )
    return waypoints


# ---------------------------------------------------------------------------
# Core calibration routine
# ---------------------------------------------------------------------------

async def run_calibration(  # noqa: PLR0912, C901
    arms: list["Arm"],
    waypoint_file: str,
    output_dir: str,
    gravity_comp: "GravityCompensator",
    slave_gravity_comp: "GravityCompensator | None" = None,
    left_image_topic: str = "/camera_left/infra1/image_rect_raw",
    right_image_topic: str = "/camera_right/infra1/image_rect_raw",
    settle_ms: int = 1500,
    move_hz: float = 300.0,
    move_dur: float = 0.5,
) -> None:
    """Execute calibration data collection.

    For each waypoint both slave arms are moved simultaneously, then we
    pause, capture images, compute arm_T_ee via FK, and save everything
    to a single YAML file.
    """
    # -- Load waypoints ----------------------------------------------------
    waypoints = load_waypoints(waypoint_file)
    if not waypoints:
        sys.stderr.write(f"No waypoints found in {waypoint_file}\n")
        return
    sys.stdout.write(f"Loaded {len(waypoints)} calibration waypoints from {waypoint_file}\n")

    # -- Identify slave arms -----------------------------------------------
    slaves: dict[str, "Arm"] = {}
    for arm in arms:
        if arm.is_slave and arm.position in ("left", "right"):
            slaves[arm.position] = arm
    if not slaves:
        sys.stderr.write("No slave arms configured – cannot run calibration.\n")
        return
    sys.stdout.write(f"Slave arms: {', '.join(slaves.keys())}\n")

    # -- Prepare output directories ----------------------------------------
    out = Path(output_dir)
    side_dirs: dict[str, Path] = {}
    for side in slaves:
        img_dir = out / side / "images"
        img_dir.mkdir(parents=True, exist_ok=True)
        side_dirs[side] = out / side
    sys.stdout.write(f"Output directory: {out}\n")

    # -- Compute fixed body_T_arm transforms (arm base at zero config) -----
    body_T_arm: dict[str, np.ndarray] = {}
    arm_T_body: dict[str, np.ndarray] = {}
    for side in slaves:
        base_body = f"openarm_{side}_link0"
        zero_q = np.zeros(7)
        base_pos, base_quat = gravity_comp.kdl.compute_body_pose(zero_q, base_body, side=side)
        T = pose_to_T(base_pos, base_quat)
        body_T_arm[side] = T
        arm_T_body[side] = T_inv(T)
    sys.stdout.write("Computed body_T_arm transforms\n")

    # -- Start image listener ----------------------------------------------
    img_listener = _ImageListener(left_image_topic, right_image_topic)
    sys.stdout.write("Waiting for camera images (up to 15s) ...")
    sys.stdout.flush()
    if img_listener.wait_for_images(timeout=15.0):
        sys.stdout.write(" OK\n")
    else:
        left, right = img_listener.grab()
        missing = []
        if left is None:
            missing.append(f"left ({left_image_topic})")
        if right is None:
            missing.append(f"right ({right_image_topic})")
        sys.stdout.write(f" WARNING: no images from: {', '.join(missing)}\n")

    # -- Gravity comp helper -----------------------------------------------
    _gc = slave_gravity_comp or gravity_comp

    # -- Iterate waypoints -------------------------------------------------
    yaml_waypoints: list[dict] = []

    try:
        for wp_idx, wp in enumerate(waypoints):
            sys.stdout.write(f"\n--- Waypoint {wp_idx + 1}/{len(waypoints)} ---\n")

            # ---- Move all slaves to this waypoint ----------------------------
            arm_start_q: dict[str, list[float]] = {}
            for side, slave in slaves.items():
                arm_start_q[side] = slave.get_positions()

            move_steps = int(move_dur * move_hz)
            dt = 1.0 / move_hz

            for step in range(move_steps + 1):
                alpha = step / move_steps if move_steps > 0 else 1.0
                alpha = 3 * alpha**2 - 2 * alpha**3

                for side, slave in slaves.items():
                    wp_q = wp.get(side, wp.get("left", []))
                    sq = arm_start_q[side]
                    n_joints = min(len(wp_q), len(MOTOR_CONFIGS))

                    grav_torques: list[float] = []
                    if _gc:
                        grav_torques = _gc.compute(slave.get_positions(), position=side)

                    for idx, motor in enumerate(slave.motors):
                        if motor is None or idx >= n_joints:
                            continue
                        target = wp_q[idx]
                        pos = sq[idx] + alpha * (target - sq[idx])
                        if slave.mirror_mode and MOTOR_CONFIGS[idx].inverted:
                            pos = -pos
                        kp, kd = JOINT_GAINS[idx] if idx < len(JOINT_GAINS) else (2.0, 1.0)
                        torque = grav_torques[idx] if idx < len(grav_torques) else 0.0
                        params = MitControlParams(q=pos, dq=0, kp=kp, kd=kd, tau=torque)
                        try:
                            encode_control_mit(motor._bus, motor._slave_id, motor._motor_limits, params)
                            time.sleep(FRAME_GAP)
                            state = decode_motor_state_sync(motor._bus, motor._master_id, motor._motor_limits)
                            if state is not None:
                                slave.states[idx] = state
                        except Exception:
                            pass

                if step % 100 == 0 or step == move_steps:
                    pct = 100.0 * step / move_steps if move_steps else 100
                    sys.stdout.write(f"\r  moving {pct:.0f}%")
                    sys.stdout.flush()

                await asyncio.sleep(dt)

            sys.stdout.write("\r  moving 100% — settling...")
            sys.stdout.flush()

            # ---- Settle ----------------------------------------------------
            await asyncio.sleep(settle_ms / 1000.0)

            # ---- Capture images -------------------------------------------
            left_img, right_img = img_listener.grab()
            side_to_img: dict[str, np.ndarray | None] = {
                "left": left_img, "right": right_img,
            }
            wp_entry: dict = {"index": wp_idx}
            for side in slaves:
                img = side_to_img.get(side)
                rel_path = f"{side}/images/{wp_idx:04d}.png"
                if img is not None:
                    cv2.imwrite(str(out / rel_path), img)
                    wp_entry[f"{side}_image"] = rel_path
                else:
                    sys.stdout.write(f"\n  WARNING: no {side} image at waypoint {wp_idx}\n")
                    wp_entry[f"{side}_image"] = None

            # ---- FK → arm_T_ee ------------------------------------------
            for side, slave in slaves.items():
                q_enc = slave.get_positions()[:7]
                ee_pos, ee_quat = gravity_comp.forward_kinematics(q_enc, position=side)
                body_T_ee = pose_to_T(ee_pos, ee_quat)
                arm_T_ee = arm_T_body[side] @ body_T_ee
                wp_entry[f"{side}_arm_T_ee"] = _T_to_list(arm_T_ee)
                wp_entry[f"{side}_joint_angles"] = [round(float(v), 6) for v in q_enc]
                sys.stdout.write(
                    f"\n  {side} EE: [{ee_pos[0]:.4f}, {ee_pos[1]:.4f}, {ee_pos[2]:.4f}]"
                )

            yaml_waypoints.append(wp_entry)
            sys.stdout.write("\n")

    except KeyboardInterrupt:
        sys.stdout.write("\nCalibration interrupted.\n")

    finally:
        # ---- Write YAML ---------------------------------------------------
        yaml_data: dict = {
            "body_T_left_arm": _T_to_list(body_T_arm["left"]) if "left" in body_T_arm else None,
            "body_T_right_arm": _T_to_list(body_T_arm["right"]) if "right" in body_T_arm else None,
            "waypoints": yaml_waypoints,
        }
        yaml_path = out / "calibration.yaml"
        yaml_path.parent.mkdir(parents=True, exist_ok=True)
        with open(yaml_path, "w") as f:
            yaml.dump(yaml_data, f, default_flow_style=None, sort_keys=False)
        sys.stdout.write(f"Saved calibration data → {yaml_path}\n")

        img_listener.shutdown()
        sys.stdout.write("Calibration data collection complete.\n")
