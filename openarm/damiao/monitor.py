"""Damiao motor monitor and teleoperation controller.

Supports motor monitoring, master-slave teleoperation with gravity compensation,
trajectory recording/playback, and FoundationPose-guided waypoint execution.
"""

from __future__ import annotations

import argparse
import asyncio
import logging
import re
import sys
import threading
import time
from dataclasses import dataclass, field

import can
import cv2
import mujoco
import mujoco.viewer
import numpy as np
from scipy.spatial.transform import Rotation as R_scipy

from openarm.bus import Bus

from .button_reader import ButtonReader
from .calibration import run_calibration
from .config import BTN_REMAP, FRAME_GAP, JOINT_GAINS, MOTOR_CONFIGS
from .detect import detect_motors
from .encoding import (
    ControlMode,
    MitControlParams,
    decode_motor_state_sync,
    encode_control_mit,
)
from .gravity import GravityCompensator
from .motor import Motor
from .recording import load_recording, run_playback, save_recording
from .trajectory import execute_waypoint, home_all_arms

# Platform-specific imports for keyboard input
try:
    import select
    import termios
    import tty

    HAS_TERMIOS = True
except ImportError:
    HAS_TERMIOS = False

try:
    import msvcrt

    HAS_MSVCRT = True
except ImportError:
    HAS_MSVCRT = False

try:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import JointState
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False

# ANSI color codes for terminal output
RED = "\033[91m"
GREEN = "\033[92m"
RESET = "\033[0m"

# Constants
FOLLOW_SPEC_PARTS = 4  # MASTER:POSITION:SLAVE:POSITION

# Set up logging
logger = logging.getLogger(__name__)


from openarm.damiao.transform_utils import (
    pose_to_T as _pose_to_T,
    T_inv as _T_inv,
    ros_pose_to_T as _ros_pose_to_T,
    T_to_pos_quat as _T_to_pos_quat,
    quat_to_rpy_deg as _quat_to_rpy_deg,
    quat_from_rpy_deg as _quat_from_rpy_deg,
    quat_multiply as _quat_multiply,
)


def load_cam_extrinsics(path: str) -> dict[str, np.ndarray]:
    """Load calibrated ee_T_cam transforms from a YAML file.

    Returns dict with 'left' and 'right' keys mapping to 4x4 numpy arrays.
    """
    import yaml
    with open(path) as f:
        data = yaml.safe_load(f)
    result = {}
    for side in ("left", "right"):
        T = np.array(data[side]["ee_T_cam"], dtype=np.float64)
        result[side] = T
        method = data[side].get("method", "?")
        std = data[side].get("board_origin_std_m", 0)
        sys.stdout.write(
            f"  Loaded {side} ee_T_cam ({method}, std={std*1000:.1f}mm):\n"
            f"    t = {T[:3,3]}\n"
        )
    return result


def check_keyboard_input() -> str | None:
    """Check if a key has been pressed (non-blocking)."""
    if HAS_MSVCRT and msvcrt.kbhit():
        return msvcrt.getch().decode("utf-8", errors="ignore").lower()
    if HAS_TERMIOS and select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1).lower()
    return None


@dataclass
class Arm:
    """Represents a single robotic arm with its motors and configuration."""

    position: str  # "left" or "right"
    can_bus: can.BusABC  # The CAN bus for this arm
    channel: str  # Channel name (e.g., "can0", "can1")
    motors: list[Motor | None] = field(default_factory=list)
    states: list = field(default_factory=list)  # Current states for each motor
    is_master: bool = False  # Whether this arm is a master
    is_slave: bool = False  # Whether this arm is a slave
    mirror_mode: bool = False  # Whether mirror mode is enabled (for slaves)
    follows: str | None = None  # Channel name of master (for slaves)

    @property
    def active_motors(self) -> list[Motor]:
        """Get list of active (non-None) motors."""
        return [m for m in self.motors if m is not None]

    @property
    def active_count(self) -> int:
        """Count of active motors."""
        return len(self.active_motors)

    def get_positions(self) -> list[float]:
        """Return current joint positions, 0.0 for missing motors/states."""
        return [
            st.position if m is not None and st is not None else 0.0
            for m, st in zip(self.motors, self.states)
        ]

    async def disable_all_motors(self) -> None:
        """Safely disable all active motors."""
        for motor in self.motors:
            if motor is not None:
                try:
                    await motor.disable()
                except Exception as e:  # noqa: BLE001
                    logger.debug("Failed to disable motor: %s", e)

    async def enable_all_motors(self, control_mode: ControlMode) -> None:
        """Enable all active motors with specified control mode."""
        for idx, motor in enumerate(self.motors):
            if motor is not None:
                try:
                    await motor.enable()
                    await motor.set_control_mode(control_mode)
                    logger.info("Motor %d: Enabled", idx + 1)
                    sys.stdout.write(f"    Motor {idx + 1}: Enabled\n")
                except Exception as e:
                    logger.exception("Motor %d: Error", idx + 1)
                    sys.stderr.write(f"{RED}    Motor {idx + 1}: Error - {e}{RESET}\n")

    async def refresh_states(self) -> None:
        """Refresh states for all motors."""
        new_states = []
        for motor in self.motors:
            if motor:
                try:
                    state = await motor.refresh_status()
                    new_states.append(state)
                except Exception as e:  # noqa: BLE001
                    logger.debug("Failed to refresh motor status: %s", e)
                    new_states.append(None)
            else:
                new_states.append(None)
        self.states = new_states


async def main(args: argparse.Namespace) -> None:
    """Run the monitor with the provided arguments."""
    # Create CAN buses
    try:
        if hasattr(can, "detect_available_configs"):
            configs = can.detect_available_configs("socketcan")
        else:
            import subprocess
            result = subprocess.run(
                ["/opt/iproute2-root/bin/ip", "link", "show"],
                capture_output=True, text=True,
            )
            configs = []
            lines = result.stdout.splitlines()
            for i, line in enumerate(lines):
                if i + 1 < len(lines) and "link/can" in lines[i + 1]:
                    iface = line.split(":")[1].strip().split("@")[0]
                    configs.append({"channel": iface, "interface": "socketcan"})
        print(f"detect_available_configs: {configs}")
        can_buses = [
            can.Bus(channel=config["channel"], interface=config["interface"])
            for config in configs
        ]
    except Exception as e:  # noqa: BLE001
        print(f"Exception: {e}")
        can_buses = []

    if not can_buses:
        return None

    sys.stdout.write(f"\nDetected {len(can_buses)} CAN bus(es)\n")

    try:
        return await _main(args, can_buses)
    finally:
        for bus in can_buses:
            bus.shutdown()


async def _main(args: argparse.Namespace, can_buses: list[can.BusABC]) -> None:  # noqa: C901, PLR0912
    # Detect motors on each bus
    all_bus_motors = []
    has_missing_motor = False

    print(f"can_buses: {can_buses}")

    for bus_idx, can_bus in enumerate(can_buses):
        sys.stdout.write(f"\nScanning for motors on bus {bus_idx + 1}...\n")
        slave_ids = [config.slave_id for config in MOTOR_CONFIGS]

        # Detect motors using raw CAN bus
        detected = list(detect_motors(can_bus, slave_ids, timeout=0.01))

        sys.stdout.write(f"\nBus {bus_idx + 1} Motor Status:\n")

        # Create lookup for detected motors by slave ID
        detected_lookup = {info.slave_id: info for info in detected}

        # Check all expected motors and their status
        bus_motors = []
        for config in MOTOR_CONFIGS:
            if config.slave_id not in detected_lookup:
                # Motor is not detected
                sys.stderr.write(
                    f"  {RED}✗{RESET} {config.name}: ID 0x{config.slave_id:02X} "
                    f"(Master: 0x{config.master_id:02X}) {RED}[NOT DETECTED]{RESET}\n"
                )
                bus_motors.append(None)
                has_missing_motor = True
            elif detected_lookup[config.slave_id].master_id != config.master_id:
                # Motor is detected but master ID doesn't match
                detected_info = detected_lookup[config.slave_id]
                sys.stderr.write(
                    f"  {RED}✗{RESET} {config.name}: ID 0x{config.slave_id:02X} "
                    f"{RED}[MASTER ID MISMATCH: Expected 0x{config.master_id:02X}, "
                    f"Got 0x{detected_info.master_id:02X}]{RESET}\n"
                )
                bus_motors.append(None)
                has_missing_motor = True
            else:
                # Motor is connected and configured correctly
                sys.stdout.write(
                    f"  {GREEN}✓{RESET} {config.name}: ID 0x{config.slave_id:02X} "
                    f"(Master: 0x{config.master_id:02X})\n"
                )
                # Create motor instance
                bus = Bus(can_bus)
                motor = Motor(
                    bus,
                    slave_id=config.slave_id,
                    master_id=config.master_id,
                    motor_type=config.type,
                )
                bus_motors.append(motor)

        all_bus_motors.append(bus_motors)

    # Exit if any motor is missing
    if has_missing_motor:
        sys.stderr.write(
            f"\n{RED}Error: Not all motors are detected or configured "
            f"correctly. Exiting.{RESET}\n"
        )
        return

    # Count total detected motors
    total_motors = sum(
        1 for bus_motors in all_bus_motors for m in bus_motors if m is not None
    )
    if total_motors == 0:
        sys.stderr.write(f"\n{RED}Error: No motors detected on any bus.{RESET}\n")
        return

    sys.stdout.write(
        f"\n{GREEN}Total {total_motors} motors detected across "
        f"{len(can_buses)} bus(es){RESET}\n"
    )

    # Disable all motors on all buses
    sys.stdout.write("\nDisabling all motors...\n")
    all_state_results = []
    for bus_idx, bus_motors in enumerate(all_bus_motors):
        bus_states = []
        for motor in bus_motors:
            if motor:
                try:
                    state = await motor.disable()
                    bus_states.append(state)
                except Exception as e:
                    logger.exception("Error disabling motor on bus %d", bus_idx + 1)
                    sys.stderr.write(
                        f"{RED}Error disabling motor on bus {bus_idx + 1}: {e}{RESET}\n"
                    )
                    bus_states.append(None)
            else:
                bus_states.append(None)
        all_state_results.append(bus_states)

    # Load camera extrinsics if provided
    args.ee_T_cam = {}
    if args.cam_extrinsics:
        sys.stdout.write(f"\nLoading camera extrinsics from {args.cam_extrinsics}\n")
        args.ee_T_cam = load_cam_extrinsics(args.cam_extrinsics)

    # Call teleop or monitor based on flag
    if args.teleop:
        await teleop(can_buses, all_bus_motors, all_state_results, args)
    else:
        await monitor_motors(can_buses, all_bus_motors, all_state_results)


async def monitor_motors(  # noqa: C901, PLR0912
    can_buses: list[can.BusABC],
    all_bus_motors: list[list[Motor | None]],
    all_state_results: list[list],
) -> None:
    """Monitor motor angles continuously and display them in a table format.

    Args:
        can_buses: List of CAN bus interfaces.
        all_bus_motors: List of motor lists for each bus.
        all_state_results: Initial state results for each motor.

    """
    # Start continuous monitoring with column display
    sys.stdout.write("\nContinuously monitoring motor angles (Ctrl+C to stop):\n\n")

    # Print header with bus labels
    header = "  Motor"
    for bus_idx in range(len(can_buses)):
        header += f"        Bus {bus_idx + 1}     "
    sys.stdout.write(header + "\n")
    sys.stdout.write("  " + "-" * (len(header) - 2) + "\n")

    # Print initial lines for each motor
    for config in MOTOR_CONFIGS:
        line = f"  {config.name:<12}"
        for _ in range(len(can_buses)):
            line += "  Initializing...  "
        sys.stdout.write(line + "\n")

    # Number of motors (lines to move up)
    num_motors = len(MOTOR_CONFIGS)

    # Use disable results for first display
    all_current_states = all_state_results

    try:
        while True:

            # Small delay before refresh
            await asyncio.sleep(0.1)

            # Refresh states for all buses
            new_all_states = []
            for bus_motors in all_bus_motors:
                bus_states = []
                for motor in bus_motors:
                    if motor:
                        try:
                            state = await motor.refresh_status()
                            bus_states.append(state)
                        except Exception as e:  # noqa: BLE001
                            logger.debug("Failed to refresh motor status: %s", e)
                            bus_states.append(None)
                    else:
                        bus_states.append(None)
                new_all_states.append(bus_states)
            all_current_states = new_all_states

    except KeyboardInterrupt:
        # Move cursor below all motor lines
        sys.stdout.write(f"\033[{num_motors}B\n")
        sys.stdout.write("\nMonitoring stopped.\n")


class _FrameVis:
    """Live OpenCV visualization of coordinate frames in a dedicated thread."""

    W, H = 900, 600
    SCALE = 600.0
    OX, OY = 150, 500

    def __init__(self):
        self._lock = threading.Lock()
        self._frames: dict[str, np.ndarray] = {}
        self._running = True
        self._thread = None
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _to_px(self, pos):
        return (int(self.OX + pos[0] * self.SCALE),
                int(self.OY - pos[2] * self.SCALE))

    def _draw_frame(self, img, T, label, axis_len=0.03):
        pos = T[:3, 3]
        R = T[:3, :3]
        center = self._to_px(pos)
        colors = [(0, 0, 255), (0, 255, 0), (255, 0, 0)]  # X=red, Y=green, Z=blue
        for i, color in enumerate(colors):
            tip_px = self._to_px(pos + R[:, i] * axis_len)
            cv2.arrowedLine(img, center, tip_px, color, 2, tipLength=0.3)
        cv2.putText(img, label, (center[0] + 5, center[1] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)

    def _render(self):
        img = np.zeros((self.H, self.W, 3), dtype=np.uint8)
        cv2.putText(img, "Side (XZ)", (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (140, 140, 140), 1)

        self._draw_frame(img, np.eye(4), "W", axis_len=0.04)

        with self._lock:
            frames = dict(self._frames)

        for label, T in frames.items():
            self._draw_frame(img, T, label)

        return img

    def _loop(self):
        cv2.namedWindow("Pose Viewer", cv2.WINDOW_AUTOSIZE)
        while self._running:
            img = self._render()
            cv2.imshow("Pose Viewer", img)
            if cv2.waitKey(33) & 0xFF == 27:  # ~30fps, ESC to close
                break
        cv2.destroyWindow("Pose Viewer")

    def update(self, **named_transforms):
        with self._lock:
            for name, T in named_transforms.items():
                if T is not None:
                    self._frames[name] = T.copy()

    def close(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)


class _PoseListener:
    """Thread-safe container for the latest FoundationPose PoseStamped and plane pose."""

    def __init__(self):
        self._lock = threading.Lock()
        self._pose = None  # type: PoseStamped | None
        self._plane = None  # type: PoseStamped | None
        self._node = None
        self._thread = None
        self._joint_pubs = {}  # type: dict[str, tuple]

    def start(self):
        if not HAS_ROS2:
            logger.warning("rclpy not available — FoundationPose pose subscription disabled")
            return
        try:
            rclpy.init()
        except RuntimeError:
            pass  # already initialised

        self._node = rclpy.create_node("teleop_pose_listener")
        self._node.create_subscription(
            PoseStamped, "/foundationpose/pose", self._pose_cb, 1,
        )
        self._node.create_subscription(
            PoseStamped, "/foundationpose/plane", self._plane_cb, 1,
        )
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()
        logger.info("Subscribed to /foundationpose/pose and /foundationpose/plane")

    def _get_joint_pubs(self, arm_name):
        """Lazily create commanded/observed/error JointState publishers for an arm."""
        if arm_name not in self._joint_pubs and self._node is not None:
            cmd_pub = self._node.create_publisher(
                JointState, f"/openarm/{arm_name}/joint_commanded", 10)
            obs_pub = self._node.create_publisher(
                JointState, f"/openarm/{arm_name}/joint_observed", 10)
            err_pub = self._node.create_publisher(
                JointState, f"/openarm/{arm_name}/joint_error", 10)
            self._joint_pubs[arm_name] = (cmd_pub, obs_pub, err_pub)
        return self._joint_pubs.get(arm_name)

    def publish_joint_state(self, arm_name, commanded, observed, stamp=None):
        """Publish commanded, observed, and error joint angles for an arm."""
        if self._node is None:
            return
        pubs = self._get_joint_pubs(arm_name)
        if pubs is None:
            return
        cmd_pub, obs_pub, err_pub = pubs
        now = self._node.get_clock().now().to_msg()

        n = min(len(commanded), len(observed))
        joint_names = [f"joint_{i}" for i in range(n)]

        cmd_msg = JointState()
        cmd_msg.header.stamp = now
        cmd_msg.name = joint_names
        cmd_msg.position = [float(v) for v in commanded[:n]]

        obs_msg = JointState()
        obs_msg.header.stamp = now
        obs_msg.name = joint_names
        obs_msg.position = [float(v) for v in observed[:n]]

        err_msg = JointState()
        err_msg.header.stamp = now
        err_msg.name = joint_names
        err_msg.position = [float(o) - float(c) for o, c in zip(observed[:n], commanded[:n])]

        cmd_pub.publish(cmd_msg)
        obs_pub.publish(obs_msg)
        err_pub.publish(err_msg)

    def _pose_cb(self, msg):
        with self._lock:
            self._pose = msg

    def _plane_cb(self, msg):
        with self._lock:
            self._plane = msg

    def _spin(self):
        try:
            rclpy.spin(self._node)
        except Exception:
            pass

    @property
    def latest(self):
        with self._lock:
            return self._pose

    @property
    def latest_plane(self):
        with self._lock:
            return self._plane

    def shutdown(self):
        if self._node is not None:
            self._node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


class _MuJoCoViewer:
    """Threaded MuJoCo passive viewer that mirrors real arm joint angles."""

    def __init__(self, ee_T_cam: dict[str, np.ndarray] | None = None):
        from openarm.damiao.gravity import patch_model_camera_bodies
        from openarm.simulation.models import OPENARM_MODEL_PATH
        self._model = mujoco.MjModel.from_xml_path(str(OPENARM_MODEL_PATH))
        if ee_T_cam:
            patch_model_camera_bodies(self._model, ee_T_cam)
        self._data = mujoco.MjData(self._model)
        self._lock = threading.Lock()
        self._running = False
        self._thread = None

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        with mujoco.viewer.launch_passive(self._model, self._data) as viewer:
            while viewer.is_running() and self._running:
                with self._lock:
                    mujoco.mj_forward(self._model, self._data)
                viewer.sync()
                time.sleep(1.0 / 60.0)

    def update(self, left_q=None, right_q=None):
        """Update displayed joint angles. left_q/right_q are lists of up to 8 floats."""
        with self._lock:
            if left_q is not None:
                n = min(len(left_q), 8)
                self._data.qpos[0:n] = left_q[:n]
            if right_q is not None:
                n = min(len(right_q), 8)
                self._data.qpos[9:9 + n] = right_q[:n]

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)


async def teleop(  # noqa: C901, PLR0912
    can_buses: list[can.BusABC],
    all_bus_motors: list[list[Motor | None]],
    all_state_results: list[list],
    args: argparse.Namespace,
) -> None:
    """Teleoperation mode - masters use MIT, slaves use PosVel."""
    # Start FoundationPose pose subscriber
    pose_listener = _PoseListener()
    pose_listener.start()

    # Live frame visualizer
    frame_vis = _FrameVis()

    # Initialize gravity compensators if enabled
    # Master uses the default model; slave uses heavier end-effector masses.
    SLAVE_MASS_OVERRIDES = {
        "openarm_left_hand": 0.75,
        "openarm_right_hand": 0.30,
    }
    gravity_comp = None
    slave_gravity_comp = None
    if args.gravity:
        sys.stdout.write("Initializing gravity compensation...\n")
        gravity_comp = GravityCompensator()
        slave_gravity_comp = GravityCompensator(body_mass_overrides=SLAVE_MASS_OVERRIDES)
        if args.ee_T_cam:
            gravity_comp.kdl.patch_camera_bodies(args.ee_T_cam)
            slave_gravity_comp.kdl.patch_camera_bodies(args.ee_T_cam)
        sys.stdout.write(
            f"  master model: default masses | slave model: hand mass={SLAVE_MASS_OVERRIDES.get('openarm_left_hand')} kg\n"
        )

    # MuJoCo viewer for live arm visualization (reuses gravity comp model)
    mj_viewer = None
    if gravity_comp is not None:
        try:
            mj_viewer = _MuJoCoViewer(ee_T_cam=args.ee_T_cam or None)
            mj_viewer.start()
            sys.stdout.write("MuJoCo viewer started\n")
        except Exception as e:
            sys.stdout.write(f"MuJoCo viewer not available: {e}\n")

    # PyBullet xArm6 kinematics + visualizer
    xarm: "XArm6 | None" = None
    if args.use_xarm:
        try:
            from openarm.damiao.kinematics_xarm import XArm6
            xarm = XArm6(use_gui=True, connect_real=False)
            if xarm.setup():
                _pb_init = [0.0, -60.0, -30.0, 0.0, 0.0, -90.0]
                xarm.set_joints(_pb_init, "left")
                _pb_init = [0.0, -60.0, -30.0, 0.0, 0.0, 0.0]
                xarm.set_joints(_pb_init, "right")
                xarm.step()
                sys.stdout.write("PyBullet xArm6 visualizer started\n")
            else:
                xarm = None
                sys.stdout.write("PyBullet xArm6 setup failed\n")
        except Exception as e:
            sys.stdout.write(f"PyBullet xArm6 not available: {e}\n")
    
    

    # Create Arm objects for each bus
    arms: list[Arm] = []
    channel_to_arm = {}  # Maps channel name to Arm object

    for bus_idx, (can_bus, bus_motors, bus_states) in enumerate(
        zip(can_buses, all_bus_motors, all_state_results)
    ):
        # Get channel info from the bus
        channel_info = (
            str(can_bus.channel_info)
            if hasattr(can_bus, "channel_info")
            else str(can_bus.channel)
        )
        # Extract channel name (e.g., "can0" from various formats)
        if "channel" in channel_info:
            # For socketcan: extract from "SocketcanBus channel 'can0'"
            match = re.search(r"channel ['\"]?(\w+)", channel_info)
            channel_name = match.group(1) if match else f"bus{bus_idx}"
        else:
            # For USB devices, use the product name or bus index
            channel_name = channel_info.split()[-1] if channel_info else f"bus{bus_idx}"

        # Create Arm object
        arm = Arm(
            position="unknown",  # Will be set based on --follow arguments
            can_bus=can_bus,
            channel=channel_name,
            motors=bus_motors,
            states=bus_states,
        )
        arms.append(arm)
        channel_to_arm[channel_name] = arm
        sys.stdout.write(f"Bus {bus_idx + 1}: Channel '{channel_name}'\n")

    # Parse --follow arguments if provided
    if args.follow:
        for follow_spec in args.follow:
            try:
                parts = follow_spec.split(":")
                if len(parts) != FOLLOW_SPEC_PARTS:
                    msg = f"Invalid format: {follow_spec}"
                    raise ValueError(msg)  # noqa: TRY301

                # Parse MASTER:POSITION:SLAVE:POSITION
                master_ch, master_pos, slave_ch, slave_pos = parts

                # Validate positions
                if master_pos not in ["left", "right"]:
                    msg = f"Invalid master position: {master_pos}"
                    raise ValueError(msg)  # noqa: TRY301
                if slave_pos not in ["left", "right"]:
                    msg = f"Invalid slave position: {slave_pos}"
                    raise ValueError(msg)  # noqa: TRY301

                # Validate channels exist
                if master_ch not in channel_to_arm:
                    sys.stderr.write(
                        f"{RED}Error: Master channel '{master_ch}' not found{RESET}\n"
                    )
                    return
                if slave_ch not in channel_to_arm:
                    sys.stderr.write(
                        f"{RED}Error: Slave channel '{slave_ch}' not found{RESET}\n"
                    )
                    return

                # Get Arm objects
                master_arm = channel_to_arm[master_ch]
                slave_arm = channel_to_arm[slave_ch]

                # Check for conflicts
                if slave_arm.is_slave:
                    sys.stderr.write(
                        f"{RED}Error: Slave '{slave_ch}' already follows "
                        f"'{slave_arm.follows}'{RESET}\n"
                    )
                    return

                # Configure master arm
                master_arm.position = master_pos
                master_arm.is_master = True

                # Configure slave arm
                slave_arm.position = slave_pos
                slave_arm.is_slave = True
                slave_arm.follows = master_ch
                slave_arm.mirror_mode = (
                    master_pos != slave_pos
                )  # Auto-detect mirror mode

            except ValueError:
                sys.stderr.write(
                    f"{RED}Error: Invalid follow format '{follow_spec}'. Use "
                    f"MASTER:POSITION:SLAVE:POSITION where POSITION is "
                    f"'left' or 'right'{RESET}\n"
                )
                sys.stderr.write(
                    f"{RED}Example: --follow can0:left:can1:right (mirror) or "
                    f"--follow can0:left:can1:left (no mirror){RESET}\n"
                )
                return

        # Validate no channel is both master and slave
        for arm in arms:
            if arm.is_master and arm.is_slave:
                sys.stderr.write(
                    f"{RED}Error: Channel {arm.channel} cannot be both "
                    f"master and slave{RESET}\n"
                )
                return

    sys.stdout.write("\nMaster-Slave Configuration:\n")
    # Group slaves by master
    for master_arm in [arm for arm in arms if arm.is_master]:
        slaves = []
        for slave_arm in [
            arm for arm in arms if arm.is_slave and arm.follows == master_arm.channel
        ]:
            mirror_str = "(mirror)" if slave_arm.mirror_mode else ""
            slave_str = f"{slave_arm.channel}:{slave_arm.position}{mirror_str}"
            slaves.append(slave_str)
        if slaves:
            sys.stdout.write(
                f"  Master: {master_arm.channel}:{master_arm.position} -> "
                f"Slaves: {', '.join(slaves)}\n"
            )

    # Enable all motors (masters with MIT, slaves with PosVel)
    sys.stdout.write("\nEnabling motors for teleoperation...\n")
    for arm in arms:
        if arm.is_slave:
            sys.stdout.write(
                f"  {arm.channel}: Enabling motors with "
                f"Position-Velocity control (slave)\n"
            )
            await arm.enable_all_motors(ControlMode.MIT)
        else:  # master
            sys.stdout.write(
                f"  {arm.channel}: Enabling motors with MIT control (master)\n"
            )
            await arm.enable_all_motors(ControlMode.MIT)

    await home_all_arms(arms)

    # ---- Calibration data collection mode --------------------------------
    if args.calibrate:
        if gravity_comp is None:
            sys.stderr.write("Calibration mode requires --gravity.\n")
        else:
            try:
                await run_calibration(
                    arms,
                    waypoint_file=args.calibrate,
                    output_dir=args.calib_output,
                    gravity_comp=gravity_comp,
                    slave_gravity_comp=slave_gravity_comp
                )
            finally:
                sys.stdout.write("\nDisabling ALL motors for safety...\n")
                for arm in arms:
                    await arm.disable_all_motors()
                sys.stdout.write("All motors disabled.\n")
        return

    # Start teleoperation with monitoring display
    sys.stdout.write("\nTeleoperation mode starting...\n\n")

    # Number of motors (lines to move up)
    num_motors = len(MOTOR_CONFIGS)

    # Print stop instruction before entering raw mode
    stop_msg = "Press 'Q' to stop | X/Y/Z pos hold | W save waypoint | Btn1 waypoint | Btn2 ori lock"
    sys.stdout.write(stop_msg + "\n")

    # Impedance control state per side.
    # setpoints: side -> {0: x, 1: y, 2: z} (float values; None = needs latching)
    # lock_ori:  side -> bool  (True = lock full orientation)
    # ori_quats: side -> np.ndarray [w,x,y,z]  (latched target quaternion)
    impedance_setpoints: dict[str, dict[int, float | None]] = {
        "left": {},
        "right": {},
    }
    impedance_lock_ori: dict[str, bool] = {"left": False, "right": False}
    impedance_ori_quats: dict[str, np.ndarray | None] = {
        "left": None,
        "right": None,
    }
    _KEY_TO_AXIS = {"x": 0, "y": 1, "z": 2}
    _AXIS_LABEL = {0: "X", 1: "Y", 2: "Z"}

    # Set terminal to raw mode for keyboard detection
    old_settings = None
    raw_mode = False
    if HAS_TERMIOS:
        try:
            old_settings = termios.tcgetattr(sys.stdin)
            tty.setraw(sys.stdin.fileno())
            raw_mode = True
        except (OSError, termios.error) as e:
            # Might fail in some environments
            logger.debug("Failed to set raw mode: %s", e)

    # Helper for raw mode printing
    def raw_print(msg: str = "") -> None:
        if raw_mode:
            sys.stdout.write(msg.replace("\n", "\r\n"))
            sys.stdout.flush()
        else:
            sys.stdout.write(msg + "\n")

    # Arduino button reader (non-blocking via background thread)
    btn_reader = ButtonReader()
    _prev_buttons = [0, 0, 0, 0]

    # ── Recording / Playback state ─────────────────────────────────────
    _recording = False
    _record_buf: list[dict[str, list[float]]] = []
    _record_file = args.record
    _rec_slaves = sorted(
        [arm for arm in arms if arm.is_slave],
        key=lambda a: a.position,
    )
    _rec_sides = [a.position for a in _rec_slaves]
    _rec_n_joints = [len(a.motors) for a in _rec_slaves]

    # ── Calibration waypoint recording ────────────────────────────────
    _wp_file = args.waypoints
    _wp_buf: list[list[float]] = []  # each entry: 16 floats (8 left + 8 right)

    _playback_frames: "list[dict[str, list[float]]]" = []
    _playback_arm_map: "dict[str, Arm]" = {}

    if args.playback:
        sys.stdout.write(f"\n=== PLAYBACK MODE: {args.playback} ===\n")
        pb_sides, pb_nj, _playback_frames = load_recording(args.playback)
        if not _playback_frames:
            sys.stdout.write("Recording file is empty — nothing to play.\n")
        else:
            sys.stdout.write(
                f"Loaded {len(_playback_frames)} frames, "
                f"arms: {', '.join(f'{s}({n}j)' for s, n in zip(pb_sides, pb_nj))}\n"
            )
            side_to_slave = {a.position: a for a in arms if a.is_slave}
            for s in pb_sides:
                if s in side_to_slave:
                    _playback_arm_map[s] = side_to_slave[s]
                else:
                    sys.stdout.write(f"  WARNING: no slave arm for recorded side '{s}' — skipping\n")

            if not _playback_arm_map:
                sys.stdout.write("No matching slave arms — cannot play back.\n")

        sys.stdout.write("\nEntering main loop (press p to play, q to quit)\n")

    try:
        # Small initial delay to ensure display is ready
        await asyncio.sleep(0.1)

        # Rate-limited slave command tracking: max joint velocity enforced every tick.
        # Keyed by slave channel -> list of last commanded positions (None = uninitialised).
        _slave_cmd_pos: dict[str, list] = {}

        # xArm6 ↔ master arm offset mapping (populated once arms settle)
        # side -> (T_master_init_inv, T_pb_init)
        _xarm_offsets: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        _xarm_last_q: dict[str, np.ndarray] = {}  # side -> last IK solution (for joint-jump check)

        loop_count = 0
        last_loop_time = time.time()
        while True:
            if loop_count < 500:
                SLAVE_MAX_VEL = 0.5  # rad/s — max joint velocity for slave tracking
            else:
                SLAVE_MAX_VEL = 10.0  # rad/s — max joint velocity for slave tracking

            loop_start = time.time()
            loop_time = loop_start - last_loop_time
            last_loop_time = loop_start

            loop_count += 1
            # Check for key presses
            if raw_mode:
                key = check_keyboard_input()
                if key == "q":
                    raw_print("\nStopping teleoperation...")
                    break
                elif key == "p" and _playback_frames and _playback_arm_map:
                    raw_print(f"\n  Replaying {len(_playback_frames)} frames ...\n")
                    run_playback(
                        _playback_frames, _playback_arm_map,
                        gravity_comp=gravity_comp,
                        slave_gravity_comp=slave_gravity_comp,
                    )
                    await home_all_arms(arms, skip_first=True)
                    _slave_cmd_pos.clear()
                    raw_print(f"\n  Playback done — press p to replay, q to quit\n")
                elif key == "r" and _record_file is not None:
                    if not _recording:
                        _record_buf.clear()
                        _recording = True
                        raw_print(f"\n  Recording STARTED → {_record_file}\n")
                    else:
                        _recording = False
                        save_recording(_record_buf, _record_file, _rec_sides, _rec_n_joints)
                        raw_print(f"\n  Recording STOPPED ({len(_record_buf)} frames)\n")
                elif key == "w" and _wp_file is not None:
                    left_q = [0.0] * 8
                    right_q = [0.0] * 8
                    for arm in arms:
                        if arm.is_slave and arm.position == "left":
                            left_q = arm.get_positions()[:8]
                        elif arm.is_slave and arm.position == "right":
                            right_q = arm.get_positions()[:8]
                    _wp_buf.append(left_q + right_q)
                    raw_print(
                        f"\n  Waypoint {len(_wp_buf)} saved"
                        f"  L=[{left_q[0]:.3f},{left_q[1]:.3f},{left_q[2]:.3f},{left_q[3]:.3f},...]"
                        f"  R=[{right_q[0]:.3f},{right_q[1]:.3f},{right_q[2]:.3f},{right_q[3]:.3f},...]\n"
                    )
                elif key in _KEY_TO_AXIS:
                    axis = _KEY_TO_AXIS[key]
                    any_was_on = any(axis in sp for sp in impedance_setpoints.values())
                    for side_sp in impedance_setpoints.values():
                        if any_was_on:
                            side_sp.pop(axis, None)
                        else:
                            side_sp[axis] = None
                    state = "OFF" if any_was_on else "ON"
                    raw_print(f"\n  Impedance {_AXIS_LABEL[axis]} {state}\n")

            # Check for button presses (rising edge)
            btns_raw = btn_reader.state
            btns = [btns_raw[BTN_REMAP[i]] for i in range(4)]
            for i in range(4):
                if btns[i] and not _prev_buttons[i]:
                    arm_of_choice = "right"
                    if i == 0:
                        # Button 1: move gripper to FoundationPose target
                        fp_pose = pose_listener.latest
                        if fp_pose is None:
                            raw_print(f"\n  Button 1: no FoundationPose pose yet\n")
                        elif gravity_comp is None:
                            raw_print(f"\n  Button 1: gravity comp not enabled\n")
                        else:
                            chosen_master = None
                            chosen_slave = None
                            for arm in arms:
                                if arm.is_master and arm.position == arm_of_choice:
                                    chosen_master = arm
                                if arm.is_slave and arm.position == arm_of_choice:
                                    chosen_slave = arm

                            if chosen_slave is None:
                                raw_print(f"\n  Button 1: no {arm_of_choice} follower arm\n")
                            else:
                                cur_q = chosen_slave.get_positions()

                                camera_body = f"openarm_{arm_of_choice}_camera"

                                # T_world_cam = T_world_tcp @ T_tcp_cam
                                tcp_pos, tcp_quat = gravity_comp.forward_kinematics(
                                    cur_q[:7], position=arm_of_choice,
                                )
                                cam_pos, cam_quat = gravity_comp.kdl.compute_body_pose(
                                    np.array(cur_q[:7]), camera_body, side=arm_of_choice,
                                )
                                T_world_tcp = _pose_to_T(tcp_pos, tcp_quat)
                                T_world_cam_direct = _pose_to_T(cam_pos, cam_quat)
                                T_tcp_cam = _T_inv(T_world_tcp) @ T_world_cam_direct

                                # additional finger length
                                T_finger_length = np.eye(4)
                                T_finger_length[2, 3] = 0.03
                                T_world_tcp = T_world_tcp @ T_finger_length
                                T_world_cam = T_world_tcp @ T_tcp_cam

                                # T_camera_object from FoundationPose
                                T_cam_obj = _ros_pose_to_T(fp_pose.pose)

                                T_world_obj = T_world_cam @ T_cam_obj

                                # Define waypoints as transforms relative to the object frame
                                # (name, obj_T_wp, duration, gripper_angle)
                                obj_T_wps = []
                                wp1 = np.eye(4)
                                wp1[0, 3] = -0.02
                                obj_T_wps.append(("approach", wp1, 1.0, -0.7))

                                wp2 = np.eye(4)
                                wp2[0, 3] = -0.08
                                obj_T_wps.append(("approach", wp2, 0.25, -0.7))

                                wp3 = np.eye(4)
                                wp3[0, 3] = -0.08
                                obj_T_wps.append(("approach", wp3, 0.25, -0.1))

                                raw_print(f"\n  Button 1: move {arm_of_choice} to FPose target ({len(obj_T_wps)} waypoints)")
                                raw_print(f"    Current TCP: [{tcp_pos[0]:.3f}, {tcp_pos[1]:.3f}, {tcp_pos[2]:.3f}]")
                                raw_print(f"    Object (cam): x={fp_pose.pose.position.x:.3f} y={fp_pose.pose.position.y:.3f} z={fp_pose.pose.position.z:.3f}")

                                seed_q = cur_q[:7]
                                current_gripper = cur_q[7] if len(cur_q) > 7 else 0.0
                                ik_waypoints = []
                                aborted = False
                                for wp_idx, (wp_name, obj_T_wp, wp_dur, grip_angle) in enumerate(obj_T_wps):
                                    T_world_wp = T_world_obj @ obj_T_wp
                                    target_pos, target_quat = _T_to_pos_quat(T_world_wp)

                                    raw_print(f"    WP {wp_idx}/{len(obj_T_wps)-1} '{wp_name}': [{target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}]")
                                    target_quat = tcp_quat
                                    right_side_up_quat = [  0, 0.7071068, 0, 0.7071068]

                                    # best angles -45 and 135
                                    #if target is below -15 go straight if target is above 105 go straight.
                                    # if target is between -15 and 105 go to best angles, then if closer to -15, go -20, if closer to 105, go 20.

           

                                    # Object RPY in a frame where X=up(Z), Y=forward(X), Z=left(Y)
                                    R_perm = np.array([[0, 0, 1], [0, -1, 0], [1, 0, 0]], dtype=float)
                                    R_obj_world = T_world_obj[:3, :3]
                                    R_obj_reframed = R_perm @ R_obj_world
                                    T_reframed = np.eye(4)
                                    T_reframed[:3, :3] = R_obj_reframed
                                    rpy_obj_xup = _quat_to_rpy_deg(_T_to_pos_quat(T_reframed)[1])


                                    additional_yaw_x_rotation = 0.0
                                    if rpy_obj_xup[0] > -15 and rpy_obj_xup[0] < 105:
                                        #if closer to -15, go -20, if closer to 105, go 20.
                                        if rpy_obj_xup[0] < 45:
                                            additional_yaw_x_rotation = 20.0
                                        else:
                                            additional_yaw_x_rotation = -20.0
                                        
                                    added_quat = _quat_from_rpy_deg(additional_yaw_x_rotation, 0, 0)
                                    right_side_up_quat = _quat_multiply(right_side_up_quat, added_quat)

                                    
                                    target_quat = right_side_up_quat
                                    target_q = gravity_comp.inverse_kinematics(
                                        target_pos=target_pos,
                                        target_quat=target_quat,
                                        seed_angles=seed_q,
                                        position=arm_of_choice,
                                    )

                                    verify_pos, verify_quat = gravity_comp.forward_kinematics(
                                        list(target_q), position=arm_of_choice,
                                    )
                                    pos_err = np.linalg.norm(verify_pos - target_pos)
                                    ori_err_vec = np.zeros(3)
                                    vq = verify_quat.copy()
                                    if target_quat is not None:
                                        if np.dot(vq, target_quat) < 0:
                                            vq = -vq
                                        mujoco.mju_subQuat(ori_err_vec, target_quat, vq)
                                        rot_err = np.degrees(np.linalg.norm(ori_err_vec))
                                    else:
                                        rot_err = 0.0
                                    raw_print(f"      IK err: {pos_err*1000:.1f} mm, {rot_err:.1f} deg")
                                    if pos_err > 0.02 or rot_err > 15.0:
                                        raw_print(f"      WARNING: IK error too large, aborting sequence\n")
                                        aborted = True
                                        break

                                    grip_val = grip_angle if grip_angle is not None else current_gripper
                                    target_q_full = list(target_q) + [grip_val]
                                    ik_waypoints.append((target_q_full, wp_dur))
                                    seed_q = list(target_q)
                                    current_gripper = grip_val

                                if not aborted:
                                    total_dur = sum(d for _, d in ik_waypoints)
                                    raw_print(f"    Executing smooth trajectory ({total_dur:.1f}s, {len(ik_waypoints)} segments)...")
                                    await execute_waypoint(
                                        ik_waypoints, chosen_master, chosen_slave,
                                        gravity_comp=gravity_comp,
                                        slave_gravity_comp=slave_gravity_comp,
                                        hz=200.0,
                                    )

                                    if chosen_slave is not None and chosen_slave.channel in _slave_cmd_pos:
                                        _slave_cmd_pos[chosen_slave.channel] = chosen_slave.get_positions()

                                    side_sp = impedance_setpoints.get(arm_of_choice, {})
                                    for axis in (0, 1, 2):
                                        if axis in side_sp:
                                            side_sp[axis] = float(target_pos[axis])
                                    if impedance_lock_ori.get(arm_of_choice, False):
                                        impedance_ori_quats[arm_of_choice] = target_quat.copy()
                                    raw_print(f"    All waypoints reached — resuming teleop\n")
                                else:
                                    raw_print(f"    Sequence aborted\n")
                    elif i == 1:
                        # Button 2: toggle orientation lock
                        # If ON -> turn OFF. If OFF -> try plane, fall back to TCP pose.
                        was_on = any(impedance_lock_ori.values())
                        if was_on:
                            for side in impedance_lock_ori:
                                impedance_lock_ori[side] = False
                                impedance_ori_quats[side] = None
                            raw_print(f"\n  Button 2: orientation lock OFF\n")
                        else:
                            locked_to_plane = False
                            plane_msg = pose_listener.latest_plane if pose_listener else None
                            if plane_msg is not None and gravity_comp is not None:
                                arm_for_lock = next(
                                    (a for a in arms if a.position == "left" and a.is_slave),
                                    next((a for a in arms if a.position == "left"), None),
                                )
                                if arm_for_lock is not None:
                                    cur_q_lock = arm_for_lock.get_positions()
                                    T_cam_plane = _ros_pose_to_T(plane_msg.pose)
                                    cam_pos_tmp, cam_quat = gravity_comp.kdl.compute_body_pose(
                                        np.array(cur_q_lock[:7]), "openarm_left_camera", side="left",
                                    )
                                    T_world_cam = _pose_to_T(cam_pos_tmp, cam_quat)
                                    T_world_plane = T_world_cam @ T_cam_plane
                                    _, plane_quat_world = _T_to_pos_quat(T_world_plane)

                                    _, tcp_quat = gravity_comp.forward_kinematics(
                                        cur_q_lock[:7], position="left",
                                    )
                                    ori_err = np.zeros(3)
                                    pq = plane_quat_world.copy()
                                    if np.dot(pq, tcp_quat) < 0:
                                        pq = -pq
                                    mujoco.mju_subQuat(ori_err, pq, tcp_quat)
                                    angle_deg = np.degrees(np.linalg.norm(ori_err))

                                    if angle_deg <= 15.0:
                                        impedance_lock_ori["left"] = True
                                        impedance_ori_quats["left"] = plane_quat_world
                                        locked_to_plane = True
                                        raw_print(f"\n  Button 2: orientation lock ON (plane, {angle_deg:.1f} deg)\n")
                                    else:
                                        raw_print(f"\n  Button 2: plane ori too far ({angle_deg:.1f} deg) — not locked\n")

                            if not locked_to_plane:
                                raw_print(f"\n  Button 2: no valid plane — not locked\n")
                    elif i == 2:
                        # Button 3: replay loaded playback sequence
                        if _playback_frames and _playback_arm_map:
                            raw_print(f"\n  Button 3: replaying {len(_playback_frames)} frames ...\n")
                            run_playback(
                                _playback_frames, _playback_arm_map,
                                gravity_comp=gravity_comp,
                                slave_gravity_comp=slave_gravity_comp,
                            )
                            await home_all_arms(arms, skip_first=True)
                            _slave_cmd_pos.clear()
                            raw_print(f"\n  Playback done — press button 3 to replay\n")
                        else:
                            raw_print(f"\n  Button 3: no playback loaded\n")
                    else:
                        raw_print(f"\n  Button {i+1} pressed\n")
            _prev_buttons[:] = btns

            # Move cursor up to the first motor line (add +1 for the status line)
            sys.stdout.write(f"\033[{num_motors + 1}A")


            # Small delay before refresh
            await asyncio.sleep(0.001)

            # Control master arms with MIT (gravity comp or zero torque)
            master_arms = {arm.channel: arm for arm in arms if arm.is_master}

            # --- Prepare master arm tasks ---
            # Pre-compute gravity for each master (cheap, CPU-only)
            master_gravity = {}  # channel -> (combined_torques, active_indices)
            for master_arm in master_arms.values():
                gravity_torques = None
                active_indices = []

                if gravity_comp and master_arm.position in ["left", "right"]:
                    active_positions = []
                    active_velocities = []
                    for idx, (motor, state) in enumerate(
                        zip(master_arm.motors, master_arm.states)
                    ):
                        if motor is not None and state:
                            active_positions.append(state.position)
                            active_velocities.append(state.velocity)
                            active_indices.append(idx)

                    if active_positions:
                        gravity_torques = gravity_comp.compute(
                            active_positions, position=master_arm.position,
                        )

                        # Impedance control for this arm's side
                        side = master_arm.position
                        side_sp = impedance_setpoints.get(side, {})
                        lock_ori = impedance_lock_ori.get(side, False)
                        if side_sp or lock_ori:
                            needs_latch = any(v is None for v in side_sp.values())
                            needs_quat = lock_ori and impedance_ori_quats[side] is None

                            if needs_latch or needs_quat:
                                tcp_pos, tcp_quat = gravity_comp.forward_kinematics(
                                    active_positions, position=side,
                                )
                                for axis in list(side_sp):
                                    if side_sp[axis] is None:
                                        side_sp[axis] = float(tcp_pos[axis])
                                if needs_quat:
                                    impedance_ori_quats[side] = tcp_quat.copy()

                            imp_torques = gravity_comp.impedance_torques(
                                active_positions,
                                setpoint=side_sp,
                                lock_orientation=lock_ori,
                                ori_quat=impedance_ori_quats[side],
                                position=side,
                                stiffness=200.0,
                                rot_stiffness=5.0,
                            )

                            gravity_torques = [
                                g + i
                                for g, i in zip(gravity_torques, imp_torques)
                            ]

                master_gravity[master_arm.channel] = (gravity_torques, active_indices)

            # Pre-compute gravity for each slave arm
            slave_gravity: dict[str, tuple] = {}
            _s_gc = slave_gravity_comp or gravity_comp
            for s_arm in [arm for arm in arms if arm.is_slave]:
                grav = None
                s_active = []
                if _s_gc and s_arm.position in ["left", "right"]:
                    for idx, (motor, state) in enumerate(zip(s_arm.motors, s_arm.states)):
                        if motor is not None and state:
                            s_active.append(idx)
                    s_positions = s_arm.get_positions()
                    if s_positions:
                        grav = _s_gc.compute(
                            s_positions, position=s_arm.position,
                        )
                slave_gravity[s_arm.channel] = (grav, s_active)

            # --- Define per-arm BLOCKING workers (run in threads) ---
            def run_master_arm_sync(m_arm: Arm) -> list:
                """Run MIT control for all motors on one master arm (blocking, same bus).

                Uses pipelined send/recv: send ALL commands first, then read ALL responses.
                This overlaps motor processing time with CAN transmission.
                """
                grav_torques, act_indices = master_gravity[m_arm.channel]

                # Phase 1: Send ALL commands as fast as possible
                active_motors = []  # (index, motor) pairs for motors we sent to
                for motor_idx, motor in enumerate(m_arm.motors):
                    if motor is None:
                        continue
                    try:
                        torque = 0.0
                        if grav_torques and motor_idx in act_indices:
                            active_idx = act_indices.index(motor_idx)
                            if active_idx < len(grav_torques):
                                torque = grav_torques[active_idx]

                        params = MitControlParams(
                            q=0, dq=0, kp=0, kd=0, tau=torque,
                        )
                        encode_control_mit(motor._bus, motor._slave_id, motor._motor_limits, params)
                        time.sleep(FRAME_GAP)
                        active_motors.append((motor_idx, motor))
                    except Exception as e:  # noqa: BLE001
                        logger.debug("MIT send failed: %s", e)

                # Phase 2: Read ALL responses
                results = [None] * len(m_arm.motors)
                for motor_idx, motor in active_motors:
                    try:
                        state = decode_motor_state_sync(motor._bus, motor._master_id, motor._motor_limits)
                        results[motor_idx] = state
                    except Exception as e:  # noqa: BLE001
                        logger.debug("MIT recv failed: %s", e)
                return results

            SLAVE_GRAV_THRESH = 10000.05  # rad — add grav comp when |delta_q| below this

            def run_slave_arm_sync(s_arm: Arm, m_arm: Arm) -> list:
                """Run MIT control for all motors on one slave arm with rate-limited tracking."""
                # Initialise command tracker on first call
                if s_arm.channel not in _slave_cmd_pos:
                    _slave_cmd_pos[s_arm.channel] = s_arm.get_positions()
                cmd = _slave_cmd_pos[s_arm.channel]
                max_delta = SLAVE_MAX_VEL * loop_time if loop_time > 0 else 0.0025

                grav_torques, grav_indices = slave_gravity.get(s_arm.channel, (None, []))

                active_motors = []
                results = [None] * len(s_arm.motors)

                for idx, (slave_motor, master_state) in enumerate(
                    zip(s_arm.motors, m_arm.states)
                ):
                    if slave_motor is None or master_state is None:
                        continue
                    try:
                        target = master_state.position
                        if (
                            s_arm.mirror_mode
                            and idx < len(MOTOR_CONFIGS)
                            and MOTOR_CONFIGS[idx].inverted
                        ):
                            target = -target

                        # Rate-limit: clamp delta to max_delta
                        delta = target - cmd[idx]
                        delta = max(-max_delta, min(max_delta, delta))
                        cmd[idx] += delta

                        kp, kd = JOINT_GAINS[idx] if idx < len(JOINT_GAINS) else (2.0, 1.0)

                        torque = 0.0
                        if grav_torques and idx in grav_indices:
                            cur_pos = s_arm.states[idx].position if s_arm.states[idx] is not None else 0.0
                            if abs(cmd[idx] - cur_pos) < SLAVE_GRAV_THRESH:
                                gi = grav_indices.index(idx)
                                if gi < len(grav_torques):
                                    torque = grav_torques[gi]

                        params = MitControlParams(
                            q=cmd[idx], dq=0, kp=kp, kd=kd, tau=torque,
                        )
                        encode_control_mit(slave_motor._bus, slave_motor._slave_id, slave_motor._motor_limits, params)
                        time.sleep(FRAME_GAP)
                        active_motors.append((idx, slave_motor))
                    except Exception as e:  # noqa: BLE001
                        logger.debug("PosVel send failed: %s", e)

                # Phase 2: Read ALL responses
                for idx, slave_motor in active_motors:
                    try:
                        state = decode_motor_state_sync(slave_motor._bus, slave_motor._master_id, slave_motor._motor_limits)

                        if state.status != 1:
                            sys.stdout.write(
                                f"Motor {slave_motor.slave_id}: Position={state.position:.2f}, "
                                f"Velocity={state.velocity:.2f}, Torque={state.torque:.2f}, "
                                f"Temp={state.temp_mos}°C, Status={state.status}\r\n"
                            )
                            sys.stdout.flush()

                        results[idx] = state
                    except Exception as e:  # noqa: BLE001
                        logger.debug("PosVel recv failed: %s", e)
                return results

            # --- Run all arms concurrently across buses using threads ---
            # Phase 1: Run ALL master arms concurrently (different buses)
            master_tasks = []
            master_arm_refs = []
            for m_arm in master_arms.values():
                loop = asyncio.get_event_loop()
                master_tasks.append(loop.run_in_executor(None, run_master_arm_sync, m_arm))
                master_arm_refs.append(m_arm)

            if master_tasks:
                master_results = await asyncio.gather(*master_tasks)
                for m_arm, result in zip(master_arm_refs, master_results):
                    m_arm.states = result

            # Phase 2: Run ALL slave arms concurrently (they now have fresh master states)
            slave_tasks = []
            slave_arm_refs = []
            for s_arm in [arm for arm in arms if arm.is_slave]:
                if s_arm.follows and s_arm.follows in master_arms:
                    m_arm = master_arms[s_arm.follows]
                    slave_tasks.append(loop.run_in_executor(None, run_slave_arm_sync, s_arm, m_arm))
                    slave_arm_refs.append(s_arm)

            if slave_tasks:
                slave_results = await asyncio.gather(*slave_tasks)
                for s_arm, result in zip(slave_arm_refs, slave_results):
                    s_arm.states = result

            # Capture master arm states (setpoints) for recording, keyed by slave side
            if _recording and slave_arm_refs:
                frame: dict[str, list[float]] = {}
                for s_arm in slave_arm_refs:
                    m_arm = master_arms.get(s_arm.follows)
                    if m_arm is None:
                        continue
                    frame[s_arm.position] = m_arm.get_positions()
                _record_buf.append(frame)

            # Update MuJoCo viewer with leader arm joint angles
            if mj_viewer is not None:
                left_q_viz = None
                right_q_viz = None
                for m_arm in master_arm_refs:
                    q = m_arm.get_positions()
                    if m_arm.position == "left":
                        left_q_viz = q
                    elif m_arm.position == "right":
                        right_q_viz = q
                mj_viewer.update(left_q=left_q_viz, right_q=right_q_viz)

            # xArm6 relative-motion mapping from master arms
            if xarm is not None and gravity_comp is not None:
                SETTLE_COUNT = 500

                # Capture offset poses once arms have settled
                if loop_count == SETTLE_COUNT and not _xarm_offsets:
                    for m_arm in master_arm_refs:
                        side = m_arm.position
                        q_rad = m_arm.get_positions()[:7]
                        m_pos, m_quat = gravity_comp.forward_kinematics(q_rad, position=side)
                        T_master_init = _pose_to_T(m_pos, m_quat)
                        T_master_init_inv = _T_inv(T_master_init)

                        pb_q_deg = xarm.get_joints(side)
                        pb_pos, pb_quat = xarm.fk(pb_q_deg, arm=side)
                        T_pb_init = np.eye(4)
                        T_pb_init[:3, 3] = pb_pos
                        T_pb_init[:3, :3] = R_scipy.from_quat(pb_quat).as_matrix()

                        _xarm_offsets[side] = (T_master_init_inv, T_pb_init)
                        _xarm_last_q[side] = pb_q_deg
                        xarm.set_rest(side, pb_q_deg)
                        raw_print(f"  xArm6 {side}: offset captured")

                # Apply relative motion every iteration after offsets are captured
                if _xarm_offsets:
                    for m_arm in master_arm_refs:
                        side = m_arm.position
                        if side not in _xarm_offsets:
                            continue
                        T_master_init_inv, T_pb_init = _xarm_offsets[side]

                        q_rad = m_arm.get_positions()[:7]
                        m_pos, m_quat = gravity_comp.forward_kinematics(q_rad, position=side)
                        T_master_now = _pose_to_T(m_pos, m_quat)

                        T_delta = T_master_init_inv @ T_master_now
                        T_pb_target = T_pb_init @ T_delta

                        target_pos = T_pb_target[:3, 3]
                        target_quat = R_scipy.from_matrix(T_pb_target[:3, :3]).as_quat()  # xyzw

                        t_ik_start = time.time()
                        ik_q = xarm.ik_safe(
                            target_pos, target_quat,
                            seed_deg=_xarm_last_q[side], arm=side,
                        )
                        t_ik_ms = (time.time() - t_ik_start) * 1000.0
                        if ik_q is not None:
                            _xarm_last_q[side] = ik_q
                            xarm.set_joints(ik_q, arm=side)
                            master_grip = m_arm.states[7].position if m_arm.states[7] is not None else 0.0
                            grip_xarm = max(0.0, min(850.0, abs(master_grip) / 0.785 * 850.0))
                            xarm.send_to_real(ik_q, gripper_pos=grip_xarm, arm=side)
                        else:
                            xarm.set_joints(_xarm_last_q[side], arm=side)

                        if loop_count % 200 == 0:
                            raw_print(f"  xArm6 {side} IK: {t_ik_ms:.1f}ms")
                    xarm.step()

            # Publish joint states to ROS 2
            if pose_listener and HAS_ROS2:
                for m_arm in master_arm_refs:
                    observed = m_arm.get_positions()
                    commanded = [0.0] * len(observed)
                    arm_name = f"master_{m_arm.position}"
                    pose_listener.publish_joint_state(arm_name, commanded, observed)

                for s_arm, m_arm_name in [(sa, sa.follows) for sa in slave_arm_refs]:
                    m_arm = master_arms.get(m_arm_name)
                    observed = s_arm.get_positions()
                    commanded = []
                    for idx, ms in enumerate(m_arm.states if m_arm else []):
                        pos = ms.position if ms is not None else 0.0
                        if (
                            s_arm.mirror_mode
                            and idx < len(MOTOR_CONFIGS)
                            and MOTOR_CONFIGS[idx].inverted
                        ):
                            pos = -pos
                        commanded.append(pos)
                    arm_name = f"follower_{s_arm.position}"
                    pose_listener.publish_joint_state(arm_name, commanded, observed)

            # Print pose and update visualizer
            fp_pose = pose_listener.latest
            right_slave = next(
                (a for a in arms if a.is_slave and a.position == "right"), None,
            )
            if fp_pose is not None and gravity_comp is not None and right_slave is not None:
                cur_q = right_slave.get_positions()

                tcp_pos, tcp_quat = gravity_comp.forward_kinematics(cur_q[:7], position="right")
                cam_pos, cam_quat = gravity_comp.kdl.compute_body_pose(
                    np.array(cur_q[:7]), "openarm_right_camera", side="right",
                )
                T_world_tcp = _pose_to_T(tcp_pos, tcp_quat)
                T_world_cam = _pose_to_T(cam_pos, cam_quat)
                T_world_obj = T_world_cam @ _ros_pose_to_T(fp_pose.pose)

                obj_in_tcp = (np.linalg.inv(T_world_tcp) @ T_world_obj)[:3, 3]
                obj_in_world = T_world_obj[:3, 3]
                rpy_tcp = _quat_to_rpy_deg(tcp_quat)
                rpy_obj = _quat_to_rpy_deg(_T_to_pos_quat(T_world_obj)[1])

                # Object RPY in a frame where X=up(Z), Y=forward(X), Z=left(Y)
                R_perm = np.array([[0, 0, 1], [0, -1, 0], [1, 0, 0]], dtype=float)
                R_obj_world = T_world_obj[:3, :3]
                R_obj_reframed = R_perm @ R_obj_world
                T_reframed = np.eye(4)
                T_reframed[:3, :3] = R_obj_reframed
                rpy_obj_xup = _quat_to_rpy_deg(_T_to_pos_quat(T_reframed)[1])

                pose_str = (
                    f"  tcp_T_obj: x={obj_in_tcp[0]:.3f} y={obj_in_tcp[1]:.3f} z={obj_in_tcp[2]:.3f}"
                    f"  |  world_obj: x={obj_in_world[0]:.3f} y={obj_in_world[1]:.3f} z={obj_in_world[2]:.3f}"
                    f"  |  TCP rpy: [{rpy_tcp[0]:.1f}, {rpy_tcp[1]:.1f}, {rpy_tcp[2]:.1f}]"
                    f"  |  Obj rpy: [{rpy_obj[0]:.1f}, {rpy_obj[1]:.1f}, {rpy_obj[2]:.1f}]"
                    f"  |  Obj rpy(Xup): [{rpy_obj_xup[0]:.1f}, {rpy_obj_xup[1]:.1f}, {rpy_obj_xup[2]:.1f}]"
                )
                frame_vis.update(TCP=T_world_tcp, CAM=T_world_cam, OBJ=T_world_obj)
            elif fp_pose is not None:
                p = fp_pose.pose.position
                pose_str = f"  FPose(cam): x={p.x:.3f} y={p.y:.3f} z={p.z:.3f}"
            else:
                pose_str = "  FPose: waiting..."
            sys.stdout.write(f"\r{pose_str}\033[K\r\n")
            sys.stdout.flush()

    except KeyboardInterrupt:
        # Move cursor below all motor lines
        sys.stdout.write(f"\033[{num_motors}B\n")
        if not raw_mode:
            sys.stdout.write("\nTeleoperation stopped.\n")
        else:
            raw_print("\nTeleoperation stopped.")

    finally:
        # Auto-save recording if still active
        if _recording and _record_buf and _record_file:
            _recording = False
            save_recording(_record_buf, _record_file, _rec_sides, _rec_n_joints)

        # Save calibration waypoints if any were captured
        if _wp_buf and _wp_file:
            with open(_wp_file, "w") as f:
                f.write("# Calibration waypoints (left_j0..j7 right_j0..j7)\n")
                for wp in _wp_buf:
                    f.write(" ".join(f"{v:.6f}" for v in wp) + "\n")
            sys.stdout.write(f"Saved {len(_wp_buf)} waypoints → {_wp_file}\n")

        # Restore terminal settings first
        if old_settings is not None and HAS_TERMIOS:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            except (OSError, termios.error) as e:
                logger.debug("Failed to restore terminal settings: %s", e)

        # SAFETY: Disable ALL motors (not just slaves) for safety
        sys.stdout.write("\nDisabling ALL motors for safety...\n")
        for arm in arms:
            await arm.disable_all_motors()
        sys.stdout.write("All motors disabled.\n")

        pose_listener.shutdown()
        frame_vis.close()
        if xarm is not None:
            xarm.disconnect()


def parse_arguments() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(description="Monitor Damiao motor angles")

    parser.add_argument(
        "--interface",
        "-i",
        default="can0",
        help="CAN interface name (default: can0, ignored on Windows/macOS)",
    )

    parser.add_argument(
        "--teleop",
        "-t",
        action="store_true",
        default=False,
        help="Enable teleoperation mode (enables motors with control mode)",
    )

    parser.add_argument(
        "--follow",
        action="append",
        help=(
            "Define follower mappings as MASTER:POSITION:SLAVE:POSITION "
            "where POSITION is 'left' or 'right'. "
            "Mirror mode is automatic when positions differ. "
            "(e.g., --follow can0:left:can1:right for mirror, "
            "--follow can0:left:can1:left for no mirror)"
        ),
    )

    parser.add_argument(
        "--gravity",
        "-g",
        action="store_true",
        help="Enable gravity compensation (MIT mode only)",
    )

    parser.add_argument(
        "--velocity",
        "-v",
        type=float,
        default=1.0,
        help="Velocity parameter for slave motors (default: 1.0)",
    )

    parser.add_argument(
        "--record",
        type=str,
        default=None,
        metavar="FILE",
        help="Enable recording mode. Slave joint angles are saved to FILE on exit or button press.",
    )

    parser.add_argument(
        "--playback",
        type=str,
        default=None,
        metavar="FILE",
        help="Playback mode: load a recorded sequence from FILE, home to the first frame, then replay it on the slave arms. Skips normal teleop.",
    )

    parser.add_argument(
        "--use-xarm",
        action="store_true",
        default=False,
        help="Use PyBullet xArm6 visualizer and real xArm6 arms",
    )

    parser.add_argument(
        "--waypoints",
        type=str,
        default=None,
        metavar="FILE",
        help=(
            "Record calibration waypoints.  Press 'w' during teleop to snapshot "
            "current slave arm positions as a waypoint.  Saved to FILE on exit "
            "(16 values per line: 8 left + 8 right).  Use with --calibrate later."
        ),
    )

    parser.add_argument(
        "--calibrate",
        type=str,
        default=None,
        metavar="WAYPOINT_FILE",
        help=(
            "Calibration data collection mode.  Moves slave arms through waypoints "
            "from WAYPOINT_FILE, captures images from ROS 2 camera topics, and saves "
            "body_T_ee poses to an output directory.  Requires --gravity and --follow."
        ),
    )
    parser.add_argument(
        "--calib-output",
        type=str,
        default="calib_data",
        metavar="DIR",
        help="Output directory for calibration data (default: calib_data).",
    )
    parser.add_argument(
        "--calib-left-topic",
        type=str,
        default="/camera_left/infra1/image_rect_raw",
        help="ROS 2 image topic for left arm camera.",
    )
    parser.add_argument(
        "--calib-right-topic",
        type=str,
        default="/camera_right/infra1/image_rect_raw",
        help="ROS 2 image topic for right arm camera.",
    )

    parser.add_argument(
        "--cam-extrinsics",
        type=str,
        default=None,
        metavar="FILE",
        help=(
            "Path to ee_T_cam.yaml (produced by calibrate_extrinsics.py). "
            "Loads calibrated ee_T_cam transforms for left/right cameras."
        ),
    )

    return parser.parse_args()


def run() -> None:
    """Entry point for the monitor script."""
    args = parse_arguments()

    try:
        asyncio.run(main(args))
    except KeyboardInterrupt:
        sys.stderr.write("\nInterrupted by user.\n")
        sys.exit(0)
    except Exception as e:
        logger.exception("Fatal error")
        sys.stderr.write(f"Error: {e}\n")
        sys.exit(1)


if __name__ == "__main__":
    run()
