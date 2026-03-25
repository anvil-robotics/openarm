from __future__ import annotations

"""Gravity compensation for robotic arms using MuJoCo physics simulation."""

import argparse
import asyncio
import os
import re
import sys

os.environ.setdefault("MUJOCO_GL", "disable")

import mujoco
import numpy as np

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

import builtins
import contextlib

import can

from openarm.bus import Bus
from openarm.damiao import Arm, ControlMode, Motor, detect_motors
from openarm.damiao.config import MOTOR_CONFIGS
from openarm.simulation.models import OPENARM_MODEL_PATH


class ArmWithGravity(Arm):
    """Extended Arm class with gravity compensation support."""

    def __init__(self, motors: list[Motor], position: str, can_bus: can.BusABC) -> None:
        """Initialize the GravityArm with motors and position."""
        # Initialize parent Arm with all motors
        super().__init__(motors)

        # Store additional attributes needed for gravity compensation
        self.position = position  # "left" or "right"
        self.can_bus = can_bus
        self.positions = [0.0] * len(motors)  # Position for each motor


def patch_model_camera_bodies(model: mujoco.MjModel, ee_T_cam: dict) -> None:
    """Overwrite camera body pos/quat in a MuJoCo model with calibrated ee_T_cam.

    Args:
        model: A loaded MuJoCo model to patch in-place.
        ee_T_cam: dict mapping "left"/"right" to 4x4 ee_T_cam numpy arrays.
    """
    for side, T_tcp_cam in ee_T_cam.items():
        cam_body = f"openarm_{side}_camera"
        tcp_body = f"openarm_{side}_hand_tcp"

        cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, cam_body)
        tcp_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, tcp_body)
        if cam_id < 0 or tcp_id < 0:
            continue

        hand_T_tcp = np.eye(4)
        hand_T_tcp[:3, 3] = model.body_pos[tcp_id]
        q = model.body_quat[tcp_id]
        R = np.empty(9)
        mujoco.mju_quat2Mat(R, q)
        hand_T_tcp[:3, :3] = R.reshape(3, 3)

        hand_T_cam = hand_T_tcp @ T_tcp_cam

        model.body_pos[cam_id] = hand_T_cam[:3, 3]
        quat_cam = np.empty(4)
        mujoco.mju_mat2Quat(quat_cam, hand_T_cam[:3, :3].flatten())
        model.body_quat[cam_id] = quat_cam


class MuJoCoKDL:
    """A simple class for computing inverse dynamics using MuJoCo."""

    def __init__(
        self,
        model_path: str | None = None,
        body_mass_overrides: dict[str, float] | None = None,
    ) -> None:
        """Initialize MuJoCo model for kinematic/dynamic calculations.

        Args:
            model_path: Path to the MuJoCo XML model. Defaults to OPENARM_MODEL_PATH.
            body_mass_overrides: Optional dict mapping body names to new masses (kg).
                E.g. {"openarm_left_hand": 0.25, "openarm_right_hand": 0.25}
        """
        path = model_path or str(OPENARM_MODEL_PATH)
        self.model = mujoco.MjModel.from_xml_path(path)
        self.model.opt.gravity = np.array([0, 0, -9.81])

        if body_mass_overrides:
            for body_name, mass in body_mass_overrides.items():
                body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body_name)
                if body_id >= 0:
                    self.model.body_mass[body_id] = mass

        self.data = mujoco.MjData(self.model)

        # Disable all collisions
        self.model.geom_contype[:] = 0
        self.model.geom_conaffinity[:] = 0

        # Disable all joint limit
        self.model.jnt_limited[:] = 0

    def patch_camera_bodies(self, ee_T_cam: dict) -> None:
        """Overwrite camera body pos/quat with calibrated ee_T_cam transforms."""
        patch_model_camera_bodies(self.model, ee_T_cam)

    def compute_inverse_dynamics(
        self, q: np.ndarray, qdot: np.ndarray, qdotdot: np.ndarray, side: str = "left"
    ) -> np.ndarray:
        """Compute inverse dynamics for the given joint states."""
        assert len(q) == len(qdot) == len(qdotdot)
        assert side in ["left", "right"], "side must be 'left' or 'right'"

        length = len(q)

        # Left joints: indices 0-7 (8 motors)
        # Right joints: indices 9-16 (8 motors), but input q is still 0-7
        joint_indices = slice(0, length) if side == "left" else slice(9, 9 + length)

        # Clear all joint states first
        self.data.qpos[:] = 0
        self.data.qvel[:] = 0
        self.data.qacc[:] = 0

        # Set joint states for the specified side
        self.data.qpos[joint_indices] = q
        self.data.qvel[joint_indices] = qdot
        self.data.qacc[joint_indices] = qdotdot

        mujoco.mj_inverse(self.model, self.data)
        return self.data.qfrc_inverse[joint_indices]

    def compute_body_pose(
        self, q: np.ndarray, body_name: str, side: str = "left"
    ) -> tuple[np.ndarray, np.ndarray]:
        """Compute the world-frame pose of a body given joint angles.

        Args:
            q: Joint angles in radians.
            body_name: MuJoCo body name to query.
            side: "left" or "right".

        Returns:
            Tuple of (pos, quat) in world frame. quat is [w, x, y, z].
        """
        assert side in ("left", "right"), "side must be 'left' or 'right'"
        length = len(q)
        joint_indices = slice(0, length) if side == "left" else slice(9, 9 + length)
        self.data.qpos[:] = 0
        self.data.qvel[:] = 0
        self.data.qpos[joint_indices] = q
        mujoco.mj_forward(self.model, self.data)

        body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        return self.data.xpos[body_id].copy(), self.data.xquat[body_id].copy()

    def compute_forward_kinematics(
        self, q: np.ndarray, side: str = "left"
    ) -> tuple[np.ndarray, np.ndarray]:
        """Compute forward kinematics for the TCP body. Shorthand for compute_body_pose."""
        return self.compute_body_pose(q, f"openarm_{side}_hand_tcp", side=side)

    def compute_jacobian(
        self, q: np.ndarray, side: str = "left"
    ) -> np.ndarray:
        """Compute the full 6 x n_joints Jacobian for the TCP body.

        Rows 0-2 are translational (linear velocity), rows 3-5 are
        rotational (angular velocity about world x, y, z).

        Args:
            q: Joint angles in radians.
            side: "left" or "right".

        Returns:
            Jacobian of shape (6, len(q)).

        """
        assert side in ("left", "right"), "side must be 'left' or 'right'"
        length = len(q)
        joint_indices = slice(0, length) if side == "left" else slice(9, 9 + length)

        self.data.qpos[:] = 0
        self.data.qvel[:] = 0
        self.data.qpos[joint_indices] = q

        mujoco.mj_forward(self.model, self.data)

        tcp_name = f"openarm_{side}_hand_tcp"
        body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, tcp_name)

        nv = self.model.nv
        jacp = np.zeros((3, nv))  # translational
        jacr = np.zeros((3, nv))  # rotational
        mujoco.mj_jacBody(self.model, self.data, jacp, jacr, body_id)

        # Extract only the columns for this arm's joints
        jac_indices = slice(0, length) if side == "left" else slice(9, 9 + length)
        return np.vstack([jacp[:, jac_indices], jacr[:, jac_indices]])


class GravityCompensator:
    """Gravity compensation calculator with persistent MuJoCo model."""

    def __init__(
        self,
        model_path: str | None = None,
        body_mass_overrides: dict[str, float] | None = None,
    ) -> None:
        """Initialize the gravity compensator with MuJoCo model.

        Args:
            model_path: Path to the MuJoCo XML model. Defaults to OPENARM_MODEL_PATH.
            body_mass_overrides: Optional dict mapping body names to new masses (kg).
        """
        self.kdl = MuJoCoKDL(model_path=model_path, body_mass_overrides=body_mass_overrides)
        self.tuning_factors = [0.8, 0.8, 1.0, 1.0, 0.8, 0.8, 0.8, 0.0]

    def compute(self, angles: list[float], position: str = "left") -> list[float]:
        """Compute gravity compensation torques for given joint angles.

        Args:
            angles: List of joint angles in radians
            position: "left" or "right" - determines if mirror motors should have
                negated torques

        Returns:
            List of gravity compensation torques for each joint

        """
        q = np.array(angles[0:7])

        gravity_torques = self.kdl.compute_inverse_dynamics(
            q, np.zeros(q.shape), np.zeros(q.shape), side=position
        )

        # Apply tuning factors
        return [
            torque * factor
            for torque, factor in zip(
                gravity_torques, self.tuning_factors
            )
        ]

    def forward_kinematics(
        self, angles: list[float], position: str = "left"
    ) -> tuple[np.ndarray, np.ndarray]:
        """Compute forward kinematics (end-effector pose) for given joint angles.

        Args:
            angles: List of joint angles in radians.
            position: "left" or "right" arm.

        Returns:
            Tuple of (pos, quat) where pos is the 3-D TCP position [x, y, z]
            and quat is the orientation as a unit quaternion [w, x, y, z].

        """
        return self.kdl.compute_forward_kinematics(np.array(angles), side=position)

    def inverse_kinematics(
        self,
        target_pos: np.ndarray,
        target_quat: np.ndarray | None = None,
        seed_angles: list[float] | None = None,
        position: str = "left",
        max_iter: int = 200,
        pos_tol: float = 1e-3,
        ori_tol: float = 1e-2,
        damping: float = 1e-3,
        step_size: float = 0.5,
    ) -> np.ndarray:
        """Solve IK using damped least-squares with the MuJoCo model.

        Args:
            target_pos: Desired TCP position [x, y, z].
            target_quat: Desired TCP orientation [w, x, y, z].
                If None, only position is targeted.
            seed_angles: Initial joint angle guess (7 joints).
                If None, uses zeros.
            position: "left" or "right" arm.
            max_iter: Maximum solver iterations.
            pos_tol: Position convergence tolerance (metres).
            ori_tol: Orientation convergence tolerance (rad).
            damping: Damped least-squares regularisation.
            step_size: Step size for joint updates.

        Returns:
            Array of 7 joint angles in radians.

        """
        n = 7
        q = np.array(seed_angles[:n], dtype=np.float64) if seed_angles else np.zeros(n)

        # Joint limits from MuJoCo model
        joint_lo = np.zeros(n)
        joint_hi = np.zeros(n)
        joint_offset = 0 if position == "left" else 9
        for i in range(n):
            jid = joint_offset + i
            if self.kdl.model.jnt_limited[jid]:
                joint_lo[i] = self.kdl.model.jnt_range[jid, 0] + 0.2
                joint_hi[i] = self.kdl.model.jnt_range[jid, 1] - 0.2
            else:
                joint_lo[i] = -np.pi
                joint_hi[i] = np.pi

        pos_only = target_quat is None

        for _ in range(max_iter):
            tcp_pos, tcp_quat = self.kdl.compute_forward_kinematics(q, side=position)

            pos_err = target_pos - tcp_pos
            if pos_only:
                err = pos_err
                jac = self.kdl.compute_jacobian(q, side=position)[:3]  # 3×N
            else:
                ori_err_body = np.zeros(3)
                tgt = target_quat.copy()
                if np.dot(tgt, tcp_quat) < 0:
                    tgt = -tgt
                mujoco.mju_subQuat(ori_err_body, tgt, tcp_quat)
                R_tcp = np.zeros(9)
                mujoco.mju_quat2Mat(R_tcp, tcp_quat)
                ori_err = R_tcp.reshape(3, 3) @ ori_err_body
                err = np.concatenate([pos_err, ori_err])
                jac = self.kdl.compute_jacobian(q, side=position)  # 6×N

            if np.linalg.norm(pos_err) < pos_tol:
                if pos_only or np.linalg.norm(ori_err) < ori_tol:
                    break

            # Damped least-squares: dq = J^T (J J^T + λI)^{-1} err
            JJT = jac @ jac.T + damping * np.eye(jac.shape[0])
            dq = jac.T @ np.linalg.solve(JJT, err)

            q = q + step_size * dq
            q = np.clip(q, joint_lo, joint_hi)

        return q

    def impedance_torques(
        self,
        angles: list[float],
        setpoint: dict[int, float],
        lock_orientation: bool = False,
        ori_quat: np.ndarray | None = None,
        position: str = "left",
        stiffness: float = 200.0,
        rot_stiffness: float = 5.0,
    ) -> list[float]:
        """Compute joint torques for Cartesian impedance control via J^T.

        Only the axes present in *setpoint* contribute a translational
        restoring force.  When *lock_orientation* is ``True`` the full
        orientation is held at *ori_quat* (all three rotation axes).

        ``tau = J^T @ wrench``

        Args:
            angles: Current joint angles in radians.
            setpoint: ``{axis: value}`` for translational axes (0=x, 1=y,
                2=z in metres).
            lock_orientation: If True, apply a restoring torque on all three
                rotation axes to hold the orientation given by *ori_quat*.
            ori_quat: Desired orientation as a unit quaternion [w,x,y,z].
                Required when *lock_orientation* is True.
            position: "left" or "right" arm.
            stiffness: Translational spring constant (N/m).
            rot_stiffness: Rotational spring constant (N·m/rad).

        Returns:
            List of impedance joint torques (same length as *angles*).

        """
        q = np.array(angles)

        tcp_pos, tcp_quat = self.kdl.compute_forward_kinematics(q, side=position)

        wrench = np.zeros(6)

        for axis in (0, 1, 2):
            if axis in setpoint:
                wrench[axis] = stiffness * (setpoint[axis] - tcp_pos[axis])

        if lock_orientation and ori_quat is not None:
            # Ensure quaternions are in the same hemisphere to avoid sign flip
            target = ori_quat.copy()
            if np.dot(target, tcp_quat) < 0:
                target = -target

            # mju_subQuat returns error in tcp_quat's body frame
            ori_err_body = np.zeros(3)
            mujoco.mju_subQuat(ori_err_body, target, tcp_quat)

            # Rotate error from body frame to world frame (Jacobian is world-frame)
            R_tcp = np.zeros(9)
            mujoco.mju_quat2Mat(R_tcp, tcp_quat)
            ori_err_world = R_tcp.reshape(3, 3) @ ori_err_body

            wrench[3:6] = rot_stiffness * ori_err_world

        jac = self.kdl.compute_jacobian(q, side=position)
        tau = jac.T @ wrench
        return tau.tolist()

    def impedance_torques_6d(
        self,
        angles: list[float],
        target_pos: np.ndarray,
        target_quat: np.ndarray,
        position: str = "left",
        trans_stiffness: np.ndarray | None = None,
        rot_stiffness: np.ndarray | None = None,
    ) -> list[float]:
        """Impedance torques with independent per-axis stiffness.

        Args:
            angles: Current joint angles in radians.
            target_pos: Desired TCP position [x, y, z] in metres.
            target_quat: Desired TCP orientation [w, x, y, z].
            position: "left" or "right" arm.
            trans_stiffness: Stiffness per translational axis [kx, ky, kz] (N/m).
            rot_stiffness: Stiffness per rotational axis [kr, kp, ky] (N·m/rad).

        Returns:
            List of impedance joint torques (same length as *angles*).
        """
        q = np.array(angles)
        tcp_pos, tcp_quat = self.kdl.compute_forward_kinematics(q, side=position)

        if trans_stiffness is None:
            trans_stiffness = np.zeros(3)
        if rot_stiffness is None:
            rot_stiffness = np.zeros(3)
        trans_k = np.asarray(trans_stiffness, dtype=np.float64)
        rot_k = np.asarray(rot_stiffness, dtype=np.float64)

        wrench = np.zeros(6)
        wrench[:3] = trans_k * (np.asarray(target_pos) - tcp_pos)

        if np.any(rot_k > 0):
            tgt = np.asarray(target_quat, dtype=np.float64).copy()
            if np.dot(tgt, tcp_quat) < 0:
                tgt = -tgt
            ori_err_body = np.zeros(3)
            mujoco.mju_subQuat(ori_err_body, tgt, tcp_quat)
            R_tcp = np.zeros(9)
            mujoco.mju_quat2Mat(R_tcp, tcp_quat)
            ori_err_world = R_tcp.reshape(3, 3) @ ori_err_body
            wrench[3:6] = rot_k * ori_err_world

        jac = self.kdl.compute_jacobian(q, side=position)
        tau = jac.T @ wrench
        return tau.tolist()


def parse_arguments() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="Gravity compensation for Damiao motors"
    )

    parser.add_argument(
        "--port",
        action="append",
        required=True,
        help=(
            "CAN ports with position to use (e.g., --port can0:left --port can1:right)"
        ),
    )

    return parser.parse_args()


async def main(args: argparse.Namespace) -> None:  # noqa: C901, PLR0912
    """Run main gravity compensation loop with proper shutdown handling."""
    # Parse port:position pairs first (before creating buses)
    port_configs = []  # List of (port_name, position)
    for port_spec in args.port:
        parts = port_spec.split(":")
        if len(parts) != 2:  # noqa: PLR2004
            # Invalid format, exit early
            sys.stderr.write(
                f"Error: Invalid format '{port_spec}'. Expected format: PORT:POSITION\n"
            )
            sys.stderr.write("Example: --port can0:left --port can1:right\n")
            return
        port_name, position = parts
        if position not in ["left", "right"]:
            # Invalid position, exit early
            sys.stderr.write(
                f"Error: Invalid position '{position}'. Must be 'left' or 'right'\n"
            )
            return
        port_configs.append((port_name, position))

    # Now create CAN buses after validation
    try:
        all_can_buses = [
            can.Bus(channel=config["channel"], interface=config["interface"])
            for config in can.detect_available_configs("socketcan")
        ]
    except Exception:  # noqa: BLE001
        all_can_buses = []

    if not all_can_buses:
        return

    # Filter buses based on specified ports and attach position
    selected_buses = []  # List of (bus, position)
    for bus in all_can_buses:
        bus_channel = (
            str(bus.channel_info) if hasattr(bus, "channel_info") else str(bus.channel)
        )
        for port_name, position in port_configs:
            if port_name in bus_channel:
                selected_buses.append((bus, position))
                break

    if not selected_buses:
        for bus in all_can_buses:
            bus.shutdown()
        return

    for bus, _position in selected_buses:
        bus_channel = (
            str(bus.channel_info) if hasattr(bus, "channel_info") else str(bus.channel)
        )
        # Extract just the channel name for cleaner display
        if "channel" in bus_channel:
            match = re.search(r"channel ['\"]?(\w+)", bus_channel)
            match.group(1) if match else bus_channel
        else:
            bus_channel.split()[-1] if bus_channel else "unknown"

    # Store arms for cleanup
    arms: list[ArmWithGravity] = []

    try:
        # Run gravity compensation for all selected buses together
        arms = await _main(selected_buses)
    finally:
        # SAFETY: Disable all motors first to avoid unwanted movements
        if arms:
            for arm in arms:
                await arm.disable()

        # Then shutdown all CAN buses
        for bus in all_can_buses:
            bus.shutdown()


def check_keyboard_input() -> str | None:
    """Check if a key has been pressed (non-blocking)."""
    if HAS_MSVCRT:
        # Windows
        if msvcrt.kbhit():
            return msvcrt.getch().decode("utf-8", errors="ignore").lower()
    elif HAS_TERMIOS and select.select([sys.stdin], [], [], 0)[0]:
        # Unix/Linux/Mac
        return sys.stdin.read(1).lower()
    return None


async def _main(selected_buses: list) -> list[ArmWithGravity]:  # noqa: C901, PLR0912
    """Run gravity compensation loop for all selected buses with their positions.

    Returns:
        List of Arm objects for cleanup in main()

    """
    # Initialize gravity compensator
    gravity_comp = GravityCompensator()

    # Setup motors on all selected buses

    # Create Arm objects for each bus
    arms: list[ArmWithGravity] = []

    for _bus_idx, (can_bus, arm_position) in enumerate(selected_buses):
        # First use detect_motors to check if ALL motors are present
        slave_ids = [config.slave_id for config in MOTOR_CONFIGS]
        detected = list(detect_motors(can_bus, slave_ids, timeout=0.01))
        detected_ids = {info.slave_id for info in detected}

        # Check if ALL required motors are detected
        missing_motors = [
            config.name
            for config in MOTOR_CONFIGS
            if config.slave_id not in detected_ids
        ]

        if missing_motors:
            continue

        # Create ALL motors for this arm
        motors = []
        for config in MOTOR_CONFIGS:
            bus = Bus(can_bus)
            motor = Motor(
                bus,
                slave_id=config.slave_id,
                master_id=config.master_id,
                motor_type=config.type,
            )
            motors.append(motor)

        # Create ArmWithGravity with ALL motors
        arm = ArmWithGravity(motors=motors, position=arm_position, can_bus=can_bus)

        try:
            # Enable all motors at once
            states = await arm.enable()

            # Set control mode for all motors at once
            await arm.set_control_mode(ControlMode.MIT)

            # Initialize positions from enable response
            for i, state in enumerate(states):
                if state:
                    arm.positions[i] = state.position

            # Successfully initialized, add to arms list
            arms.append(arm)

        except Exception:  # noqa: BLE001, S110
            pass
            # Don't add to arms list - this arm is broken

    # Count total motors across all working arms
    total_motors = sum(len(arm.motors) for arm in arms)

    if total_motors == 0:
        return []

    # Report motors per arm
    for _arm_idx, _arm in enumerate(arms):
        pass

    # NOW set terminal to raw mode for keyboard detection during the main loop
    old_settings = None
    raw_mode = False
    if HAS_TERMIOS:
        try:
            old_settings = termios.tcgetattr(sys.stdin)
            tty.setraw(sys.stdin.fileno())
            raw_mode = True
        except (OSError, termios.error):
            # Might fail in some environments (e.g., when piped)
            pass

    # Helper function for printing in raw mode
    def raw_print(msg: str = "") -> None:
        """Print with proper line endings in raw mode."""
        if raw_mode:
            sys.stdout.write(msg.replace("\n", "\r\n"))
            sys.stdout.flush()
        else:
            sys.stdout.write(msg + "\n")

    try:
        while True:
            # Check for 'Q' key press
            key = check_keyboard_input()
            if key == "q":
                break

            # Process each arm
            for arm_idx, arm in enumerate(arms):
                # Compute gravity compensation torques for all motors
                torques = gravity_comp.compute(arm.positions, position=arm.position)

                # Use Arm's batch control method for all motors at once
                try:
                    states = await arm.control_mit(
                        kp=0,  # No position gain
                        kd=0,  # No damping gain
                        q=0,  # No position control
                        dq=0,  # No velocity control
                        tau=torques,  # Gravity compensation torques for all motors
                    )

                    # Update positions from motor responses
                    for i, state in enumerate(states):
                        if state:
                            arm.positions[i] = state.position

                except Exception as e:  # noqa: BLE001
                    raw_print(f"Error in batch control on arm {arm_idx + 1}: {e}")

            # Small delay
            await asyncio.sleep(0.01)

        raw_print("\nStopping gravity compensation...")

    except Exception as e:  # noqa: BLE001
        raw_print(f"\nError in gravity compensation loop: {e}")

    finally:
        # Restore terminal settings (Unix/Linux/Mac)
        if old_settings is not None and HAS_TERMIOS:
            with contextlib.suppress(builtins.BaseException):
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

        # SAFETY: Disable all motors to avoid unwanted movements
        raw_print("Disabling all motors for safety...")
        for arm in arms:
            await arm.disable()

    # Return arms for cleanup in main()
    return arms


def run() -> None:
    """Run the gravity compensation script."""
    args = parse_arguments()

    try:
        asyncio.run(main(args))
    except KeyboardInterrupt:
        sys.exit(0)
    except Exception:  # noqa: BLE001
        sys.exit(1)


if __name__ == "__main__":
    run()
