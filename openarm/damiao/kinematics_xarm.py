"""
Kinematics and visualization for the xArm6 robot using PyBullet.

Provides FK, IK, and a dual-arm 3D viewer backed by a single PyBullet
physics client.  Intended to be used from monitor.py as the single point
of contact for everything xArm6-related.
"""

from __future__ import annotations

import contextlib
import logging
import math
import os
import sys
import time
from pathlib import Path
from typing import Optional, Tuple

import numpy as np
import pybullet as p
import pybullet_data
from scipy.spatial.transform import Rotation as R

logger = logging.getLogger(__name__)

# ── xArm6 joint configuration ───────────────────────────────────────────
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
NUM_JOINTS = len(JOINT_NAMES)
END_EFFECTOR_LINK_NAME = "left_finger"

_DEFAULT_URDF = str(Path(__file__).resolve().parent.parent / "xarm" / "xarm6_with_gripper.urdf")


@contextlib.contextmanager
def _suppress_stdout_stderr():
    """Temporarily redirect stdout/stderr to /dev/null at the fd level."""
    stdout_fd = sys.stdout.fileno()
    stderr_fd = sys.stderr.fileno()
    saved_out = os.dup(stdout_fd)
    saved_err = os.dup(stderr_fd)
    try:
        devnull = os.open(os.devnull, os.O_WRONLY)
        os.dup2(devnull, stdout_fd)
        os.dup2(devnull, stderr_fd)
        yield
    finally:
        os.dup2(saved_out, stdout_fd)
        os.dup2(saved_err, stderr_fd)
        os.close(saved_out)
        os.close(saved_err)
        os.close(devnull)


class XArm6:
    """PyBullet-backed FK / IK solver **and** visualiser for two xArm6 arms.

    Usage::

        arm = XArm6(use_gui=True)
        arm.setup()                        # opens the viewer window
        arm.set_joints([0,-45,-45,0,0,0], 'left')   # degrees
        pos, quat = arm.fk([0,-45,-45,0,0,0], 'left')
        ik_q = arm.ik(pos, quat, seed_deg=[0,-45,-45,0,0,0], arm='left')
        arm.disconnect()
    """

    def __init__(
        self,
        urdf_path: str = _DEFAULT_URDF,
        use_gui: bool = True,
        suppress_output: bool = True,
        connect_real: bool = False,
    ):
        self.urdf_path = urdf_path
        self.use_gui = use_gui
        self.suppress_output = suppress_output
        self.connect_real = connect_real

        self.physics_client: int = -1
        self.robot_ids: dict[str, int | None] = {"left": None, "right": None}
        self.joint_indices: dict[str, list[int | None]] = {
            "left": [None] * NUM_JOINTS,
            "right": [None] * NUM_JOINTS,
        }
        self.ee_link_idx: dict[str, int] = {"left": -1, "right": -1}
        self._gripper_idx: dict[str, int | None] = {"left": None, "right": None}

        self.joint_limits_lower_deg = np.full(NUM_JOINTS, -180.0)
        self.joint_limits_upper_deg = np.full(NUM_JOINTS, 180.0)

        # Visualisation helpers
        self._markers: dict = {}

        # Safety: last accepted IK solution (for joint-jump check)
        self._last_ik_deg: dict[str, np.ndarray | None] = {"left": None, "right": None}
        self._rest_deg: dict[str, np.ndarray | None] = {"left": None, "right": None}

        # Real xArm6 hardware (optional)
        self.real_robot: "BiXarm6Follower | None" = None
        self._real_last_cmd: dict[str, np.ndarray | None] = {"left": None, "right": None}

        self.is_connected = False

    # ── setup / teardown ────────────────────────────────────────────────

    def setup(self) -> bool:
        ctx = _suppress_stdout_stderr if self.suppress_output else contextlib.nullcontext

        # Connect
        mode = p.GUI if self.use_gui else p.DIRECT
        try:
            with ctx():
                self.physics_client = p.connect(mode)
        except p.error:
            try:
                with ctx():
                    self.physics_client = p.connect(p.DIRECT)
            except p.error:
                logger.error("Failed to connect to PyBullet")
                return False

        if self.physics_client < 0:
            return False

        if self.suppress_output and not self.use_gui:
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        with ctx():
            p.loadURDF("plane.urdf")

        if not os.path.exists(self.urdf_path):
            logger.error("URDF not found: %s", self.urdf_path)
            return False

        # Load left and right robots
        for arm, pos in [("left", [0.2, 0, 0]), ("right", [-0.2, 0, 0])]:
            try:
                with ctx():
                    self.robot_ids[arm] = p.loadURDF(
                        self.urdf_path, pos,
                        [0, 0, -0.7071, 0.7071],
                        useFixedBase=1,
                    )
            except p.error as e:
                logger.error("Failed to load %s URDF: %s", arm, e)
                return False

        if not self._map_joints():
            return False
        if not self._find_ee():
            return False
        self._read_limits()
        self._create_markers()
        self._setup_camera()
        for arm in self.robot_ids:
            self._close_gripper(arm)

        # Step the simulation and sync the GUI so everything is fully rendered
        p.stepSimulation()
        if self.use_gui:
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
            for _ in range(10):
                p.stepSimulation()
                time.sleep(0.01)

        # Connect and home real xArm6 arms if requested
        if self.connect_real:
            try:
                from openarm.damiao.xarm6_dual_robot import BiXarm6Follower
                self.real_robot = BiXarm6Follower()
                self.real_robot.connect()
                logger.info("Real xArm6 arms connected — homing...")
                self.real_robot.reset_to_rest_position()
                logger.info("Real xArm6 arms homed")
            except Exception as e:
                logger.error("Failed to connect real xArm6 arms: %s", e)
                self.real_robot = None

        self.is_connected = True
        return True

    def disconnect(self):
        if self.real_robot is not None:
            try:
                self.real_robot.disconnect()
            except Exception as e:
                logger.debug("Real arm disconnect error: %s", e)
            self.real_robot = None
        if self.is_connected and p.isConnected(self.physics_client):
            p.disconnect(self.physics_client)
            self.is_connected = False

    # ── FK / IK ─────────────────────────────────────────────────────────

    def get_joints(self, arm: str = "left") -> np.ndarray:
        """Read the current PyBullet joint angles in degrees (6,)."""
        rid = self.robot_ids[arm]
        out = np.zeros(NUM_JOINTS)
        for i in range(NUM_JOINTS):
            idx = self.joint_indices[arm][i]
            if idx is not None:
                out[i] = math.degrees(p.getJointState(rid, idx)[0])
        return out

    def _save_joints(self, arm: str) -> list[float]:
        rid = self.robot_ids[arm]
        return [
            p.getJointState(rid, self.joint_indices[arm][i])[0]
            if self.joint_indices[arm][i] is not None else 0.0
            for i in range(NUM_JOINTS)
        ]

    def _restore_joints(self, arm: str, saved: list[float]) -> None:
        rid = self.robot_ids[arm]
        for i in range(NUM_JOINTS):
            idx = self.joint_indices[arm][i]
            if idx is not None:
                p.resetJointState(rid, idx, saved[i])
        self._close_gripper(arm)

    def _close_gripper(self, arm: str) -> None:
        gidx = self._gripper_idx.get(arm)
        if gidx is not None:
            p.resetJointState(self.robot_ids[arm], gidx, 1.0)

    def fk(
        self,
        joint_angles_deg: "list[float] | np.ndarray",
        arm: str = "left",
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Forward kinematics.  Returns (position[3], quaternion_xyzw[4]).

        Saves and restores the visualised joint state so the sim is not disturbed.
        """
        saved = self._save_joints(arm)
        rid = self.robot_ids[arm]
        rad = np.deg2rad(joint_angles_deg)
        for i in range(NUM_JOINTS):
            idx = self.joint_indices[arm][i]
            if idx is not None:
                p.resetJointState(rid, idx, rad[i])
        ls = p.getLinkState(rid, self.ee_link_idx[arm])
        self._restore_joints(arm, saved)
        return np.array(ls[4]), np.array(ls[5])

    def ik(
        self,
        target_pos: np.ndarray,
        target_quat_xyzw: Optional[np.ndarray] = None,
        seed_deg: "list[float] | np.ndarray | None" = None,
        arm: str = "left",
    ) -> np.ndarray:
        """Inverse kinematics.  Returns joint angles in degrees (6,).

        Saves and restores the visualised joint state so the sim is not disturbed.
        """
        saved = self._save_joints(arm)
        rid = self.robot_ids[arm]
        lower = np.deg2rad(self.joint_limits_lower_deg)
        upper = np.deg2rad(self.joint_limits_upper_deg)
        ranges = upper - lower

        if seed_deg is not None:
            seed_rad = np.deg2rad(seed_deg[:NUM_JOINTS])
            for i in range(NUM_JOINTS):
                idx = self.joint_indices[arm][i]
                if idx is not None:
                    p.resetJointState(rid, idx, seed_rad[i])
            rest = seed_rad.tolist()
        else:
            rest = np.zeros(NUM_JOINTS).tolist()

        kwargs = dict(
            bodyUniqueId=rid,
            endEffectorLinkIndex=self.ee_link_idx[arm],
            targetPosition=target_pos.tolist(),
            lowerLimits=lower.tolist(),
            upperLimits=upper.tolist(),
            jointRanges=ranges.tolist(),
            restPoses=rest,
            solver=0,
            maxNumIterations=100,
            residualThreshold=1e-6,
        )
        if target_quat_xyzw is not None:
            kwargs["targetOrientation"] = target_quat_xyzw.tolist()

        # Iterative refinement: feed each solution back as a new seed
        for _ in range(10):
            sol = p.calculateInverseKinematics(**kwargs)
            for i in range(NUM_JOINTS):
                idx = self.joint_indices[arm][i]
                if idx is not None:
                    p.resetJointState(rid, idx, sol[i])

        self._restore_joints(arm, saved)
        sol_deg = np.rad2deg(sol[:NUM_JOINTS])
        sol_deg = np.clip(sol_deg, self.joint_limits_lower_deg, self.joint_limits_upper_deg)
        return sol_deg

    # ── Safe IK with error-state gating ───────────────────────────────

    IK_POS_ERR_THRESH = 0.03        # 30 mm — PyBullet IK is approximate
    IK_MAX_JOINT_JUMP_DEG = 45.0    # max per-joint change from last accepted solution (wrist joints are sensitive)

    def set_rest(self, arm: str, angles_deg: "list[float] | np.ndarray") -> None:
        """Store the home/rest joint angles for this arm.

        Used by ik_safe() as an alternative IK seed to prevent configuration
        drift away from the initial branch.
        """
        self._rest_deg[arm] = np.asarray(angles_deg, dtype=float)

    def _ik_with_fk_err(
        self,
        target_pos: np.ndarray,
        target_quat_xyzw: Optional[np.ndarray],
        seed_deg: "list[float] | np.ndarray | None",
        arm: str,
    ) -> Tuple[np.ndarray, float]:
        """Run IK and return (solution_deg, fk_position_error)."""
        sol = self.ik(target_pos, target_quat_xyzw, seed_deg=seed_deg, arm=arm)
        achieved, _ = self.fk(sol, arm=arm)
        return sol, float(np.linalg.norm(achieved - target_pos))

    def ik_safe(
        self,
        target_pos: np.ndarray,
        target_quat_xyzw: Optional[np.ndarray] = None,
        seed_deg: "list[float] | np.ndarray | None" = None,
        arm: str = "left",
    ) -> Optional[np.ndarray]:
        """IK with safety checks.  Returns joint angles (deg) or None if rejected.

        Tries IK from both the rest/home seed (to prevent configuration drift)
        and the tracking seed (for smooth motion).  Picks the solution with the
        lowest FK position error.

        On failure the arm keeps its last accepted joint state and the solver
        retries on the next call — no permanent error state.
        """
        candidates: list[Tuple[np.ndarray, float]] = []

        rest = self._rest_deg.get(arm)
        if rest is not None:
            candidates.append(self._ik_with_fk_err(
                target_pos, target_quat_xyzw, seed_deg=rest, arm=arm))

        if seed_deg is not None:
            candidates.append(self._ik_with_fk_err(
                target_pos, target_quat_xyzw, seed_deg=seed_deg, arm=arm))

        if not candidates:
            candidates.append(self._ik_with_fk_err(
                target_pos, target_quat_xyzw, seed_deg=None, arm=arm))

        valid = [(s, e) for s, e in candidates if e <= self.IK_POS_ERR_THRESH]

        if not valid:
            best_sol, best_err = min(candidates, key=lambda x: x[1])
            logger.debug(
                "xArm6 %s IK skip: FK err %.1fmm > %.0fmm — holding last position",
                arm, best_err * 1000, self.IK_POS_ERR_THRESH * 1000,
            )
            return None

        sol_deg = min(valid, key=lambda x: x[1])[0]

        prev = self._last_ik_deg[arm]
        if prev is not None:
            delta = np.abs(sol_deg - prev)
            worst_idx = int(np.argmax(delta))
            worst_deg = float(delta[worst_idx])
            if worst_deg > self.IK_MAX_JOINT_JUMP_DEG:
                logger.debug(
                    "xArm6 %s IK skip: j%d jumped %.1f° > %.1f° — holding last position",
                    arm, worst_idx + 1, worst_deg, self.IK_MAX_JOINT_JUMP_DEG,
                )
                return None

        self._last_ik_deg[arm] = sol_deg.copy()
        return sol_deg

    # ── Real arm output ──────────────────────────────────────────────────

    REAL_MAX_JOINT_STEP_DEG = 3.5   # max per-joint change per call sent to real hardware

    def send_to_real(self, joint_angles_deg: np.ndarray, gripper_pos: float | None = None, arm: str = "left") -> bool:
        """Send joint angles to the real xArm6, with per-joint rate limiting.

        Returns True if the command was sent, False if skipped (no real robot,
        error state, or would exceed the rate limit).
        """
        if self.real_robot is None:
            return False

        target = np.asarray(joint_angles_deg, dtype=float)
        prev = self._real_last_cmd[arm]

        if prev is not None:
            delta = target - prev
            max_abs = float(np.max(np.abs(delta)))
            if max_abs > self.REAL_MAX_JOINT_STEP_DEG:
                clamped = prev + np.clip(delta, -self.REAL_MAX_JOINT_STEP_DEG, self.REAL_MAX_JOINT_STEP_DEG)
                target = clamped
        else:
            # First command: read current real position and rate-limit from there
            try:
                if arm == "left":
                    cur_angles, _ = self.real_robot.get_left_angles()
                else:
                    cur_angles, _ = self.real_robot.get_right_angles()
                prev = np.array(cur_angles[:NUM_JOINTS], dtype=float)
                delta = target - prev
                max_abs = float(np.max(np.abs(delta)))
                if max_abs > self.REAL_MAX_JOINT_STEP_DEG:
                    target = prev + np.clip(delta, -self.REAL_MAX_JOINT_STEP_DEG, self.REAL_MAX_JOINT_STEP_DEG)
            except Exception:
                pass

        self._real_last_cmd[arm] = target.copy()
        try:
            self.real_robot.set_arm_joints(arm, target.tolist(), gripper_pos)
            return True
        except Exception as e:
            logger.error("Failed to send to real %s arm: %s", arm, e)
            return False

    # ── Visualisation convenience ───────────────────────────────────────

    def set_joints(
        self,
        joint_angles_deg: "list[float] | np.ndarray",
        arm: str = "left",
    ) -> None:
        """Set the visualised joint angles (degrees) for one arm."""
        if not self.is_connected:
            return
        rad = np.deg2rad(joint_angles_deg)
        rid = self.robot_ids[arm]
        for i in range(NUM_JOINTS):
            idx = self.joint_indices[arm][i]
            if idx is not None and i < len(rad):
                p.resetJointState(rid, idx, rad[i])
        self._close_gripper(arm)

    def set_marker(
        self,
        name: str,
        position: np.ndarray,
        orientation_xyzw: Optional[np.ndarray] = None,
    ) -> None:
        if not self.is_connected or name not in self._markers:
            return
        ori = orientation_xyzw if orientation_xyzw is not None else [0, 0, 0, 1]
        p.resetBasePositionAndOrientation(self._markers[name], position.tolist(), ori)

    def set_frame(
        self,
        name: str,
        position: np.ndarray,
        orientation_xyzw: Optional[np.ndarray] = None,
    ) -> None:
        if not self.is_connected or name not in self._markers:
            return
        lines = self._markers[name]
        if not isinstance(lines, list):
            return
        rot = R.from_quat(orientation_xyzw if orientation_xyzw is not None else [0, 0, 0, 1])
        mat = rot.as_matrix()
        colors = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
        for i in range(3):
            if i < len(lines):
                end = position + mat[:, i] * 0.05
                p.addUserDebugLine(
                    position.tolist(), end.tolist(),
                    lineColorRGB=colors[i], lineWidth=3,
                    replaceItemUniqueId=lines[i],
                )

    def hide_marker(self, name: str) -> None:
        if name in self._markers:
            self.set_marker(name, np.array([0, 0, -1]))

    def hide_frame(self, name: str) -> None:
        if name in self._markers and isinstance(self._markers[name], list):
            for lid in self._markers[name]:
                p.addUserDebugLine(
                    [0, 0, -1], [0, 0, -1],
                    lineColorRGB=[0, 0, 0], lineWidth=1,
                    replaceItemUniqueId=lid,
                )

    def step(self) -> None:
        if self.is_connected:
            p.stepSimulation()

    # ── Internal helpers ────────────────────────────────────────────────

    def _map_joints(self) -> bool:
        ok = True
        for arm, rid in self.robot_ids.items():
            name_to_idx: dict[str, int] = {}
            for i in range(p.getNumJoints(rid)):
                info = p.getJointInfo(rid, i)
                jname = info[1].decode("UTF-8")
                name_to_idx[jname] = i
                if info[2] != p.JOINT_FIXED:
                    p.setJointMotorControl2(rid, i, p.VELOCITY_CONTROL, force=0)
            mapped = 0
            for ji, jn in enumerate(JOINT_NAMES):
                if jn in name_to_idx:
                    self.joint_indices[arm][ji] = name_to_idx[jn]
                    mapped += 1
            if "drive_joint" in name_to_idx:
                self._gripper_idx[arm] = name_to_idx["drive_joint"]
            if mapped < NUM_JOINTS:
                missing = [n for i, n in enumerate(JOINT_NAMES) if self.joint_indices[arm][i] is None]
                logger.error("Missing joints for %s: %s", arm, missing)
                ok = False
        return ok

    def _find_ee(self) -> bool:
        ok = True
        for arm, rid in self.robot_ids.items():
            found = False
            for i in range(p.getNumJoints(rid)):
                info = p.getJointInfo(rid, i)
                if info[12].decode("UTF-8") == END_EFFECTOR_LINK_NAME:
                    self.ee_link_idx[arm] = i
                    found = True
                    break
            if not found:
                logger.error("EE link '%s' not found for %s", END_EFFECTOR_LINK_NAME, arm)
                ok = False
        return ok

    def _read_limits(self) -> None:
        rid = self.robot_ids["left"]
        for i in range(NUM_JOINTS):
            idx = self.joint_indices["left"][i]
            if idx is not None:
                info = p.getJointInfo(rid, idx)
                lo, hi = info[8], info[9]
                if lo < hi:
                    self.joint_limits_lower_deg[i] = math.degrees(lo)
                    self.joint_limits_upper_deg[i] = math.degrees(hi)

    def _create_markers(self) -> None:
        colors = {
            "left_target": [1, 0, 0, 0.8],
            "right_target": [0, 0, 1, 0.8],
            "left_goal": [0, 1, 0, 0.9],
            "right_goal": [1, 1, 0, 0.9],
        }
        for name, rgba in colors.items():
            shape = p.createVisualShape(p.GEOM_SPHERE, radius=0.02, rgbaColor=rgba)
            self._markers[name] = p.createMultiBody(baseVisualShapeIndex=shape, basePosition=[0, 0, -1])

        axis_colors = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
        for name in ["left_target_frame", "right_target_frame", "left_goal_frame", "right_goal_frame"]:
            lines = []
            for c in axis_colors:
                lines.append(p.addUserDebugLine([0, 0, -1], [0, 0, -1], lineColorRGB=c, lineWidth=3))
            self._markers[name] = lines

    def _setup_camera(self) -> None:
        p.resetDebugVisualizerCamera(
            cameraDistance=0.5,
            cameraYaw=160,
            cameraPitch=-30,
            cameraTargetPosition=[0.0, 0.0, 0.2],
        )
