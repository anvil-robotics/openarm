"""
Bimanual xArm6 follower driver.

Manages two xArm6 arms over the xArm SDK, providing parallel
joint-angle + gripper commands, state readback, and a safe
reset-to-home routine.
"""

from __future__ import annotations

import copy
import logging
import threading
import time
from typing import Any

import numpy as np
from xarm.wrapper import XArmAPI

logger = logging.getLogger(__name__)

# ── Configuration (edit these to match your setup) ──────────────────────

ARM_IPS: list[str] = ["192.168.1.11", "192.168.1.223"]
USE_ARM: list[bool] = [True, True]
USE_FT_SENSOR: list[bool] = [False, False]
GRIPPER_SPEED: int = 5000

HOME_ANGLES_LEFT: list[float] = [0,-60,-30,0,0,-90]   # degrees
HOME_ANGLES_RIGHT: list[float] = [0,-60,-30,0,0,0]  # degrees
HOME_GRIPPER_POS: float = 100
HOME_SPEED: float = 30  # deg/s


class BiXarm6Follower:
    """Bimanual xArm6 follower — manages two xArm6 arms."""

    def __init__(self) -> None:
        self._arms: list[XArmAPI | None] = [None, None]
        self._is_connected = False

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    # ── Connection ──────────────────────────────────────────────────────

    def _connect_arm(self, idx: int) -> None:
        arm = XArmAPI(ARM_IPS[idx], is_radian=True)
        arm.motion_enable(enable=True)
        arm.clean_error()
        arm.set_mode(1)
        arm.set_state(state=0)
        time.sleep(1)
        arm.set_gripper_mode(0)
        arm.set_gripper_enable(True)
        arm.set_gripper_speed(GRIPPER_SPEED)
        self._arms[idx] = arm

    def connect(self) -> None:
        if self._is_connected:
            raise RuntimeError("Already connected")

        for i in range(2):
            if not USE_ARM[i]:
                continue
            self._connect_arm(i)

            code, _ = self._arms[i].get_servo_angle()
            if code != 0:
                name = "left" if i == 0 else "right"
                raise RuntimeError(f"Failed to read joint angles from {name} arm (code {code})")

        self._is_connected = True
        logger.info("BiXarm6Follower connected")

    # ── State readback ──────────────────────────────────────────────────

    def get_left_angles(self) -> tuple[list[float], float]:
        """Return (joint_angles_deg[6], gripper_pos) for left arm."""
        code, angles = self._arms[0].get_servo_angle(is_radian=False)
        gcode, gpos = self._arms[0].get_gripper_position()
        return angles, gpos

    def get_right_angles(self) -> tuple[list[float], float]:
        """Return (joint_angles_deg[6], gripper_pos) for right arm."""
        code, angles = self._arms[1].get_servo_angle(is_radian=False)
        gcode, gpos = self._arms[1].get_gripper_position()
        return angles, gpos

    def get_angles(self) -> tuple[tuple[list[float], float], tuple[list[float], float]]:
        """Read both arms in parallel.  Returns (left, right) tuples."""
        left_result: list[Any] = [None, None]
        right_result: list[Any] = [None, None]

        def _read_left():
            left_result[0], left_result[1] = self.get_left_angles()

        def _read_right():
            right_result[0], right_result[1] = self.get_right_angles()

        threads = []
        if USE_ARM[0] and self._arms[0] is not None:
            t = threading.Thread(target=_read_left)
            t.start()
            threads.append(t)
        if USE_ARM[1] and self._arms[1] is not None:
            t = threading.Thread(target=_read_right)
            t.start()
            threads.append(t)
        for t in threads:
            t.join()

        return tuple(left_result), tuple(right_result)

    # ── Command ─────────────────────────────────────────────────────────

    def set_arm_joints(self, arm: str, angles_deg: list[float], gripper_pos: float | None = None) -> None:
        """Set joint angles (degrees, 6 values) for a single arm. Non-blocking."""
        idx = 0 if arm == "left" else 1
        if not USE_ARM[idx] or self._arms[idx] is None:
            return
        ret = self._arms[idx].set_servo_angle_j(angles_deg[:6], wait=False, is_radian=False)
        if ret != 0:
            logger.error("%s arm set_servo_angle_j failed (%d), reconnecting", arm, ret)
            self._arms[idx].disconnect()
            self._connect_arm(idx)
            time.sleep(1)
        if gripper_pos is not None:
            self._arms[idx].set_gripper_position(gripper_pos, wait=False)

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        """Command both arms to target joint configs (degrees) + gripper.

        Expected keys::

            left_joint1.pos … left_joint6.pos, left_gripper.pos
            right_joint1.pos … right_joint6.pos, right_gripper.pos
        """
        if not self._is_connected:
            raise RuntimeError("Not connected")

        left_goal = [action[f"left_joint{i}.pos"] for i in range(1, 7)]
        left_goal.append(action["left_gripper.pos"])

        right_goal = [action[f"right_joint{i}.pos"] for i in range(1, 7)]
        right_goal.append(action["right_gripper.pos"])

        def _set_left():
            ret = self._arms[0].set_servo_angle_j(left_goal[:6], wait=False, is_radian=False)
            if ret != 0:
                logger.error("Left arm set_servo_angle_j failed (%d), reconnecting", ret)
                self._arms[0].disconnect()
                self._connect_arm(0)
                time.sleep(1)
            self._arms[0].set_gripper_position(left_goal[6], wait=False)

        def _set_right():
            ret = self._arms[1].set_servo_angle_j(right_goal[:6], wait=False, is_radian=False)
            if ret != 0:
                logger.error("Right arm set_servo_angle_j failed (%d), reconnecting", ret)
                self._arms[1].disconnect()
                self._connect_arm(1)
                time.sleep(1)
            self._arms[1].set_gripper_position(right_goal[6], wait=False)

        threads = []
        if USE_ARM[0] and self._arms[0] is not None:
            t = threading.Thread(target=_set_left)
            t.start()
            threads.append(t)
        if USE_ARM[1] and self._arms[1] is not None:
            t = threading.Thread(target=_set_right)
            t.start()
            threads.append(t)
        for t in threads:
            t.join()

        return copy.deepcopy(action)

    # ── Home / reset ────────────────────────────────────────────────────

    def reset_to_rest_position(self) -> None:
        """Move both arms to their home position (blocking)."""
        if not self._is_connected:
            raise RuntimeError("Not connected")

        home_angles = [HOME_ANGLES_LEFT, HOME_ANGLES_RIGHT]
        for i in range(2):
            if not USE_ARM[i] or self._arms[i] is None:
                continue
            arm = self._arms[i]
            arm.set_mode(0)
            arm.set_state(state=0)
            time.sleep(1)
            ret = arm.set_servo_angle(angle=home_angles[i], speed=HOME_SPEED, wait=True, is_radian=False)
            if ret != 0:
                name = "left" if i == 0 else "right"
                logger.error("Failed to home %s arm (code %d)", name, ret)
            arm.set_gripper_position(HOME_GRIPPER_POS, wait=True)
            arm.set_mode(1)
            arm.set_state(state=0)
            time.sleep(1)

    # ── Disconnect ──────────────────────────────────────────────────────

    def disconnect(self) -> None:
        if not self._is_connected:
            return

        for i, arm in enumerate(self._arms):
            if arm is None:
                continue
            if USE_FT_SENSOR[i]:
                arm.set_ft_sensor_mode(0)
                arm.set_ft_sensor_enable(0)
            arm.disconnect()

        self._arms = [None, None]
        self._is_connected = False
        logger.info("BiXarm6Follower disconnected")
