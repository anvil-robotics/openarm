"""Trajectory execution helpers: homing and waypoint following."""

from __future__ import annotations

import asyncio
import sys
import time
from typing import TYPE_CHECKING

import mujoco
import numpy as np

from .config import FRAME_GAP, HOME_WAYPOINTS, JOINT_GAINS, MOTOR_CONFIGS
from .encoding import (
    MitControlParams,
    decode_motor_state_sync,
    encode_control_mit,
)

if TYPE_CHECKING:
    from .gravity import GravityCompensator
    from .monitor import Arm


async def home_all_arms(
    arms: list["Arm"],
    *,
    skip_first: bool = False,
) -> None:
    """Move all arms through HOME_WAYPOINTS with fresh motor reads and smoothstep."""
    waypoints = HOME_WAYPOINTS[1:] if skip_first and len(HOME_WAYPOINTS) > 1 else HOME_WAYPOINTS
    home_hz = 200.0
    home_dt = 1.0 / home_hz
    n_home_joints = min(
        len(MOTOR_CONFIGS),
        min(len(q) for q, _ in waypoints) if waypoints else len(MOTOR_CONFIGS),
    )
    total_home_dur = sum(d for _, d in waypoints)
    sys.stdout.write(f"\nHoming all arms ({len(waypoints)} waypoints, {total_home_dur:.1f}s total) ...\n")

    for wp_idx, (wp_q, wp_dur) in enumerate(waypoints):
        for arm in arms:
            for idx, motor in enumerate(arm.motors):
                if motor is None:
                    continue
                try:
                    zero = MitControlParams(q=0, dq=0, kp=0, kd=0, tau=0)
                    encode_control_mit(motor._bus, motor._slave_id, motor._motor_limits, zero)
                    time.sleep(FRAME_GAP)
                    state = decode_motor_state_sync(
                        motor._bus, motor._master_id, motor._motor_limits,
                    )
                    if state is not None:
                        arm.states[idx] = state
                except Exception:
                    pass

        arm_start_q: dict[str, list[float]] = {}
        for arm in arms:
            cur = []
            for motor, state in zip(arm.motors, arm.states):
                cur.append(state.position if motor is not None and state is not None else 0.0)
            arm_start_q[arm.channel] = cur

        wp_steps = int(wp_dur * home_hz)
        for step in range(wp_steps + 1):
            alpha = step / wp_steps if wp_steps > 0 else 1.0
            alpha = 3 * alpha**2 - 2 * alpha**3

            for arm in arms:
                start_q = arm_start_q[arm.channel]
                mirror = arm.is_slave and arm.mirror_mode
                for idx, motor in enumerate(arm.motors):
                    if motor is None or idx >= n_home_joints:
                        continue
                    target = wp_q[idx]
                    pos = start_q[idx] + alpha * (target - start_q[idx])
                    if mirror and MOTOR_CONFIGS[idx].inverted:
                        pos = -pos
                    kp, kd = JOINT_GAINS[idx] if idx < len(JOINT_GAINS) else (2.0, 1.0)
                    if wp_idx == 0 and not skip_first:
                        kp *= 0.1
                        kd *= 0.1
                    params = MitControlParams(q=pos, dq=0, kp=kp, kd=kd, tau=0)
                    try:
                        encode_control_mit(motor._bus, motor._slave_id, motor._motor_limits, params)
                        time.sleep(FRAME_GAP)
                        state = decode_motor_state_sync(motor._bus, motor._master_id, motor._motor_limits)
                        if state is not None:
                            arm.states[idx] = state
                    except Exception:
                        pass

            if step % 100 == 0 or step == wp_steps:
                sys.stdout.write(f"\r  waypoint {wp_idx + 1}/{len(HOME_WAYPOINTS)}  {100.0 * step / wp_steps if wp_steps else 100:.0f}%")
                sys.stdout.flush()

            await asyncio.sleep(home_dt)

    sys.stdout.write("\r  homing done.                    \n")


async def execute_waypoint(
    waypoints: list[tuple[list[float], float]],
    master_arm: "Arm",
    slave_arm: "Arm | None",
    gravity_comp: "GravityCompensator | None" = None,
    slave_gravity_comp: "GravityCompensator | None" = None,
    hz: float = 200.0,
    hold_time: float = 0.25,
) -> None:
    """Move through a sequence of (joint_angles, duration) waypoints with smooth cubic blending.

    Each segment uses a cubic ease-in-out (smoothstep) from the previous waypoint
    to the next. A hold phase at the final waypoint is appended automatically.
    """
    start_q = master_arm.get_positions()

    all_q = [start_q]
    seg_durations = []
    for q_target, dur in waypoints:
        n = min(len(q_target), len(start_q))
        all_q.append(list(q_target[:n]) + start_q[n:])
        seg_durations.append(dur)

    cum_times = [0.0]
    for d in seg_durations:
        cum_times.append(cum_times[-1] + d)
    total_move_time = cum_times[-1]
    total_time = total_move_time + hold_time

    n_joints = len(start_q)
    final_q = all_q[-1]
    dt = 1.0 / hz
    steps = int(total_time * hz)

    for step in range(steps + 1):
        t = step * dt

        if t >= total_move_time:
            interp_q = list(final_q)
        else:
            seg = 0
            for si in range(len(seg_durations)):
                if t < cum_times[si + 1]:
                    seg = si
                    break
            else:
                seg = len(seg_durations) - 1

            seg_start = cum_times[seg]
            seg_dur = seg_durations[seg]
            alpha = (t - seg_start) / seg_dur if seg_dur > 0 else 1.0
            alpha = min(max(alpha, 0.0), 1.0)
            alpha = 3 * alpha**2 - 2 * alpha**3

            q_from = all_q[seg]
            q_to = all_q[seg + 1]
            interp_q = [
                s + alpha * (e - s) for s, e in zip(q_from, q_to)
            ]

        # Gravity compensation torques
        master_grav: list[float] = []
        if gravity_comp and master_arm.position in ("left", "right"):
            master_grav = gravity_comp.compute(master_arm.get_positions(), position=master_arm.position)

        slave_grav: list[float] = []
        _s_grav_comp = slave_gravity_comp or gravity_comp
        if _s_grav_comp and slave_arm is not None and slave_arm.position in ("left", "right"):
            slave_grav = _s_grav_comp.compute(slave_arm.get_positions(), position=slave_arm.position)

        # Command master arm
        for idx, motor in enumerate(master_arm.motors):
            if motor is None or idx >= n_joints:
                continue
            kp, kd = JOINT_GAINS[idx] if idx < len(JOINT_GAINS) else (2.0, 1.0)
            torque = master_grav[idx] if idx < len(master_grav) else 0.0
            params = MitControlParams(
                q=interp_q[idx], dq=0, kp=kp, kd=kd, tau=torque,
            )
            try:
                encode_control_mit(motor._bus, motor._slave_id, motor._motor_limits, params)
                time.sleep(FRAME_GAP)
                state = decode_motor_state_sync(motor._bus, motor._master_id, motor._motor_limits)
                if state is not None:
                    master_arm.states[idx] = state
            except Exception:
                pass

        # Command slave arm (mirror if needed)
        if slave_arm is not None:
            for idx, slave_motor in enumerate(slave_arm.motors):
                if slave_motor is None or idx >= n_joints:
                    continue
                pos = interp_q[idx]
                if (
                    slave_arm.mirror_mode
                    and idx < len(MOTOR_CONFIGS)
                    and MOTOR_CONFIGS[idx].inverted
                ):
                    pos = -pos
                kp, kd = JOINT_GAINS[idx] if idx < len(JOINT_GAINS) else (2.0, 1.0)
                torque = slave_grav[idx] if idx < len(slave_grav) else 0.0
                params = MitControlParams(
                    q=pos, dq=0, kp=kp, kd=kd, tau=torque,
                )
                try:
                    encode_control_mit(slave_motor._bus, slave_motor._slave_id, slave_motor._motor_limits, params)
                    time.sleep(FRAME_GAP)
                    state = decode_motor_state_sync(slave_motor._bus, slave_motor._master_id, slave_motor._motor_limits)
                    if state is not None:
                        slave_arm.states[idx] = state
                except Exception:
                    pass

        # Print tracking error every 50 steps and on the last step
        if (step % 50 == 0 or step == steps) and slave_arm is not None and gravity_comp is not None:
            actual_q = slave_arm.get_positions()
            actual_pos, actual_quat = gravity_comp.forward_kinematics(actual_q[:7], position=slave_arm.position)
            desired_pos, desired_quat = gravity_comp.forward_kinematics(final_q[:7], position=slave_arm.position)
            pos_err = desired_pos - actual_pos
            ori_err = np.zeros(3)
            tgt_q = desired_quat.copy()
            if np.dot(tgt_q, actual_quat) < 0:
                tgt_q = -tgt_q
            mujoco.mju_subQuat(ori_err, tgt_q, actual_quat)
            joint_err_deg = [np.degrees(f - a) for f, a in zip(final_q[:n_joints], actual_q[:n_joints])]
            joint_err_str = " ".join(f"j{i}={e:.2f}" for i, e in enumerate(joint_err_deg))
            sys.stdout.write(
                f"\r  step {step}/{steps}  "
                f"pos_err: dx={pos_err[0]*1000:.1f} dy={pos_err[1]*1000:.1f} dz={pos_err[2]*1000:.1f} mm  "
                f"ori_err: r={np.degrees(ori_err[0]):.1f} p={np.degrees(ori_err[1]):.1f} y={np.degrees(ori_err[2]):.1f} deg  "
                f"q_err(deg): {joint_err_str}"
                f"\033[K\r\n"
            )
            sys.stdout.flush()

        await asyncio.sleep(dt)
