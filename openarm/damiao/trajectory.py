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
    from .hardware import Arm


async def home_all_arms(
    arms: list["Arm"],
    *,
    skip_first: bool = False,
) -> None:
    """Move all arms through HOME_WAYPOINTS using execute_waypoint."""
    waypoints = HOME_WAYPOINTS[1:] if skip_first and len(HOME_WAYPOINTS) > 1 else HOME_WAYPOINTS
    total_dur = sum(wp[1] for wp in waypoints)
    sys.stdout.write(f"\nHoming all arms ({len(waypoints)} waypoints, {total_dur:.1f}s total) ...\n")

    masters = [a for a in arms if a.is_master]
    slave_by_follows: dict[str, "Arm"] = {a.follows: a for a in arms if a.is_slave and a.follows}
    paired_slave_ids = {id(a) for a in slave_by_follows.values()}
    standalone = [a for a in arms if not a.is_master and id(a) not in paired_slave_ids]

    tasks = []
    for master in masters:
        slave = slave_by_follows.get(master.channel)
        tasks.append(execute_waypoint(waypoints, master, slave, hold_time=0.0))
    for arm in standalone:
        tasks.append(execute_waypoint(waypoints, arm, None, hold_time=0.0))
    await asyncio.gather(*tasks)

    sys.stdout.write("\r  homing done.                    \n")


async def execute_waypoint(
    waypoints: list[tuple],
    master_arm: "Arm",
    slave_arm: "Arm | None",
    gravity_comp: "GravityCompensator | None" = None,
    slave_gravity_comp: "GravityCompensator | None" = None,
    hz: float = 200.0,
    hold_time: float = 0.25,
) -> None:
    """Move through a sequence of waypoints with smooth cubic blending.

    Each segment uses a cubic ease-in-out (smoothstep) from the previous waypoint
    to the next. A hold phase at the final waypoint is appended automatically.

    Waypoints are tuples of (joint_angles, duration) or (joint_angles, duration, gain_scale).
    When gain_scale is omitted it defaults to 1.0. The gain_scale is interpolated
    between segments the same way joint angles are, allowing smooth transitions
    from soft to stiff control.

    Uses wall-clock time for interpolation so the trajectory never runs ahead of
    the actual command rate. Master and slave are commanded concurrently on
    separate threads (they live on different CAN buses).
    """
    # Zero-torque read to get fresh positions before starting
    for idx, motor in enumerate(master_arm.motors):
        if motor is None:
            continue
        try:
            zero = MitControlParams(q=0, dq=0, kp=0, kd=0, tau=0)
            encode_control_mit(motor._bus, motor._slave_id, motor._motor_limits, zero)
            time.sleep(FRAME_GAP)
            state = decode_motor_state_sync(motor._bus, motor._master_id, motor._motor_limits)
            if state is not None:
                master_arm.states[idx] = state
        except Exception:
            pass

    start_q = master_arm.get_positions()

    all_q = [start_q]
    seg_durations = []
    seg_gains = [1.0]  # gain for the start position (before first waypoint)
    for wp in waypoints:
        q_target, dur = wp[0], wp[1]
        gs = wp[2] if len(wp) > 2 else 1.0
        n = min(len(q_target), len(start_q))
        all_q.append(list(q_target[:n]) + start_q[n:])
        seg_durations.append(dur)
        seg_gains.append(gs)

    cum_times = [0.0]
    for d in seg_durations:
        cum_times.append(cum_times[-1] + d)
    total_move_time = cum_times[-1]
    total_time = total_move_time + hold_time

    n_joints = len(start_q)
    final_q = all_q[-1]
    final_gain = seg_gains[-1]
    dt = 1.0 / hz

    def _interp_at(t: float) -> tuple[list[float], float]:
        """Returns (interpolated joint angles, interpolated gain_scale)."""
        if t >= total_move_time:
            return list(final_q), final_gain
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
        gs = seg_gains[seg + 1]
        return [s + alpha * (e - s) for s, e in zip(q_from, q_to)], gs

    def _cmd_master(interp_q: list[float], grav: list[float], gain_scale: float) -> list:
        results = [None] * len(master_arm.motors)
        for idx, motor in enumerate(master_arm.motors):
            if motor is None or idx >= n_joints:
                continue
            kp, kd = JOINT_GAINS[idx] if idx < len(JOINT_GAINS) else (2.0, 1.0)
            kp *= gain_scale
            kd *= gain_scale
            torque = grav[idx] if idx < len(grav) else 0.0
            params = MitControlParams(
                q=interp_q[idx], dq=0, kp=kp, kd=kd, tau=torque,
            )
            try:
                encode_control_mit(motor._bus, motor._slave_id, motor._motor_limits, params)
                time.sleep(FRAME_GAP)
                state = decode_motor_state_sync(motor._bus, motor._master_id, motor._motor_limits)
                results[idx] = state
            except Exception:
                pass
        return results

    def _cmd_slave(interp_q: list[float], grav: list[float], gain_scale: float) -> list:
        results = [None] * len(slave_arm.motors)
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
            kp *= gain_scale
            kd *= gain_scale
            torque = grav[idx] if idx < len(grav) else 0.0
            params = MitControlParams(
                q=pos, dq=0, kp=kp, kd=kd, tau=torque,
            )
            try:
                encode_control_mit(slave_motor._bus, slave_motor._slave_id, slave_motor._motor_limits, params)
                time.sleep(FRAME_GAP)
                state = decode_motor_state_sync(slave_motor._bus, slave_motor._master_id, slave_motor._motor_limits)
                results[idx] = state
            except Exception:
                pass
        return results

    loop = asyncio.get_event_loop()
    t0 = time.perf_counter()
    step = 0

    while True:
        iter_start = time.perf_counter()
        t = iter_start - t0

        if t >= total_time:
            break

        interp_q, gs = _interp_at(t)

        master_grav: list[float] = []
        if gravity_comp and master_arm.position in ("left", "right"):
            master_grav = gravity_comp.compute(master_arm.get_positions(), position=master_arm.position)

        slave_grav: list[float] = []
        _s_grav_comp = slave_gravity_comp or gravity_comp
        if _s_grav_comp and slave_arm is not None and slave_arm.position in ("left", "right"):
            slave_grav = _s_grav_comp.compute(slave_arm.get_positions(), position=slave_arm.position)

        tasks = [loop.run_in_executor(None, _cmd_master, interp_q, master_grav, gs)]
        if slave_arm is not None:
            tasks.append(loop.run_in_executor(None, _cmd_slave, interp_q, slave_grav, gs))

        results = await asyncio.gather(*tasks)

        master_arm.states = results[0]
        if slave_arm is not None and len(results) > 1:
            slave_arm.states = results[1]

        if (step % 50 == 0) and slave_arm is not None and gravity_comp is not None:
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
                f"\r  t={t:.2f}/{total_time:.1f}s  "
                f"pos_err: dx={pos_err[0]*1000:.1f} dy={pos_err[1]*1000:.1f} dz={pos_err[2]*1000:.1f} mm  "
                f"ori_err: r={np.degrees(ori_err[0]):.1f} p={np.degrees(ori_err[1]):.1f} y={np.degrees(ori_err[2]):.1f} deg  "
                f"q_err(deg): {joint_err_str}"
                f"\033[K\r\n"
            )
            sys.stdout.flush()

        step += 1
        elapsed = time.perf_counter() - iter_start
        remaining = dt - elapsed
        if remaining > 0:
            await asyncio.sleep(remaining)
