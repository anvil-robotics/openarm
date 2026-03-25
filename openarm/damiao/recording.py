"""Recording and playback of joint-angle sequences for multi-arm teleoperation."""

from __future__ import annotations

import sys
import time
from typing import TYPE_CHECKING

from .config import FRAME_GAP, JOINT_GAINS, MOTOR_CONFIGS
from .encoding import (
    MitControlParams,
    decode_motor_state_sync,
    encode_control_mit,
)

if TYPE_CHECKING:
    from .gravity import GravityCompensator
    from .hardware import Arm


def save_recording(
    buf: list[dict[str, list[float]]],
    path: str,
    sides: list[str],
    n_joints: list[int],
) -> None:
    """Write recorded frames to a text file with a header."""
    with open(path, "w") as f:
        f.write("# " + " ".join(f"{s} {n}" for s, n in zip(sides, n_joints)) + "\n")
        for frame in buf:
            vals: list[float] = []
            for s in sides:
                vals.extend(frame.get(s, [0.0] * n_joints[sides.index(s)]))
            f.write(" ".join(f"{v:.6f}" for v in vals) + "\n")
    sys.stdout.write(f"\nRecording saved: {path} ({len(buf)} frames, arms: {' '.join(sides)})\n")


def load_recording(
    path: str,
) -> tuple[list[str], list[int], list[dict[str, list[float]]]]:
    """Load a recorded sequence. Returns (sides, n_joints_per_side, frames)."""
    sides: list[str] = []
    n_joints: list[int] = []
    frames: list[dict[str, list[float]]] = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.startswith("#"):
                parts = line[1:].split()
                for i in range(0, len(parts), 2):
                    sides.append(parts[i])
                    n_joints.append(int(parts[i + 1]))
                continue
            vals = [float(v) for v in line.split()]
            frame: dict[str, list[float]] = {}
            offset = 0
            for s, n in zip(sides, n_joints):
                frame[s] = vals[offset : offset + n]
                offset += n
            frames.append(frame)
    return sides, n_joints, frames


def run_playback(
    frames: list[dict[str, list[float]]],
    arm_map: dict[str, "Arm"],
    gravity_comp: "GravityCompensator | None" = None,
    slave_gravity_comp: "GravityCompensator | None" = None,
    home_dur: float = 3.0,
    playback_dt: float = 0.0125,
) -> None:
    """Home to first frame then replay the full sequence with gravity comp."""
    if not frames or not arm_map:
        return

    _gc = slave_gravity_comp or gravity_comp

    # Phase 1: Home to first frame
    first_frame = frames[0]
    sys.stdout.write(f"Homing {len(arm_map)} arm(s) to first frame over {home_dur}s ...\n")

    arm_start_q: dict[str, list[float]] = {}
    for side, slave in arm_map.items():
        for idx, motor in enumerate(slave.motors):
            if motor is None:
                continue
            try:
                zero = MitControlParams(q=0, dq=0, kp=0, kd=0, tau=0)
                encode_control_mit(motor._bus, motor._slave_id, motor._motor_limits, zero)
                time.sleep(FRAME_GAP)
                state = decode_motor_state_sync(motor._bus, motor._master_id, motor._motor_limits)
                if state is not None:
                    slave.states[idx] = state
            except Exception:
                pass
        arm_start_q[side] = slave.get_positions()

    home_hz = 200.0
    home_steps = int(home_dur * home_hz)
    for step in range(home_steps + 1):
        alpha = step / home_steps if home_steps > 0 else 1.0
        alpha = 3 * alpha**2 - 2 * alpha**3
        for side, slave in arm_map.items():
            target_q = first_frame.get(side, [])
            sq = arm_start_q[side]
            n = min(len(target_q), len(slave.motors))

            grav_torques: list[float] = []
            if _gc and side in ("left", "right"):
                grav_torques = _gc.compute(slave.get_positions(), position=side)

            for idx, motor in enumerate(slave.motors):
                if motor is None or idx >= n:
                    continue
                pos = sq[idx] + alpha * (target_q[idx] - sq[idx])
                if slave.mirror_mode and idx < len(MOTOR_CONFIGS) and MOTOR_CONFIGS[idx].inverted:
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
        if step % 100 == 0:
            sys.stdout.write(f"\r  homing {100*step//home_steps}%")
            sys.stdout.flush()
    sys.stdout.write("\r  homing 100% — done\n")

    # Phase 2: Replay
    sys.stdout.write(f"Playing {len(frames)} frames at {1/playback_dt:.0f} Hz ... (Ctrl+C to stop)\n")
    prev_frame: dict[str, list[float]] = {}
    try:
        for fi, frame in enumerate(frames):
            if prev_frame:
                skip = True
                for side in arm_map:
                    cur_vals = frame.get(side, [])
                    prv_vals = prev_frame.get(side, [])
                    if len(cur_vals) != len(prv_vals) or sum(abs(c - p) for c, p in zip(cur_vals, prv_vals)) >= 0.005:
                        skip = False
                        break
                if skip:
                    continue
            prev_frame = frame
            t_frame_start = time.time()
            for side, slave in arm_map.items():
                joint_vals = frame.get(side, [])
                n = min(len(joint_vals), len(slave.motors))

                grav_torques_: list[float] = []
                if _gc and side in ("left", "right"):
                    grav_torques_ = _gc.compute(slave.get_positions(), position=side)

                for idx, motor in enumerate(slave.motors):
                    if motor is None or idx >= n:
                        continue
                    pos = joint_vals[idx]
                    if slave.mirror_mode and idx < len(MOTOR_CONFIGS) and MOTOR_CONFIGS[idx].inverted:
                        pos = -pos
                    kp, kd = JOINT_GAINS[idx] if idx < len(JOINT_GAINS) else (2.0, 1.0)
                    torque = grav_torques_[idx] if idx < len(grav_torques_) else 0.0
                    params = MitControlParams(q=pos, dq=0, kp=kp, kd=kd, tau=torque)
                    try:
                        encode_control_mit(motor._bus, motor._slave_id, motor._motor_limits, params)
                        time.sleep(FRAME_GAP)
                        state = decode_motor_state_sync(motor._bus, motor._master_id, motor._motor_limits)
                        if state is not None:
                            slave.states[idx] = state
                    except Exception:
                        pass
            elapsed = time.time() - t_frame_start
            remaining = playback_dt - elapsed
            if remaining > 0:
                time.sleep(remaining)
            if fi % 100 == 0:
                sys.stdout.write(f"\r  frame {fi}/{len(frames)}")
                sys.stdout.flush()
        sys.stdout.write(f"\r  frame {len(frames)}/{len(frames)} — done\n")
    except KeyboardInterrupt:
        sys.stdout.write("\nPlayback interrupted.\n")
