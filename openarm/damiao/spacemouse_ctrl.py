"""SpaceMouse input via spacenavd + libspnav (ctypes, no PyPI spnav).

Background thread polls motion and button events. Safe to use alongside
other spnav clients (daemon multiplexes).
"""

from __future__ import annotations

import ctypes
import threading
import time
from ctypes import Structure, Union, pointer
from dataclasses import dataclass, field

import numpy as np

# --- libspnav ctypes (same as spacemouse_test.py) ---------------------------------

try:
    _lib = ctypes.CDLL("libspnav.so")
    _HAVE_SPNAV = True
except OSError:
    _lib = None
    _HAVE_SPNAV = False

SPNAV_EVENT_MOTION = 1
SPNAV_EVENT_BUTTON = 2
SPNAV_SCALE = 350.0

# Velocity caps at full stick deflection (before teleop gains)
DEFAULT_LINEAR_CAP = 0.18  # m/s
DEFAULT_ANGULAR_CAP = 0.55  # rad/s
DEFAULT_DEADBAND = 0.06
GAIN_STEP = 1.12
GAIN_TRANS_MIN = 0.02
GAIN_TRANS_MAX = 3.0
GAIN_ROT_MIN = 0.02
GAIN_ROT_MAX = 3.0
DEFAULT_GAIN_TRANS = 0.1
DEFAULT_GAIN_ROT = 0.1


class _MotionStruct(Structure):
    _fields_ = [
        ("type", ctypes.c_int),
        ("x", ctypes.c_int),
        ("y", ctypes.c_int),
        ("z", ctypes.c_int),
        ("rx", ctypes.c_int),
        ("ry", ctypes.c_int),
        ("rz", ctypes.c_int),
        ("period", ctypes.c_uint),
        ("data", ctypes.c_void_p),
    ]


class _ButtonStruct(Structure):
    _fields_ = [
        ("type", ctypes.c_int),
        ("press", ctypes.c_int),
        ("bnum", ctypes.c_int),
    ]


class _SpnavEvent(Union):
    _fields_ = [("type", ctypes.c_int), ("motion", _MotionStruct), ("button", _ButtonStruct)]


if _HAVE_SPNAV:
    _lib.spnav_open.argtypes = []
    _lib.spnav_open.restype = ctypes.c_int
    _lib.spnav_close.argtypes = []
    _lib.spnav_close.restype = None
    _lib.spnav_poll_event.argtypes = [ctypes.POINTER(_SpnavEvent)]
    _lib.spnav_poll_event.restype = ctypes.c_int


def _poll_event() -> tuple[str, object] | None:
    if not _HAVE_SPNAV:
        return None
    ev = _SpnavEvent()
    if _lib.spnav_poll_event(pointer(ev)) == 0:
        return None
    if ev.type == SPNAV_EVENT_MOTION:
        m = ev.motion
        return ("motion", (m.x, m.y, m.z, m.rx, m.ry, m.rz))
    if ev.type == SPNAV_EVENT_BUTTON:
        b = ev.button
        return ("button", (b.bnum, bool(b.press)))
    return None


@dataclass
class SpacemouseSample:
    """Latest normalized stick (-1..1) and button states."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    buttons: tuple[int, ...] = field(default_factory=lambda: (0, 0))


class SpacemouseReader:
    """Poll spacenavd in a daemon thread."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._sample = SpacemouseSample()
        self._connected = False
        self._error: str | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> bool:
        if not _HAVE_SPNAV:
            self._error = "libspnav.so not found (apt install libspnav0)"
            return False
        if self._thread is not None and self._thread.is_alive():
            return True
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        return True

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.5)
            self._thread = None
        if _HAVE_SPNAV:
            try:
                _lib.spnav_close()
            except Exception:
                pass
        with self._lock:
            self._connected = False

    def _loop(self) -> None:
        try:
            if _lib.spnav_open() == -1:
                with self._lock:
                    self._error = "spnav_open failed (is spacenavd running?)"
                return
            with self._lock:
                self._connected = True
                self._error = None
            btn = [0, 0]
            while not self._stop.is_set():
                ev = _poll_event()
                if ev is not None:
                    kind, data = ev
                    if kind == "motion":
                        x, y, z, rx, ry, rz = data
                        with self._lock:
                            self._sample.x = x / SPNAV_SCALE
                            self._sample.y = y / SPNAV_SCALE
                            self._sample.z = z / SPNAV_SCALE
                            self._sample.roll = rx / SPNAV_SCALE
                            self._sample.pitch = ry / SPNAV_SCALE
                            self._sample.yaw = rz / SPNAV_SCALE
                    elif kind == "button":
                        bnum, press = data
                        if bnum < len(btn):
                            btn[bnum] = 1 if press else 0
                        with self._lock:
                            self._sample.buttons = tuple(btn)
                time.sleep(0.002)
        except Exception as e:
            with self._lock:
                self._error = str(e)
                self._connected = False
        finally:
            try:
                _lib.spnav_close()
            except Exception:
                pass
            with self._lock:
                self._connected = False

    def get_sample(self) -> SpacemouseSample:
        with self._lock:
            return SpacemouseSample(
                self._sample.x,
                self._sample.y,
                self._sample.z,
                self._sample.roll,
                self._sample.pitch,
                self._sample.yaw,
                self._sample.buttons,
            )

    @property
    def connected(self) -> bool:
        with self._lock:
            return self._connected

    @property
    def error(self) -> str | None:
        with self._lock:
            return self._error


def deadband(v: float, band: float = DEFAULT_DEADBAND) -> float:
    return 0.0 if abs(v) < band else v


def translation_world_from_spacemouse(sample: SpacemouseSample) -> tuple[float, float, float]:
    """Map puck translation (spnav x,y,z) to world linear twist components [vx,vy,vz].

    Permutation: **z→x**, **x→y**, **y→z**; **world y and z** use flipped signs vs raw puck
    so GUI +y/+z match push direction on your setup.
    """
    sx = deadband(sample.x)
    sy = deadband(sample.y)
    sz = deadband(sample.z)
    return (sz, -sx, sy)


def angular_world_from_spacemouse(sample: SpacemouseSample) -> tuple[float, float, float]:
    """Map puck rotation (spnav rx,ry,rz → roll,pitch,yaw) to world angular twist [wx,wy,wz].

    Same permutation as translation (**z→x**, **x→y**, **y→z**) with matching sign flips
    on the world **y** and **z** angular slots.
    """
    sr = deadband(sample.roll)
    sp = deadband(sample.pitch)
    syaw = deadband(sample.yaw)
    return (syaw, -sr, sp)


def normalized_effective_axes(
    sample: SpacemouseSample,
    axis_on: list[bool],
    gain_trans: float,
    gain_rot: float,
) -> list[float]:
    """Per-axis display values ~[-1,1] scaled by gains (matches spacemouse_test plots)."""
    tx, ty, tz = translation_world_from_spacemouse(sample)
    wx, wy, wz = angular_world_from_spacemouse(sample)
    raw = [tx, ty, tz, wx, wy, wz]
    out: list[float] = []
    for i, v in enumerate(raw):
        if not axis_on[i]:
            out.append(0.0)
        elif i < 3:
            out.append(v * gain_trans)
        else:
            out.append(v * gain_rot)
    return out


def spacemouse_to_twist(
    sample: SpacemouseSample,
    axis_on: list[bool],
    gain_trans: float,
    gain_rot: float,
    lin_cap: float = DEFAULT_LINEAR_CAP,
    ang_cap: float = DEFAULT_ANGULAR_CAP,
) -> np.ndarray:
    """Build 6D spatial velocity [vx,vy,vz, wx,wy,wz] in world frame (MuJoCo Jacobian)."""
    tx, ty, tz = translation_world_from_spacemouse(sample)
    wx, wy, wz = angular_world_from_spacemouse(sample)
    raw = np.array([tx, ty, tz, wx, wy, wz], dtype=np.float64)
    for i in range(6):
        if not axis_on[i]:
            raw[i] = 0.0
    v_lin = raw[:3] * gain_trans * lin_cap
    v_ang = raw[3:] * gain_rot * ang_cap
    nlin = np.linalg.norm(v_lin)
    if nlin > lin_cap:
        v_lin *= lin_cap / nlin
    nang = np.linalg.norm(v_ang)
    if nang > ang_cap:
        v_ang *= ang_cap / nang
    return np.concatenate([v_lin, v_ang])


def damped_twist_to_dq(J: np.ndarray, twist: np.ndarray, damping: float = 0.05) -> np.ndarray:
    """twist shape (6,), J shape (6, n) -> dq shape (n,)."""
    lam = damping
    m = J.shape[1]
    jjt = J @ J.T + lam**2 * np.eye(6)
    return J.T @ np.linalg.solve(jjt, twist)


def clip_q_arm7(q: np.ndarray, joint_lo: np.ndarray, joint_hi: np.ndarray) -> np.ndarray:
    return np.clip(q.astype(np.float64), joint_lo, joint_hi)


def joint_limits_left7(gravity_comp) -> tuple[np.ndarray, np.ndarray]:
    """Lower/upper limits for first 7 left-arm joints from MuJoCo model."""
    model = gravity_comp.kdl.model
    n = 7
    lo = np.zeros(n)
    hi = np.zeros(n)
    for i in range(n):
        jid = i
        if model.jnt_limited[jid]:
            lo[i] = model.jnt_range[jid, 0]
            hi[i] = model.jnt_range[jid, 1]
        else:
            lo[i] = -np.pi
            hi[i] = np.pi
    return lo, hi
