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
from pathlib import Path

import can
import cv2
import mujoco
import mujoco.viewer
import numpy as np
from scipy.spatial.transform import Rotation as R_scipy

from .button_reader import ButtonReader
from .calibration import run_calibration
from .config import BTN_REMAP, FRAME_GAP, JOINT_GAINS, MOTOR_CONFIGS
from .encoding import (
    ControlMode,
    MitControlParams,
    PosVelControlParams,
    decode_motor_state_sync,
    encode_control_mit,
    encode_control_pos_vel,
)
from .gravity import GravityCompensator
from .hardware import (
    Arm,
    detect_and_disable_motors,
    monitor_motors,
    open_can_buses,
)
from .motor import Motor
from .recording import load_recording, run_playback, save_recording
from .trajectory import execute_waypoint, home_all_arms

 
from efficientloftr.src.loftr import LoFTR, full_default_cfg, reparameter

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
    from std_msgs.msg import Bool
    from sensor_msgs.msg import Image as RosImage
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

    Returns dict with 'left', 'right', and optionally 'servo' keys
    mapping to 4x4 numpy arrays.
    """
    import yaml
    with open(path) as f:
        data = yaml.safe_load(f)
    result = {}
    for side in ("left", "right", "servo"):
        if side not in data:
            continue
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


# ---------------------------------------------------------------------------
# IBVS feature providers + control helpers
# ---------------------------------------------------------------------------

from dataclasses import dataclass


@dataclass
class IBVSFeatures:
    """Result from a feature provider: matched current and target points."""
    current: np.ndarray     # (N, 2) current pixel coordinates (undistorted)
    target: np.ndarray      # (N, 2) desired pixel coordinates (undistorted)
    Z: np.ndarray           # (N,) per-point depth estimates in metres
    vis_frame: np.ndarray   # BGR image with annotations drawn


def ibvs_detect_aruco(
    corners_px: np.ndarray,
    servo_tv: np.ndarray,
    frame: np.ndarray,
    s_star: np.ndarray,
    K: np.ndarray,
    D: np.ndarray,
) -> IBVSFeatures | None:
    """ArUco-based feature provider for IBVS.

    Takes raw detected corners and returns undistorted current/target pairs
    with per-point depth and an annotated visualisation frame.
    """
    corners_ud = cv2.undistortPoints(
        corners_px.reshape(-1, 1, 2).astype(np.float64), K, D, P=K,
    ).reshape(-1, 2)

    Z_avg = float(servo_tv.flatten()[2])
    if Z_avg <= 0.01:
        Z_avg = 0.15
    Z_per_pt = np.full(len(corners_ud), Z_avg)

    vis = frame.copy()
    for ci in range(len(corners_ud)):
        pt_s = tuple(s_star[ci].astype(int))
        pt_c = tuple(corners_ud[ci].astype(int))
        cv2.circle(vis, pt_s, 6, (0, 255, 0), 2)
        cv2.circle(vis, pt_c, 6, (0, 0, 255), -1)
        cv2.line(vis, pt_c, pt_s, (255, 255, 0), 1)

    return IBVSFeatures(
        current=corners_ud,
        target=s_star,
        Z=Z_per_pt,
        vis_frame=vis,
    )


def load_template(template_dir: str) -> tuple[np.ndarray, np.ndarray]:
    """Load a template image and its COCO RLE segmentation mask.

    Returns (image_bgr, mask) where mask is a uint8 HxW array (255 inside, 0 outside).
    """
    import json as _json
    from pycocotools import mask as mask_utils

    template_path = Path(template_dir)
    json_file = template_path / "_annotations.coco.json"
    if not json_file.exists():
        raise FileNotFoundError(f"No _annotations.coco.json in {template_dir}")

    with open(json_file) as f:
        data = _json.load(f)

    img_name = data["images"][0]["file_name"]
    img = cv2.imread(str(template_path / img_name))
    if img is None:
        raise FileNotFoundError(f"Could not load image {template_path / img_name}")

    seg = data["annotations"][0]["segmentation"]
    h, w = seg["size"]
    counts = seg["counts"]
    if isinstance(counts, str):
        counts = counts.encode("utf-8")
    rle = {"counts": counts, "size": [h, w]}
    binary_mask = mask_utils.decode(rle)

    mask = (binary_mask * 255).astype(np.uint8)
    return img, mask


IBVS_HOMOGRAPHY_ALPHA = 1.0  # EMA smoothing for homography-derived points (0→max smooth, 1→no smooth)


@dataclass
class FeatureMatcherState:
    """Holds matcher backend, template data, and smoothing state."""
    backend: str                        # "disk" or "loftr"
    matcher: object
    extractor: object | None = None     # DISK extractor (None for LoFTR)
    template_feats: dict | None = None  # pre-extracted template features (DISK only)
    template_img: np.ndarray | None = None
    template_gray_torch: object = None  # pre-converted template tensor (LoFTR only)
    template_mask_torch: object = None  # pre-converted mask tensor (LoFTR only)
    template_mask: np.ndarray | None = None
    _smooth_current: np.ndarray | None = None


def init_feature_matcher(
    template_img: np.ndarray,
    template_mask: np.ndarray,
    backend: str = "loftr",
) -> FeatureMatcherState:
    """Initialise feature matcher. backend is 'disk' or 'loftr'."""
    import torch

    if backend == "disk":
        from lightglue import DISK, LightGlue
        from lightglue.utils import numpy_image_to_torch

        extractor = DISK(max_num_keypoints=2048).eval().cuda()
        matcher = LightGlue(features="disk", flash=True).eval().cuda()
        matcher.compile(mode="reduce-overhead")

        img_torch = numpy_image_to_torch(template_img).cuda()
        with torch.no_grad():
            feats = extractor.extract(img_torch)

        kpts = feats["keypoints"][0]
        kpts_np = kpts.cpu().numpy().astype(int)
        h, w = template_mask.shape[:2]
        keep = [
            i for i, (x, y) in enumerate(kpts_np)
            if 0 <= x < w and 0 <= y < h and template_mask[int(y), int(x)] > 0
        ]
        keep_t = torch.tensor(keep, device=kpts.device, dtype=torch.long)

        n_kpts = kpts.shape[0]
        filtered: dict = {}
        for k, v in feats.items():
            if isinstance(v, torch.Tensor) and v.ndim >= 2 and v.shape[1] == n_kpts:
                filtered[k] = v[:, keep_t]
            else:
                filtered[k] = v

        logger.info("DISK template: %d keypoints, %d inside mask", n_kpts, len(keep))
        return FeatureMatcherState(
            backend="disk",
            matcher=matcher,
            extractor=extractor,
            template_feats=filtered,
            template_img=template_img,
            template_mask=template_mask,
        )

    elif backend == "loftr":
        from copy import deepcopy
        # Initialize the matcher with default settings
        _default_cfg = deepcopy(full_default_cfg)
        matcher = LoFTR(config=_default_cfg)

        matcher.load_state_dict(torch.load("/home/hans/projects/openarm/efficientloftr/weights/eloftr_outdoor.ckpt")['state_dict'])
        matcher = reparameter(matcher)  # Essential for good performance
        matcher = matcher.eval().cuda()

        gray = cv2.cvtColor(template_img, cv2.COLOR_BGR2GRAY) if template_img.ndim == 3 else template_img
        template_gray_torch = torch.from_numpy(gray).float()[None, None].cuda() / 255.0

        mask_bool = (template_mask > 0).astype(np.uint8)
        template_mask_torch = torch.from_numpy(mask_bool)[None].cuda()

        logger.info("EfficientLoFTR initialised with template %dx%d", template_img.shape[1], template_img.shape[0])
        return FeatureMatcherState(
            backend="loftr",
            matcher=matcher,
            template_img=template_img,
            template_mask=template_mask,
            template_gray_torch=template_gray_torch,
            template_mask_torch=template_mask_torch,
        )

    else:
        raise ValueError(f"Unknown matcher backend: {backend!r} (use 'disk' or 'loftr')")


IBVS_TEMPLATE_REFS = np.array(
    [[173, 165], [192, 167], [172, 277], [189, 282], [190, 213]],
    dtype=np.float64,
)
IBVS_CUSTOM_Z = 0.15


def _match_disk(frame: np.ndarray, fm: FeatureMatcherState):
    """Run DISK + LightGlue matching. Returns (pts_template, pts_query, n_matches, vis) or None."""
    import torch
    from lightglue.utils import numpy_image_to_torch

    MATCH_CONF_THRESH = 0.85

    query_torch = numpy_image_to_torch(frame).cuda()
    with torch.no_grad():
        query_feats = fm.extractor.extract(query_torch)
        pred = fm.matcher({"image0": fm.template_feats, "image1": query_feats})

    matches0 = pred["matches0"][0]
    scores0 = pred["matching_scores0"][0]
    valid = (matches0 > -1) & (scores0 > MATCH_CONF_THRESH)
    n_matches = int(valid.sum())

    if n_matches < 10:
        return None, None, n_matches

    t_idx = torch.where(valid)[0]
    q_idx = matches0[valid]
    pts_template = fm.template_feats["keypoints"][0][t_idx].cpu().numpy().astype(np.float32)
    pts_query = query_feats["keypoints"][0][q_idx].cpu().numpy().astype(np.float32)
    return pts_template, pts_query, n_matches


def _match_loftr(frame: np.ndarray, fm: FeatureMatcherState):
    """Run EfficientLoFTR matching. Returns (pts_template, pts_query, n_matches) or None."""
    import torch

    MATCH_CONF_THRESH = 0.5

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
    query_torch = torch.from_numpy(gray).float()[None, None].cuda() / 255.0

    batch = {"image0": fm.template_gray_torch, "image1": query_torch}
    with torch.no_grad():
        fm.matcher(batch)
        mkpts0 = batch["mkpts0_f"].cpu().numpy()
        mkpts1 = batch["mkpts1_f"].cpu().numpy()
        mconf = batch["mconf"].cpu().numpy()

    # Filter by confidence and template mask
    mask_img = fm.template_mask
    valid = mconf >= MATCH_CONF_THRESH
    if mask_img is not None:
        h, w = mask_img.shape[:2]
        for i in range(len(mkpts0)):
            x, y = int(mkpts0[i, 0]), int(mkpts0[i, 1])
            if x < 0 or x >= w or y < 0 or y >= h or mask_img[y, x] == 0:
                valid[i] = False
    keep = np.where(valid)[0]
    n_matches = len(keep)
    if n_matches < 10:
        return None, None, n_matches

    pts_template = mkpts0[keep].astype(np.float32)
    pts_query = mkpts1[keep].astype(np.float32)
    return pts_template, pts_query, n_matches


def ibvs_match_custom(
    frame: np.ndarray,
    fm: FeatureMatcherState,
    K: np.ndarray,
    D: np.ndarray,
) -> IBVSFeatures | None:
    """Match template against live frame and return IBVSFeatures for the control law."""
    vis = frame.copy()

    if fm.backend == "disk":
        pts_template, pts_query, n_matches = _match_disk(frame, fm)
    elif fm.backend == "loftr":
        pts_template, pts_query, n_matches = _match_loftr(frame, fm)
    else:
        return None

    if pts_template is None:
        cv2.putText(
            vis, f"Matches: {n_matches} (need 10+)", (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2,
        )
        return None

    H, inlier_mask = cv2.findHomography(pts_template, pts_query, cv2.RANSAC, 3.0)
    if H is None:
        cv2.putText(
            vis, f"Homography FAILED ({n_matches} matches)", (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2,
        )
        return None

    inliers = inlier_mask.ravel().astype(bool)
    n_inliers = int(inliers.sum())

    for pt, is_in in zip(pts_query, inliers):
        color = (0, 200, 0) if is_in else (0, 0, 180)
        cv2.circle(vis, tuple(pt.astype(int)), 2, color, -1)

    # Warp template outline
    th, tw = fm.template_img.shape[:2]
    t_corners = np.array(
        [[0, 0], [tw, 0], [tw, th], [0, th]], dtype=np.float64,
    ).reshape(-1, 1, 2)
    warped_corners = cv2.perspectiveTransform(t_corners, H).reshape(-1, 2).astype(int)
    for i in range(4):
        cv2.line(vis, tuple(warped_corners[i]), tuple(warped_corners[(i + 1) % 4]), (255, 0, 255), 2)

    # Transform reference points: template (target) → live frame (current)
    target_dist = IBVS_TEMPLATE_REFS.copy()
    current_raw = cv2.perspectiveTransform(
        IBVS_TEMPLATE_REFS.reshape(-1, 1, 2), H,
    ).reshape(-1, 2)

    # Temporal EMA smoothing on transformed points
    alpha = IBVS_HOMOGRAPHY_ALPHA
    if fm._smooth_current is None:
        fm._smooth_current = current_raw.copy()
    else:
        fm._smooth_current = alpha * current_raw + (1.0 - alpha) * fm._smooth_current
    current_dist = fm._smooth_current

    # Draw distorted target (green) and current (red) with error lines (yellow)
    for i in range(len(IBVS_TEMPLATE_REFS)):
        pt_t = tuple(target_dist[i].astype(int))
        pt_c = tuple(current_dist[i].astype(int))
        cv2.circle(vis, pt_t, 6, (0, 255, 0), 2)
        cv2.circle(vis, pt_c, 6, (0, 0, 255), -1)
        cv2.line(vis, pt_c, pt_t, (0, 255, 255), 1)

    cv2.putText(
        vis, f"{fm.backend} {n_inliers}/{n_matches}", (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1,
    )

    # Undistort both point sets for the interaction matrix
    current_ud = cv2.undistortPoints(
        current_dist.reshape(-1, 1, 2).astype(np.float64), K, D, P=K,
    ).reshape(-1, 2)
    target_ud = cv2.undistortPoints(
        target_dist.reshape(-1, 1, 2).astype(np.float64), K, D, P=K,
    ).reshape(-1, 2)

    Z_per_pt = np.full(len(IBVS_TEMPLATE_REFS), IBVS_CUSTOM_Z)

    return IBVSFeatures(
        current=current_ud,
        target=target_ud,
        Z=Z_per_pt,
        vis_frame=vis,
    )


def ibvs_compute_velocity(
    features: IBVSFeatures,
    K: np.ndarray,
    lam: float,
    kd: float,
    deadband_px: float,
    e_norm_prev: np.ndarray | None,
    dt: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    """Compute IBVS camera-frame velocity from feature error.

    Returns (v_cam, e_norm, e_raw_flat, pix_err).
    """
    fx, fy = K[0, 0], K[1, 1]
    u0, v0 = K[0, 2], K[1, 2]

    n_pts = len(features.current)
    L = np.zeros((2 * n_pts, 6))
    for k in range(n_pts):
        u, v = features.current[k]
        x = (u - u0) / fx
        y = (v - v0) / fy
        Z = features.Z[k]
        row = 2 * k
        L[row]     = [-1/Z,    0, x/Z,     x*y, -(1+x*x),  y]
        L[row + 1] = [   0, -1/Z, y/Z, 1+y*y,     -x*y, -x]

    e_raw = (features.current - features.target).flatten()
    e = np.where(np.abs(e_raw) < deadband_px, 0.0, e_raw)
    e_norm = np.array([
        e[i] / fx if i % 2 == 0 else e[i] / fy
        for i in range(len(e))
    ])
    pix_err = np.linalg.norm(e_raw)

    L_pinv = np.linalg.pinv(L)
    v_cam = -lam * L_pinv @ e_norm

    if e_norm_prev is not None and dt > 0 and len(e_norm_prev) == len(e_norm):
        de = (e_norm - e_norm_prev) / dt
        v_cam -= kd * L_pinv @ de

    return v_cam, e_norm, e_raw, pix_err


def ibvs_draw_hud(
    vis: np.ndarray,
    v_cam: np.ndarray,
    dq: np.ndarray,
    pix_err: float,
    Z_avg: float,
    hz: float,
    lam: float,
    lam_max: float,
    kd: float,
    ibvs_cmd_q: list,
    ibvs_start_q: list,
    max_total_deg: float,
    is_moving: bool,
) -> None:
    """Draw IBVS HUD overlay onto vis image (in-place)."""
    h, w = vis.shape[:2]
    vx, vy = v_cam[0], v_cam[1]

    vel_scale = 200.0
    center = (w // 2, h // 2)
    cv2.arrowedLine(
        vis, center,
        (int(center[0] + vx * vel_scale), int(center[1] + vy * vel_scale)),
        (255, 0, 255), 2, tipLength=0.3,
    )

    total_disp_deg = np.degrees(
        max(abs(ibvs_cmd_q[j] - ibvs_start_q[j]) for j in range(len(ibvs_cmd_q)))
    )
    dq_deg = np.degrees(dq)
    info_lines = [
        f"e_px: {pix_err:.1f}  Z: {Z_avg:.3f}m  {hz:.1f} Hz",
        f"lam: {lam:.3f}/{lam_max:.2f}  kd: {kd:.2f}",
        f"v: [{v_cam[0]:.4f} {v_cam[1]:.4f} {v_cam[2]:.4f}]",
        f"w: [{v_cam[3]:.4f} {v_cam[4]:.4f} {v_cam[5]:.4f}]",
        f"dq: [{' '.join(f'{d:.2f}' for d in dq_deg)}] deg",
        f"max_disp: {total_disp_deg:.1f}/{max_total_deg:.0f} deg",
        f"{'SERVOING' if is_moving else 'PAUSED (press y)'}",
    ]
    for li, txt in enumerate(info_lines):
        cv2.putText(
            vis, txt, (10, 20 + li * 18),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1,
        )


async def main(args: argparse.Namespace) -> None:
    """Run the monitor with the provided arguments."""
    can_buses = open_can_buses()
    if not can_buses:
        return

    sys.stdout.write(f"\nDetected {len(can_buses)} CAN bus(es)\n")

    try:
        result = await detect_and_disable_motors(can_buses)
        if result is None:
            return
        all_bus_motors, all_state_results = result

        args.ee_T_cam = {}
        if args.cam_extrinsics:
            sys.stdout.write(f"\nLoading camera extrinsics from {args.cam_extrinsics}\n")
            args.ee_T_cam = load_cam_extrinsics(args.cam_extrinsics)

        if args.teleop:
            await teleop(can_buses, all_bus_motors, all_state_results, args)
        else:
            await monitor_motors(can_buses, all_bus_motors, all_state_results)
    finally:
        for bus in can_buses:
            bus.shutdown()


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
    """Thread-safe container for the latest FoundationPose, ArUco, and head PoseStamped."""

    ARUCO_DICT_TYPE = cv2.aruco.DICT_4X4_1000
    ARUCO_MARKER_SIZE = 0.060  # metres

    def __init__(self):
        self._lock = threading.Lock()
        self._pose = None  # type: PoseStamped | None
        self._aruco = None  # type: PoseStamped | None
        self._head = None  # type: PoseStamped | None
        self._servo_corners = None  # type: np.ndarray | None  — (4,2) pixel coords
        self._servo_rvec = None  # type: np.ndarray | None
        self._servo_tvec = None  # type: np.ndarray | None
        self._servo_frame = None  # type: np.ndarray | None  — latest BGR frame
        self._servo_frame_seq = 0
        self._servo_K = None  # type: np.ndarray | None
        self._servo_D = None  # type: np.ndarray | None
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
            PoseStamped, "/aruco/pose", self._aruco_cb, 1,
        )
        self._node.create_subscription(
            PoseStamped, "/head_pose", self._head_cb, 1,
        )
        self._node.create_subscription(
            RosImage, "/camera_visual_servo/image_raw", self._servo_image_cb, 1,
        )
        self._servo_debug_pub = self._node.create_publisher(
            RosImage, "/servo_aruco/debug_image", 1,
        )
        self._register_pub = self._node.create_publisher(Bool, "/foundationpose/register", 10)
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()
        logger.info(
            "Subscribed to /foundationpose/pose, /aruco/pose, "
            "/head_pose, /camera_visual_servo/image_raw"
        )

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

    def send_register(self):
        if self._node is not None:
            self._last_register = not getattr(self, '_last_register', False)
            msg = Bool()
            msg.data = self._last_register
            self._register_pub.publish(msg)
            return self._last_register
        return None

    def _pose_cb(self, msg):
        with self._lock:
            self._pose = msg

    def _aruco_cb(self, msg):
        with self._lock:
            self._aruco = msg

    def _head_cb(self, msg):
        with self._lock:
            self._head = msg

    def load_servo_intrinsics(self, yaml_path: str) -> None:
        """Load servo camera intrinsics from a Kalibr-style YAML file."""
        import yaml as _yaml
        with open(yaml_path) as f:
            data = _yaml.safe_load(f)
        cam = data["cam0"]
        fx, fy, cx, cy = cam["intrinsics"]
        self._servo_K = np.array([
            [fx, 0, cx],
            [0, fy, cy],
            [0,  0,  1],
        ], dtype=np.float64)
        self._servo_D = np.array(cam["distortion_coeffs"], dtype=np.float64)
        logger.info("Loaded servo intrinsics from %s", yaml_path)

    @staticmethod
    def _ros_image_to_cv(msg) -> np.ndarray:
        """Convert a sensor_msgs/Image to a BGR numpy array."""
        h, w = msg.height, msg.width
        if msg.encoding in ("rgb8", "RGB8"):
            return cv2.cvtColor(
                np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3),
                cv2.COLOR_RGB2BGR,
            )
        if msg.encoding in ("bgr8", "BGR8"):
            return np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3).copy()
        if msg.encoding in ("mono8", "8UC1"):
            gray = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w)
            return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        if msg.encoding == "yuyv":
            raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 2)
            return cv2.cvtColor(raw, cv2.COLOR_YUV2BGR_YUYV)
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, -1)
        return arr[:, :, :3].copy()

    def _cv_to_ros_image(self, img: np.ndarray) -> "RosImage":
        """Convert a BGR numpy array to a sensor_msgs/Image."""
        import array as _array
        msg = RosImage()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = "camera_visual_servo"
        msg.height, msg.width = img.shape[:2]
        msg.encoding = "bgr8"
        msg.step = msg.width * 3
        buf = _array.array("B")
        buf.frombytes(np.ascontiguousarray(img).tobytes())
        msg.data = buf
        return msg

    def _servo_image_cb(self, msg) -> None:
        """Detect ArUco on the servo camera image, store corners, publish debug."""
        if self._servo_K is None:
            return
        frame = self._ros_image_to_cv(msg)
        aruco_dict = cv2.aruco.getPredefinedDictionary(self.ARUCO_DICT_TYPE)
        params = cv2.aruco.DetectorParameters()
        corners, ids, _ = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=params)

        rvec, tvec, raw_corners = None, None, None
        if len(corners) > 0 and ids is not None and len(ids) == 1:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
            cv2.cornerSubPix(
                gray, corners[0],
                winSize=(5, 5), zeroZone=(-1, -1),
                criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01),
            )
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners[0], self.ARUCO_MARKER_SIZE, self._servo_K, self._servo_D,
            )
            raw_corners = corners[0].reshape(4, 2)
            cv2.aruco.drawDetectedMarkers(frame, corners)
            cv2.drawFrameAxes(
                frame, self._servo_K, self._servo_D,
                rvec, tvec, self.ARUCO_MARKER_SIZE / 2,
            )

        with self._lock:
            self._servo_corners = raw_corners
            self._servo_rvec = rvec
            self._servo_tvec = tvec
            self._servo_frame = frame
            self._servo_frame_seq += 1

        #self._servo_debug_pub.publish(self._cv_to_ros_image(frame))

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
    def latest_aruco(self):
        with self._lock:
            return self._aruco

    @property
    def latest_servo_corners(self):
        """Return the latest servo-camera ArUco corner pixels as (4,2) array, or None."""
        with self._lock:
            return self._servo_corners.copy() if self._servo_corners is not None else None

    @property
    def latest_servo_pose(self):
        """Return (rvec, tvec) from the servo camera ArUco detection, or (None, None)."""
        with self._lock:
            rv = self._servo_rvec.copy() if self._servo_rvec is not None else None
            tv = self._servo_tvec.copy() if self._servo_tvec is not None else None
            return rv, tv

    @property
    def latest_servo_frame(self):
        """Return a copy of the latest servo camera BGR frame, or None."""
        with self._lock:
            return self._servo_frame.copy() if self._servo_frame is not None else None

    @property
    def servo_frame_seq(self):
        """Monotonically increasing counter — bumped on each new servo image."""
        with self._lock:
            return self._servo_frame_seq

    @property
    def servo_K(self):
        return self._servo_K

    @property
    def servo_D(self):
        return self._servo_D

    def publish_servo_debug(self, img: np.ndarray) -> None:
        """Publish an annotated BGR image on /servo_aruco/debug_image."""
        if self._node is not None:
            self._servo_debug_pub.publish(self._cv_to_ros_image(img))

    @property
    def latest_head(self):
        with self._lock:
            return self._head

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
    if args.servo_intrinsics:
        pose_listener.load_servo_intrinsics(args.servo_intrinsics)

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
    stop_msg = "Q stop | X/Y/Z hold | I impedance | V vr-track | W save wp | Btn1 FPose | Btn2 ArUco | Btn4 register"
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

    # Slave impedance control: when enabled, slave tracks master TCP pose
    # via J^T impedance instead of joint-level position control.
    slave_impedance_enabled: bool = False
    slave_imp_trans_stiffness = np.array([5000.0, 200.0, 200.0])  # N/m per axis [x, y, z]
    slave_imp_rot_stiffness = np.array([10.0, 10.0, 10.0])          # N·m/rad per axis [r, p, y]
    slave_imp_kp_scale = 0.1  # multiply normal kp by this when impedance active


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
                elif key == "i":
                    slave_impedance_enabled = not slave_impedance_enabled
                    state = "ON" if slave_impedance_enabled else "OFF"
                    raw_print(
                        f"\n  Slave impedance {state}"
                        f"  trans={slave_imp_trans_stiffness.tolist()}"
                        f"  rot={slave_imp_rot_stiffness.tolist()}"
                        f"  kp_scale={slave_imp_kp_scale}\n"
                    )

                elif key == "v":
                    head_pose = pose_listener.latest_head
                    if head_pose is None:
                        raw_print("\n  VR tracking: no /head_pose data yet\n")
                    elif gravity_comp is None:
                        raw_print("\n  VR tracking: gravity comp not enabled\n")
                    else:
                        vr_master = None
                        vr_slave = None
                        for arm in arms:
                            if arm.is_master and arm.position == "right":
                                vr_master = arm
                            if arm.is_slave and arm.position == "right":
                                vr_slave = arm
                        if vr_master is None or vr_slave is None:
                            raw_print("\n  VR tracking: no right master/slave pair\n")
                        else:
                            cur_q = vr_slave.get_positions()
                            vr_ref_arm_pos, vr_ref_arm_quat = gravity_comp.forward_kinematics(
                                cur_q[:7], position="right",
                            )
                            hq = head_pose.pose.orientation
                            vr_ref_head_quat = np.array([hq.w, hq.x, hq.y, hq.z])
                            raw_print(
                                f"\n  VR tracking ON — press V to exit"
                                f"  arm_rpy={_quat_to_rpy_deg(vr_ref_arm_quat)}"
                                f"  head_ref=[{vr_ref_head_quat[0]:.3f},{vr_ref_head_quat[1]:.3f},"
                                f"{vr_ref_head_quat[2]:.3f},{vr_ref_head_quat[3]:.3f}]\n"
                            )

                            _s_gc = slave_gravity_comp or gravity_comp
                            vr_hz = 200.0
                            vr_dt = 1.0 / vr_hz
                            gripper_q = cur_q[7] if len(cur_q) > 7 else -0.7
                            gripper_kp, gripper_kd = JOINT_GAINS[7] if len(JOINT_GAINS) > 7 else (4.0, 1.0)
                            vr_step = 0
                            vr_smooth_alpha = 0.15  # SLERP blend per tick (0=frozen, 1=instant)
                            vr_max_joint_vel = 2.0  # rad/s max per joint
                            vr_smooth_quat = vr_ref_arm_quat.copy()
                            vr_cmd_q = list(cur_q[:7])

                            while True:
                                vr_iter_start = time.perf_counter()

                                vr_key = check_keyboard_input()
                                if vr_key == "v" or vr_key == "q":
                                    break

                                hp = pose_listener.latest_head
                                if hp is None:
                                    await asyncio.sleep(vr_dt)
                                    continue

                                hq_now = hp.pose.orientation
                                head_now = np.array([hq_now.w, hq_now.x, hq_now.y, hq_now.z])

                                # delta = inv(head_ref) * head_now
                                head_ref_inv = np.array([
                                    vr_ref_head_quat[0],
                                    -vr_ref_head_quat[1],
                                    -vr_ref_head_quat[2],
                                    -vr_ref_head_quat[3],
                                ])
                                delta_quat = _quat_multiply(head_ref_inv, head_now)

                                # Remap axes: negate roll, swap pitch and yaw of delta quat
                                delta_quat = np.array([
                                    delta_quat[0],
                                    delta_quat[2],
                                    delta_quat[1],
                                    -delta_quat[3],
                                ])

                                # target = arm_ref * delta
                                raw_target_quat = _quat_multiply(vr_ref_arm_quat, delta_quat)

                                # SLERP smoothing: blend towards raw target
                                if np.dot(vr_smooth_quat, raw_target_quat) < 0:
                                    raw_target_quat = -raw_target_quat
                                vr_smooth_quat = (
                                    (1.0 - vr_smooth_alpha) * vr_smooth_quat
                                    + vr_smooth_alpha * raw_target_quat
                                )
                                vr_smooth_quat /= np.linalg.norm(vr_smooth_quat)

                                s_positions = vr_slave.get_positions()

                                ik_q = gravity_comp.inverse_kinematics(
                                    target_pos=vr_ref_arm_pos,
                                    target_quat=vr_smooth_quat,
                                    seed_angles=s_positions[:7],
                                    position="right",
                                )

                                # Rate-limit joint commands
                                max_delta = vr_max_joint_vel * vr_dt
                                for j in range(7):
                                    d = float(ik_q[j]) - vr_cmd_q[j]
                                    d = max(-max_delta, min(max_delta, d))
                                    vr_cmd_q[j] += d

                                target_q_full = list(vr_cmd_q) + [gripper_q]
                                grav = _s_gc.compute(s_positions, position="right")

                                # Command slave (right arm)
                                for idx, motor in enumerate(vr_slave.motors):
                                    if motor is None:
                                        continue
                                    if idx == 7:
                                        params = MitControlParams(
                                            q=gripper_q, dq=0,
                                            kp=gripper_kp, kd=gripper_kd, tau=0,
                                        )
                                    else:
                                        kp, kd = JOINT_GAINS[idx] if idx < len(JOINT_GAINS) else (2.0, 1.0)
                                        tau = grav[idx] if idx < len(grav) else 0.0
                                        params = MitControlParams(
                                            q=target_q_full[idx], dq=0,
                                            kp=kp, kd=kd, tau=tau,
                                        )
                                    try:
                                        encode_control_mit(motor._bus, motor._slave_id, motor._motor_limits, params)
                                        time.sleep(FRAME_GAP)
                                        state = decode_motor_state_sync(motor._bus, motor._master_id, motor._motor_limits)
                                        if state is not None:
                                            vr_slave.states[idx] = state
                                    except Exception:
                                        pass

                                # Command master (track same target so it follows)
                                m_grav = gravity_comp.compute(vr_master.get_positions(), position="right")
                                for idx, motor in enumerate(vr_master.motors):
                                    if motor is None:
                                        continue
                                    tau = m_grav[idx] if idx < len(m_grav) else 0.0
                                    kp, kd = JOINT_GAINS[idx] if idx < len(JOINT_GAINS) else (2.0, 1.0)
                                    params = MitControlParams(
                                        q=target_q_full[idx], dq=0, kp=kp, kd=kd, tau=tau,
                                    )
                                    try:
                                        encode_control_mit(motor._bus, motor._slave_id, motor._motor_limits, params)
                                        time.sleep(FRAME_GAP)
                                        state = decode_motor_state_sync(motor._bus, motor._master_id, motor._motor_limits)
                                        if state is not None:
                                            vr_master.states[idx] = state
                                    except Exception:
                                        pass

                                if vr_step % 100 == 0:
                                    delta_rpy = _quat_to_rpy_deg(delta_quat)
                                    target_rpy = _quat_to_rpy_deg(vr_smooth_quat)
                                    raw_print(
                                        f"  VR: delta_rpy=[{delta_rpy[0]:.1f},{delta_rpy[1]:.1f},{delta_rpy[2]:.1f}]"
                                        f"  target_rpy=[{target_rpy[0]:.1f},{target_rpy[1]:.1f},{target_rpy[2]:.1f}]\n"
                                    )

                                vr_step += 1
                                elapsed = time.perf_counter() - vr_iter_start
                                remaining = vr_dt - elapsed
                                if remaining > 0:
                                    await asyncio.sleep(remaining)

                            # Update slave cmd pos so teleop resumes smoothly
                            if vr_slave.channel in _slave_cmd_pos:
                                _slave_cmd_pos[vr_slave.channel] = vr_slave.get_positions()
                            raw_print("\n  VR tracking OFF — resuming teleop\n")

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
                                            additional_yaw_x_rotation = 30.0
                                        else:
                                            additional_yaw_x_rotation = -30.0
                                        
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
                        # Button 2: move left hand to ArUco pose
                        aruco_pose = pose_listener.latest_aruco
                        if aruco_pose is None:
                            raw_print(f"\n  Button 2: no ArUco pose yet\n")
                        elif gravity_comp is None:
                            raw_print(f"\n  Button 2: gravity comp not enabled\n")
                        else:
                            aruco_arm = "left"
                            chosen_master = None
                            chosen_slave = None
                            for arm in arms:
                                if arm.is_master and arm.position == aruco_arm:
                                    chosen_master = arm
                                if arm.is_slave and arm.position == aruco_arm:
                                    chosen_slave = arm

                            if chosen_slave is None:
                                raw_print(f"\n  Button 2: no {aruco_arm} follower arm\n")
                            else:
                                cur_q = chosen_slave.get_positions()

                                camera_body = f"openarm_{aruco_arm}_camera"
                                tcp_pos, tcp_quat = gravity_comp.forward_kinematics(
                                    cur_q[:7], position=aruco_arm,
                                )
                                cam_pos, cam_quat = gravity_comp.kdl.compute_body_pose(
                                    np.array(cur_q[:7]), camera_body, side=aruco_arm,
                                )
                                T_world_tcp = _pose_to_T(tcp_pos, tcp_quat)
                                T_world_cam = _pose_to_T(cam_pos, cam_quat)
                                T_tcp_cam = _T_inv(T_world_tcp) @ T_world_cam

                                T_finger_length = np.eye(4)
                                T_finger_length[2, 3] = 0.02
                                T_world_tcp_ext = T_world_tcp @ T_finger_length
                                T_world_cam_ext = T_world_tcp_ext @ T_tcp_cam

                                T_cam_obj = _ros_pose_to_T(aruco_pose.pose)
                                T_world_obj = T_world_cam_ext @ T_cam_obj

                                Rz_90 = np.eye(4)
                                Rz_90[:3, :3] = np.array([
                                    [0, 1, 0],
                                    [-1,  0, 0],
                                    [0,  0, 1],
                                ])
                                T_world_obj = T_world_obj @ Rz_90

                                obj_T_wps = []
                                wp1 = np.eye(4)
                                wp1[2, 3] = -0.135
                                #wp1[2, 3] = -0.18
                                obj_T_wps.append(("above", wp1, 3.0, -0.1))

                                wp2 = np.eye(4)
                                wp2[2, 3] = -0.135
                                #wp2[1, 3] = -0.03
                                wp2[1, 3] = -0.07
                                obj_T_wps.append(("above", wp2, 3.0, -0.1))


                                # wp2 = np.eye(4)
                                # wp2[0, 3] = 0.02
                                # wp2[1, 3] = -0.065
                                # wp2[2, 3] = -0.1
                                # obj_T_wps.append(("contact", wp2, 2.0, -0.2))

                                # wp3 = np.eye(4)
                                # wp3[0, 3] = 0.02
                                # wp3[1, 3] = -0.065
                                # wp3[2, 3] = -0.08
                                # obj_T_wps.append(("contact", wp3, 2.0, -0.2))

                                raw_print(f"\n  Button 2: move {aruco_arm} to ArUco target ({len(obj_T_wps)} waypoints)")
                                raw_print(f"    Current TCP: [{tcp_pos[0]:.3f}, {tcp_pos[1]:.3f}, {tcp_pos[2]:.3f}]")
                                raw_print(f"    ArUco (cam): x={aruco_pose.pose.position.x:.3f} y={aruco_pose.pose.position.y:.3f} z={aruco_pose.pose.position.z:.3f}")

                                seed_q = cur_q[:7]
                                current_gripper = cur_q[7] if len(cur_q) > 7 else 0.0
                                ik_waypoints = []
                                aborted = False
                                for wp_idx, (wp_name, obj_T_wp, wp_dur, grip_angle) in enumerate(obj_T_wps):
                                    T_world_wp = T_world_obj @ obj_T_wp
                                    target_pos, target_quat = _T_to_pos_quat(T_world_wp)

                                    raw_print(f"    WP {wp_idx}/{len(obj_T_wps)-1} '{wp_name}': [{target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}]")

                                    target_q = gravity_comp.inverse_kinematics(
                                        target_pos=target_pos,
                                        target_quat=target_quat,
                                        seed_angles=seed_q,
                                        position=aruco_arm,
                                    )

                                    verify_pos, verify_quat = gravity_comp.forward_kinematics(
                                        list(target_q), position=aruco_arm,
                                    )
                                    pos_err = np.linalg.norm(verify_pos - target_pos)
                                    ori_err_vec = np.zeros(3)
                                    vq = verify_quat.copy()
                                    if np.dot(vq, target_quat) < 0:
                                        vq = -vq
                                    mujoco.mju_subQuat(ori_err_vec, target_quat, vq)
                                    rot_err = np.degrees(np.linalg.norm(ori_err_vec))
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

                                    # --- IBVS visualisation + servo loop ---
                                    raw_print(f"    Approach done — entering IBVS (press y to servo, q to exit)\n")

                                    IMG_W, IMG_H = 640, 480
                                    SQUARE_SIZE = 400
                                    half = SQUARE_SIZE / 3.0
                                    ibvs_cx, ibvs_cy = IMG_W / 2.0, IMG_H / 2.0
                                    s_star = np.array([
                                        [ibvs_cx + half, ibvs_cy - half],
                                        [ibvs_cx + half, ibvs_cy + half],
                                        [ibvs_cx - half, ibvs_cy + half],
                                        [ibvs_cx - half, ibvs_cy - half],
                                    ], dtype=np.float64)

                                    IBVS_LAMBDA = 0.25
                                    IBVS_LAMBDA_RAMP_PX = 50.0
                                    IBVS_KD = 0.0
                                    IBVS_MAX_TOTAL_DEG = 20.0
                                    ibvs_max_total_rad = np.radians(IBVS_MAX_TOTAL_DEG)
                                    IBVS_MAX_DQ_PER_STEP = np.radians(1.0)
                                    IBVS_DEADBAND_PX = 1.5

                                    ibvs_moving = False
                                    ibvs_start_q = chosen_slave.get_positions()[:7]
                                    ibvs_cmd_q = list(ibvs_start_q)
                                    servo_cam_body = "openarm_left_servo_camera"
                                    ibvs_tick_t = time.perf_counter()
                                    ibvs_hz = 0.0
                                    ibvs_last_seq = pose_listener.servo_frame_seq
                                    e_norm_prev = None
                                    ibvs_converge_count = 0
                                    ibvs_use_aruco = False  # toggle: True = ArUco, False = custom features

                                    template_img, template_mask = load_template(
                                        "/home/hans/projects/openarm/port_template_image",
                                    )
                                    overlay = template_img.copy()
                                    overlay[template_mask > 0] = (
                                        overlay[template_mask > 0] * 0.5
                                        + np.array([0, 255, 0], dtype=np.uint8) * 0.5
                                    ).astype(np.uint8)


                                    feature_matcher = init_feature_matcher(template_img, template_mask)

                                    while True:
                                        key = check_keyboard_input()
                                        if key == "q":
                                            raw_print(f"    IBVS exited by user\n")
                                            break
                                        if key == "y" and not ibvs_moving:
                                            ibvs_moving = True
                                            ibvs_start_q = chosen_slave.get_positions()[:7]
                                            ibvs_cmd_q = list(ibvs_start_q)
                                            for motor in chosen_slave.motors:
                                                if motor is not None:
                                                    await motor.set_control_mode(ControlMode.POS_VEL)
                                            if chosen_master is not None:
                                                for motor in chosen_master.motors:
                                                    if motor is not None:
                                                        await motor.set_control_mode(ControlMode.POS_VEL)
                                            raw_print(f"    IBVS servoing ENABLED (POS_VEL mode)\n")

                                        cur_seq = pose_listener.servo_frame_seq
                                        if cur_seq == ibvs_last_seq:
                                            await asyncio.sleep(0.001)
                                            continue
                                        ibvs_last_seq = cur_seq

                                        now_t = time.perf_counter()
                                        dt_actual = now_t - ibvs_tick_t
                                        ibvs_tick_t = now_t
                                        if dt_actual > 0:
                                            ibvs_hz = 0.9 * ibvs_hz + 0.1 * (1.0 / dt_actual)

                                        K = pose_listener.servo_K
                                        D = pose_listener.servo_D

                                        # --- Feature detection ---
                                        if ibvs_use_aruco:
                                            corners_px = pose_listener.latest_servo_corners
                                            servo_rv, servo_tv = pose_listener.latest_servo_pose
                                            frame = pose_listener.latest_servo_frame
                                            if corners_px is None or servo_tv is None or frame is None:
                                                await asyncio.sleep(0.05)
                                                continue
                                            features = ibvs_detect_aruco(
                                                corners_px, servo_tv, frame, s_star, K, D,
                                            )
                                        else:
                                            frame = pose_listener.latest_servo_frame
                                            if frame is None:
                                                await asyncio.sleep(0.05)
                                                continue
                                            features = ibvs_match_custom(frame, feature_matcher, K, D)

                                        if features is None:
                                            await asyncio.sleep(0.05)
                                            continue

                                        # --- Adaptive gain ---
                                        pix_err_preview = np.linalg.norm(
                                            (features.current - features.target).flatten()
                                        )
                                        lam = IBVS_LAMBDA * min(1.0, pix_err_preview / IBVS_LAMBDA_RAMP_PX)

                                        # --- IBVS velocity ---
                                        v_cam, e_norm, e_raw, pix_err = ibvs_compute_velocity(
                                            features, K,
                                            lam=lam,
                                            kd=IBVS_KD,
                                            deadband_px=IBVS_DEADBAND_PX,
                                            e_norm_prev=e_norm_prev,
                                            dt=dt_actual,
                                        )
                                        e_norm_prev = e_norm.copy()

                                        # --- Camera-frame Jacobian via adjoint ---
                                        cur_q7 = chosen_slave.get_positions()[:7]
                                        J_world = gravity_comp.kdl.compute_jacobian_body(
                                            np.array(cur_q7), servo_cam_body, side=aruco_arm,
                                        )
                                        cam_pos, cam_quat = gravity_comp.kdl.compute_body_pose(
                                            np.array(cur_q7), servo_cam_body, side=aruco_arm,
                                        )
                                        R_cam = R_scipy.from_quat(
                                            [cam_quat[1], cam_quat[2], cam_quat[3], cam_quat[0]]
                                        ).as_matrix()
                                        R_cw = R_cam.T
                                        Ad = np.zeros((6, 6))
                                        Ad[:3, :3] = R_cw
                                        Ad[3:, 3:] = R_cw
                                        J_cam = Ad @ J_world

                                        J_cam_pinv = np.linalg.pinv(J_cam)
                                        dq = J_cam_pinv @ (v_cam * dt_actual)

                                        dq_norm = np.linalg.norm(dq)
                                        if dq_norm > IBVS_MAX_DQ_PER_STEP:
                                            dq = dq * (IBVS_MAX_DQ_PER_STEP / dq_norm)

                                        # --- Motor commands ---
                                        if ibvs_moving:
                                            candidate = [ibvs_cmd_q[j] + dq[j] for j in range(7)]
                                            clamped_joints = []
                                            for j in range(7):
                                                delta = candidate[j] - ibvs_start_q[j]
                                                clamped = max(-ibvs_max_total_rad, min(ibvs_max_total_rad, delta))
                                                if abs(clamped - delta) > 1e-6:
                                                    clamped_joints.append(j)
                                                ibvs_cmd_q[j] = ibvs_start_q[j] + clamped
                                            if clamped_joints:
                                                raw_print(f"    IBVS total clamp hit on joints {clamped_joints}\n")

                                            gripper_val = 0.2
                                            target_full = list(ibvs_cmd_q) + [gripper_val]
                                            IBVS_MAX_VEL = 1.0  # rad/s max velocity for POS_VEL mode

                                            for idx, motor in enumerate(chosen_slave.motors):
                                                if motor is None or idx >= 8:
                                                    continue
                                                params = PosVelControlParams(
                                                    position=target_full[idx], velocity=IBVS_MAX_VEL,
                                                )
                                                try:
                                                    encode_control_pos_vel(
                                                        motor._bus, motor._slave_id, params,
                                                    )
                                                    time.sleep(FRAME_GAP)
                                                    state = decode_motor_state_sync(
                                                        motor._bus, motor._master_id, motor._motor_limits,
                                                    )
                                                    if state is not None:
                                                        chosen_slave.states[idx] = state
                                                except Exception:
                                                    pass

                                            if chosen_master is not None:
                                                for idx, motor in enumerate(chosen_master.motors):
                                                    if motor is None or idx >= 8:
                                                        continue
                                                    params = PosVelControlParams(
                                                        position=target_full[idx], velocity=IBVS_MAX_VEL,
                                                    )
                                                    try:
                                                        encode_control_pos_vel(
                                                            motor._bus, motor._slave_id, params,
                                                        )
                                                        time.sleep(FRAME_GAP)
                                                        state = decode_motor_state_sync(
                                                            motor._bus, motor._master_id, motor._motor_limits,
                                                        )
                                                        if state is not None:
                                                            chosen_master.states[idx] = state
                                                    except Exception:
                                                        pass

                                        # --- Convergence: push 2 cm forward in EE Z ---
                                        IBVS_CONVERGE_PX = 3.0
                                        IBVS_CONVERGE_ITERS = 5
                                        IBVS_PUSH_Z_M = 0.02
                                        IBVS_PUSH_DUR = 3.0
                                        if ibvs_moving and pix_err < IBVS_CONVERGE_PX:
                                            ibvs_converge_count += 1
                                        else:
                                            ibvs_converge_count = 0
                                        if ibvs_moving and ibvs_converge_count >= IBVS_CONVERGE_ITERS:
                                            raw_print(f"    IBVS converged ({ibvs_converge_count} iters < {IBVS_CONVERGE_PX}px, err={pix_err:.2f}px) — pushing {IBVS_PUSH_Z_M*100:.0f}cm forward\n")
                                            for motor in chosen_slave.motors:
                                                if motor is not None:
                                                    await motor.set_control_mode(ControlMode.MIT)
                                            if chosen_master is not None:
                                                for motor in chosen_master.motors:
                                                    if motor is not None:
                                                        await motor.set_control_mode(ControlMode.MIT)
                                            cur_q7 = chosen_slave.get_positions()[:7]
                                            tcp_pos, tcp_quat = gravity_comp.forward_kinematics(
                                                cur_q7, position=aruco_arm,
                                            )
                                            T_world_ee = np.eye(4)
                                            T_world_ee[:3, :3] = R_scipy.from_quat(
                                                [tcp_quat[1], tcp_quat[2], tcp_quat[3], tcp_quat[0]]
                                            ).as_matrix()
                                            T_world_ee[:3, 3] = tcp_pos
                                            T_push = np.eye(4)
                                            T_push[1, 3] = 0.000015
                                            T_push[0, 3] = -0.0065
                                            T_push[2, 3] = IBVS_PUSH_Z_M
                                            push_pos = (T_world_ee @ T_push)[:3, 3]

                                            push_q = gravity_comp.inverse_kinematics(
                                                push_pos, tcp_quat,
                                                seed_angles=list(cur_q7),
                                                position=aruco_arm,
                                            )
                                            gripper_val = 0.2
                                            push_wp = [(list(push_q) + [gripper_val], IBVS_PUSH_DUR)]
                                            raw_print(f"    Executing push waypoint ({IBVS_PUSH_DUR:.1f}s)...\n")
                                            await execute_waypoint(
                                                push_wp, chosen_master, chosen_slave,
                                                gravity_comp=gravity_comp,
                                                slave_gravity_comp=slave_gravity_comp,
                                                hz=200.0,
                                            )
                                            raw_print(f"    Push complete — returning to teleop\n")
                                            break

                                        # --- HUD + publish ---
                                        Z_avg = float(np.mean(features.Z))
                                        vis = features.vis_frame
                                        ibvs_draw_hud(
                                            vis, v_cam, dq,
                                            pix_err=pix_err,
                                            Z_avg=Z_avg,
                                            hz=ibvs_hz,
                                            lam=lam,
                                            lam_max=IBVS_LAMBDA,
                                            kd=IBVS_KD,
                                            ibvs_cmd_q=ibvs_cmd_q,
                                            ibvs_start_q=ibvs_start_q,
                                            max_total_deg=IBVS_MAX_TOTAL_DEG,
                                            is_moving=ibvs_moving,
                                        )
                                        pose_listener.publish_servo_debug(vis)

                                    # Restore MIT mode after IBVS loop exits
                                    if ibvs_moving:
                                        for motor in chosen_slave.motors:
                                            if motor is not None:
                                                await motor.set_control_mode(ControlMode.MIT)
                                        if chosen_master is not None:
                                            for motor in chosen_master.motors:
                                                if motor is not None:
                                                    await motor.set_control_mode(ControlMode.MIT)
                                        raw_print(f"    Restored MIT control mode\n")

                                    if chosen_slave is not None and chosen_slave.channel in _slave_cmd_pos:
                                        _slave_cmd_pos[chosen_slave.channel] = chosen_slave.get_positions()

                                    side_sp = impedance_setpoints.get(aruco_arm, {})
                                    tcp_now, quat_now = gravity_comp.forward_kinematics(
                                        chosen_slave.get_positions()[:7], position=aruco_arm,
                                    )
                                    for axis in (0, 1, 2):
                                        if axis in side_sp:
                                            side_sp[axis] = float(tcp_now[axis])
                                    if impedance_lock_ori.get(aruco_arm, False):
                                        impedance_ori_quats[aruco_arm] = quat_now.copy()
                                    raw_print(f"    Resuming teleop\n")
                                else:
                                    raw_print(f"    Sequence aborted\n")
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
                    elif i == 3:
                        # Button 4: toggle FoundationPose register (true/false)
                        reg_val = pose_listener.send_register()
                        raw_print(f"\n  Button 4: sent register={reg_val}\n")
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
                    for idx, (motor, state) in enumerate(
                        zip(master_arm.motors, master_arm.states)
                    ):
                        if motor is not None and state:
                            active_positions.append(state.position)
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

            # Pre-compute gravity (and optional impedance) for each slave arm
            slave_gravity: dict[str, tuple] = {}
            slave_imp_torques_map: dict[str, list[float] | None] = {}
            _s_gc = slave_gravity_comp or gravity_comp
            for s_arm in [arm for arm in arms if arm.is_slave]:
                grav = None
                imp = None
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

                        if slave_impedance_enabled and s_arm.follows and s_arm.follows in master_arms:
                            m_arm = master_arms[s_arm.follows]
                            m_positions = m_arm.get_positions()
                            if m_positions:
                                m_pos, m_quat = gravity_comp.forward_kinematics(
                                    m_positions[:7], position=s_arm.position,
                                )
                                imp = _s_gc.impedance_torques_6d(
                                    s_positions,
                                    target_pos=m_pos,
                                    target_quat=m_quat,
                                    position=s_arm.position,
                                    trans_stiffness=slave_imp_trans_stiffness,
                                    rot_stiffness=slave_imp_rot_stiffness,
                                )

                slave_gravity[s_arm.channel] = (grav, s_active)
                slave_imp_torques_map[s_arm.channel] = imp

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
                if s_arm.channel not in _slave_cmd_pos:
                    _slave_cmd_pos[s_arm.channel] = s_arm.get_positions()
                cmd = _slave_cmd_pos[s_arm.channel]
                max_delta = SLAVE_MAX_VEL * loop_time if loop_time > 0 else 0.0025

                grav_torques, grav_indices = slave_gravity.get(s_arm.channel, (None, []))
                imp_torques = slave_imp_torques_map.get(s_arm.channel)
                use_imp = slave_impedance_enabled and imp_torques is not None

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

                        if use_imp and idx < 7 and idx in grav_indices:
                            gi = grav_indices.index(idx)
                            if gi < len(imp_torques):
                                torque += imp_torques[gi]
                            kp *= slave_imp_kp_scale
                            kd *= slave_imp_kp_scale
                        
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
        "--cam-extrinsics",
        type=str,
        default=None,
        metavar="FILE",
        help=(
            "Path to ee_T_cam.yaml (produced by calibrate_extrinsics.py). "
            "Loads calibrated ee_T_cam transforms for left/right cameras."
        ),
    )
    parser.add_argument(
        "--servo-intrinsics",
        type=str,
        default="/home/hans/projects/openarm/new_cam/servo_calib.yaml",
        metavar="FILE",
        help=(
            "Path to servo_calib.yaml (Kalibr pinhole format). "
            "Enables ArUco detection on the servo camera stream."
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
