"""SE(3) transform utilities using MuJoCo quaternion convention [w, x, y, z]."""

from __future__ import annotations

import mujoco
import numpy as np


def pose_to_T(pos: np.ndarray, quat_wxyz: np.ndarray) -> np.ndarray:
    """Build a 4x4 homogeneous transform from position + MuJoCo quaternion [w,x,y,z]."""
    T = np.eye(4)
    T[:3, 3] = pos
    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, quat_wxyz)
    T[:3, :3] = R.reshape(3, 3)
    return T


def T_inv(T: np.ndarray) -> np.ndarray:
    """Proper SE(3) inverse — uses R^T instead of general matrix inversion."""
    R = T[:3, :3]
    t = T[:3, 3]
    out = np.eye(4)
    out[:3, :3] = R.T
    out[:3, 3] = -R.T @ t
    return out


def ros_pose_to_T(pose_msg) -> np.ndarray:
    """Build a 4x4 transform from a ROS geometry_msgs/Pose."""
    T = np.eye(4)
    p = pose_msg.position
    o = pose_msg.orientation
    T[:3, 3] = [p.x, p.y, p.z]
    quat_wxyz = np.array([o.w, o.x, o.y, o.z])
    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, quat_wxyz)
    T[:3, :3] = R.reshape(3, 3)
    return T


def T_to_pos_quat(T: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Extract position and MuJoCo quaternion [w,x,y,z] from a 4x4 transform."""
    pos = T[:3, 3].copy()
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, T[:3, :3].flatten())
    return pos, quat


def quat_to_rpy_deg(q_wxyz: np.ndarray) -> np.ndarray:
    """Convert MuJoCo quaternion [w,x,y,z] to roll/pitch/yaw in degrees."""
    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, q_wxyz)
    R = R.reshape(3, 3)
    pitch = np.arcsin(-np.clip(R[2, 0], -1, 1))
    if np.abs(R[2, 0]) < 0.9999:
        roll = np.arctan2(R[2, 1], R[2, 2])
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll = np.arctan2(-R[1, 2], R[1, 1])
        yaw = 0.0
    return np.degrees([roll, pitch, yaw])


def quat_from_rpy_deg(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    """Build a MuJoCo quaternion [w,x,y,z] from roll/pitch/yaw in degrees (XYZ extrinsic)."""
    r, p, y = np.radians([roll_deg, pitch_deg, yaw_deg])
    cr, sr = np.cos(r / 2), np.sin(r / 2)
    cp, sp = np.cos(p / 2), np.sin(p / 2)
    cy, sy = np.cos(y / 2), np.sin(y / 2)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y_ = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return np.array([w, x, y_, z])


def quat_multiply(q1_wxyz: np.ndarray, q2_wxyz: np.ndarray) -> np.ndarray:
    """Hamilton product of two MuJoCo quaternions [w,x,y,z]. Returns q1 * q2."""
    result = np.empty(4)
    mujoco.mju_mulQuat(result, np.asarray(q1_wxyz, dtype=np.float64),
                       np.asarray(q2_wxyz, dtype=np.float64))
    return result
