from aprilgrid import Detector
import numpy as np
import os
from glob import glob
import yaml
import cv2
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class CalibrateExtrinsics:
    def __init__(self, cal_folder: str):
        self.cal_folder = cal_folder
        self.left_images, self.right_images, self.right_arm_T_ee, self.left_arm_T_ee, self.body_T_left_arm, self.body_T_right_arm = self.load_calibration()
        self.left_K, self.left_D, self.right_K, self.right_D = self.load_intrinsics()
        self.tag_rows, self.tag_cols, self.tag_size, self.tag_spacing, self.object_points = self.load_april_grid()
        self.detector = Detector("t36h11")
        print(f"Found {len(self.left_images)} left images and {len(self.right_images)} right images")
        print(f"AprilGrid: {self.tag_rows}x{self.tag_cols}, tag_size={self.tag_size}m, "
              f"{len(self.object_points)} tags with 3D points")
    
    def load_calibration(self):
        with open(os.path.join(self.cal_folder, "calibration.yaml"), "r") as f:
            calibration = yaml.safe_load(f)
        left_images = []
        right_images = []
        right_arm_T_ee = []
        left_arm_T_ee = []
        for wp in calibration["waypoints"]:
            left_image_path = os.path.join(self.cal_folder, wp["left_image"])
            right_image_path = os.path.join(self.cal_folder, wp["right_image"])
            left_images.append(cv2.imread(left_image_path, cv2.IMREAD_GRAYSCALE))
            right_images.append(cv2.imread(right_image_path, cv2.IMREAD_GRAYSCALE))
            right_arm_T_ee.append(wp["right_arm_T_ee"])
            left_arm_T_ee.append(wp["left_arm_T_ee"])
        body_T_left_arm = np.array(calibration["body_T_left_arm"])
        body_T_right_arm = np.array(calibration["body_T_right_arm"])
        return left_images, right_images, right_arm_T_ee, left_arm_T_ee, body_T_left_arm, body_T_right_arm

    @staticmethod
    def _parse_pinhole(cam_dict):
        fx, fy, cx, cy = cam_dict["intrinsics"]
        K = np.array([[fx, 0, cx],
                       [0, fy, cy],
                       [0,  0,  1]], dtype=np.float64)
        D = np.array(cam_dict["distortion_coeffs"], dtype=np.float64)
        return K, D

    def load_intrinsics(self):
        left_path = os.path.join(self.cal_folder, "d405_left_stereo_calib.yaml")
        right_path = os.path.join(self.cal_folder, "d405_right_stereo_calib.yaml")
        with open(left_path) as f:
            left_calib = yaml.safe_load(f)
        with open(right_path) as f:
            right_calib = yaml.safe_load(f)
        left_K, left_D = self._parse_pinhole(left_calib["cam0"])
        right_K, right_D = self._parse_pinhole(right_calib["cam0"])
        print(f"Loaded left intrinsics:\n{left_K}\n  dist: {left_D}")
        print(f"Loaded right intrinsics:\n{right_K}\n  dist: {right_D}")
        return left_K, left_D, right_K, right_D

    def load_april_grid(self):
        with open(os.path.join(self.cal_folder, "april.yaml")) as f:
            cfg = yaml.safe_load(f)
        tag_rows = cfg["tagRows"]
        tag_cols = cfg["tagCols"]
        tag_size = cfg["tagSize"]
        tag_spacing = cfg["tagSpacing"]
        step = tag_size * (1.0 + tag_spacing)

        # Kalibr convention: tag_id = row * tagCols + col, origin at bottom-left.
        # Each tag has 4 corners ordered: bottom-left, bottom-right, top-right, top-left.
        object_points = {}
        for row in range(tag_rows):
            for col in range(tag_cols):
                tag_id = row * tag_cols + col
                ox = col * step
                oy = row * step
                object_points[tag_id] = np.array([
                    [ox,            oy,            0.0],
                    [ox + tag_size, oy,            0.0],
                    [ox + tag_size, oy + tag_size, 0.0],
                    [ox,            oy + tag_size, 0.0],
                ], dtype=np.float64)
        print(f"Loaded {len(object_points)} object points")
        print(f"Object points: {object_points}")
        return tag_rows, tag_cols, tag_size, tag_spacing, object_points

    def _safe_detect(self, img):
        """Run detector, falling back to padded image if corners are out of bounds."""
        try:
            return self.detector.detect(img)
        except cv2.error:
            pass
        for pad in (50, 100, 200):
            try:
                padded = cv2.copyMakeBorder(img, pad, pad, pad, pad,
                                            cv2.BORDER_REPLICATE)
                detections = self.detector.detect(padded)
                for det in detections:
                    det.corners -= pad
                return detections
            except cv2.error:
                continue
        print("    WARNING: detection failed even with padding, skipping")
        return []

    def detect_april_tags(self, images):
        all_detections = []
        for i, img in enumerate(images):
            detections = self._safe_detect(img)
            print(f"Image {i}: {img.shape}, found {len(detections)} tags")
            all_detections.append(detections)

            vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            for det in detections:
                pts = det.corners.astype(int).reshape((-1, 1, 2))
                cv2.polylines(vis, [pts], isClosed=True, color=(0, 0, 255), thickness=2)
                center = det.corners.reshape(-1, 2).mean(axis=0).astype(int)
                cv2.putText(vis, str(det.tag_id), (int(center[0]), int(center[1])),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.imshow("Detections", vis)
            cv2.waitKey(100)
        cv2.destroyAllWindows()
        return all_detections

    def estimate_board_pose(self, detections, K, D):
        """Estimate cam_T_board via solvePnP. Returns (T, mean_reproj_px)."""
        obj_pts = []
        img_pts = []
        for det in detections:
            tid = det.tag_id
            if tid not in self.object_points:
                continue
            obj_pts.append(self.object_points[tid])
            img_pts.append(det.corners.reshape(-1, 2))
        if len(obj_pts) < 2:
            return None, float("inf")
        obj_pts = np.vstack(obj_pts)
        img_pts = np.vstack(img_pts).astype(np.float64)

        ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, K, D,
                                       flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            return None, float("inf")

        proj, _ = cv2.projectPoints(obj_pts, rvec, tvec, K, D)
        reproj_err = float(np.linalg.norm(proj.reshape(-1, 2) - img_pts, axis=1).mean())

        R, _ = cv2.Rodrigues(rvec)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = tvec.flatten()
        return T, reproj_err

    def calibrate_hand_eye(self, arm_T_ee_list, cam_T_board_list):
        """Solve ee_T_cam via AX=XB hand-eye calibration.

        arm_T_ee @ ee_T_cam @ cam_T_board = arm_T_board  (constant)
        """
        R_g2b, t_g2b = [], []
        R_t2c, t_t2c = [], []
        for arm_T_ee_raw, cam_T_board in zip(arm_T_ee_list, cam_T_board_list):
            if cam_T_board is None:
                continue
            T = np.array(arm_T_ee_raw, dtype=np.float64)
            R_g2b.append(T[:3, :3])
            t_g2b.append(T[:3, 3].reshape(3, 1))
            R_t2c.append(cam_T_board[:3, :3])
            t_t2c.append(cam_T_board[:3, 3].reshape(3, 1))

        n = len(R_g2b)
        print(f"  Hand-eye: using {n} valid pose pairs")
        if n < 3:
            raise RuntimeError("Need >= 3 valid pose pairs for hand-eye calibration")

        methods = {
            "TSAI":      cv2.CALIB_HAND_EYE_TSAI,
            "PARK":      cv2.CALIB_HAND_EYE_PARK,
            "HORAUD":    cv2.CALIB_HAND_EYE_HORAUD,
            "ANDREFF":   cv2.CALIB_HAND_EYE_ANDREFF,
            "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
        }
        results = {}
        for name, method in methods.items():
            R, t = cv2.calibrateHandEye(R_g2b, t_g2b, R_t2c, t_t2c, method=method)
            ee_T_cam = np.eye(4)
            ee_T_cam[:3, :3] = R
            ee_T_cam[:3, 3] = t.flatten()
            results[name] = ee_T_cam
            print(f"  {name}: t = {t.flatten()}")
        return results

    @staticmethod
    def _board_origin_std(arm_T_ee_list, cam_T_board_list, ee_T_cam):
        """Compute std of board origins projected into arm frame."""
        origins = []
        for ate_raw, ctb in zip(arm_T_ee_list, cam_T_board_list):
            if ctb is None:
                continue
            arm_T_ee = np.array(ate_raw, dtype=np.float64)
            arm_T_board = arm_T_ee @ ee_T_cam @ ctb
            origins.append(arm_T_board[:3, 3])
        origins = np.array(origins)
        return np.linalg.norm(np.std(origins, axis=0))

    def calibrate_hand_eye_robust(self, arm_T_ee_list, cam_T_board_list,
                                   origin_thresh_mm=25.0, min_pairs=6,
                                   method_flag=None):
        """Hand-eye calibration with iterative board-origin outlier rejection.

        Solves, finds the worst waypoint by board origin deviation from the
        mean, removes it, and re-solves until all residuals are below the
        threshold or we hit min_pairs.
        """
        if method_flag is None:
            method_flag = cv2.CALIB_HAND_EYE_PARK

        ate_list = list(arm_T_ee_list)
        ctb_list = list(cam_T_board_list)
        n_total = sum(1 for c in ctb_list if c is not None)
        removed = []

        while True:
            valid = [(i, ate_list[i], ctb_list[i])
                     for i in range(len(ate_list)) if ctb_list[i] is not None]
            if len(valid) < min_pairs:
                break

            R_g, t_g, R_t, t_t = [], [], [], []
            for _, ate_raw, ctb in valid:
                T = np.array(ate_raw, dtype=np.float64)
                R_g.append(T[:3, :3])
                t_g.append(T[:3, 3].reshape(3, 1))
                R_t.append(ctb[:3, :3])
                t_t.append(ctb[:3, 3].reshape(3, 1))

            R, t = cv2.calibrateHandEye(R_g, t_g, R_t, t_t, method=method_flag)
            ee_T_cam = np.eye(4)
            ee_T_cam[:3, :3] = R
            ee_T_cam[:3, 3] = t.flatten()

            # Compute per-waypoint board origin
            origins = []
            for _, ate_raw, ctb in valid:
                arm_T_ee = np.array(ate_raw, dtype=np.float64)
                origins.append((arm_T_ee @ ee_T_cam @ ctb)[:3, 3])
            origins = np.array(origins)
            mean_origin = origins.mean(axis=0)
            dists_mm = np.linalg.norm(origins - mean_origin, axis=1) * 1000

            worst_idx_in_valid = int(np.argmax(dists_mm))
            worst_dist = dists_mm[worst_idx_in_valid]
            worst_wp = valid[worst_idx_in_valid][0]

            if worst_dist <= origin_thresh_mm:
                break

            print(f"    Removing wp {worst_wp} (origin err = {worst_dist:.1f} mm)")
            ctb_list[worst_wp] = None
            removed.append(worst_wp)

        n_kept = sum(1 for c in ctb_list if c is not None)
        print(f"    Robust: kept {n_kept}/{n_total} waypoints "
              f"(removed {len(removed)}: {removed})")
        return ee_T_cam, ctb_list

    @staticmethod
    def _draw_frame(ax, T, length=0.05, lw=1.5):
        """Draw a coordinate frame at pose T on a 3D axis."""
        o = T[:3, 3]
        for i, c in enumerate(["r", "g", "b"]):
            d = T[:3, i] * length
            ax.quiver(*o, *d, color=c, linewidth=lw, arrow_length_ratio=0.15)

    def plot_board_poses(self, side, arm_T_ee_list, cam_T_board_list, ee_T_cam,
                         body_T_arm=None):
        """Plot estimated board poses in arm-base frame to verify consistency."""
        fig = plt.figure(figsize=(14, 6))
        fig.suptitle(f"{side} camera — board poses in arm frame")

        ax_arm = fig.add_subplot(121, projection="3d")
        ax_arm.set_title("Arm frame")

        board_origins = []
        for i, (ate_raw, ctb) in enumerate(zip(arm_T_ee_list, cam_T_board_list)):
            if ctb is None:
                continue
            arm_T_ee = np.array(ate_raw, dtype=np.float64)
            arm_T_board = arm_T_ee @ ee_T_cam @ ctb
            board_origins.append(arm_T_board[:3, 3])
            self._draw_frame(ax_arm, arm_T_board, length=0.03)
            ax_arm.text(*arm_T_board[:3, 3], f"{i}", fontsize=7)

        if not board_origins:
            print(f"  No valid poses to plot for {side}")
            return

        board_origins = np.array(board_origins)
        spread = np.std(board_origins, axis=0)
        mean = np.mean(board_origins, axis=0)
        print(f"  {side} board origin mean: {mean}")
        print(f"  {side} board origin std:  {spread}  (total: {np.linalg.norm(spread):.4f} m)")

        ax_arm.set_xlabel("X")
        ax_arm.set_ylabel("Y")
        ax_arm.set_zlabel("Z")
        _set_equal_aspect(ax_arm, board_origins)

        ax_body = fig.add_subplot(122, projection="3d")
        ax_body.set_title("Body frame")
        if body_T_arm is None:
            body_T_arm = np.eye(4)
        body_origins = []
        for i, (ate_raw, ctb) in enumerate(zip(arm_T_ee_list, cam_T_board_list)):
            if ctb is None:
                continue
            arm_T_ee = np.array(ate_raw, dtype=np.float64)
            body_T_board = body_T_arm @ arm_T_ee @ ee_T_cam @ ctb
            body_origins.append(body_T_board[:3, 3])
            self._draw_frame(ax_body, body_T_board, length=0.03)
            ax_body.text(*body_T_board[:3, 3], f"{i}", fontsize=7)
        body_origins = np.array(body_origins)
        ax_body.set_xlabel("X")
        ax_body.set_ylabel("Y")
        ax_body.set_zlabel("Z")
        _set_equal_aspect(ax_body, body_origins)

        plt.tight_layout()


    def plot_reprojection_errors(self, side, detections_list, arm_T_ee_list,
                                    cam_T_board_list, ee_T_cam, K, D):
        """Plot per-waypoint reprojection error through the full calibrated chain.

        For each waypoint, compute the mean board pose in arm frame from all
        other waypoints, project 3D corners through
        arm_T_ee^-1 @ arm_T_board_mean into the camera, and compare to
        detected 2D corners.  Also shows the direct PnP reprojection error
        for comparison.
        """
        # -- Compute mean board pose in arm frame ---
        arm_T_boards = []
        valid_indices = []
        for i, (ate_raw, ctb) in enumerate(zip(arm_T_ee_list, cam_T_board_list)):
            if ctb is None:
                arm_T_boards.append(None)
                continue
            arm_T_ee = np.array(ate_raw, dtype=np.float64)
            arm_T_boards.append(arm_T_ee @ ee_T_cam @ ctb)
            valid_indices.append(i)

        # Mean board pose (average translation, use first valid rotation as ref)
        valid_origins = np.array([arm_T_boards[i][:3, 3] for i in valid_indices])
        mean_origin = valid_origins.mean(axis=0)

        pnp_errors = []
        chain_errors = []
        wp_indices = []

        for i, (dets, ate_raw, ctb) in enumerate(
                zip(detections_list, arm_T_ee_list, cam_T_board_list)):
            if ctb is None:
                pnp_errors.append(0)
                chain_errors.append(0)
                wp_indices.append(i)
                continue

            # Collect matched 3D/2D points for this waypoint
            obj_pts, img_pts = [], []
            for det in dets:
                tid = det.tag_id
                if tid not in self.object_points:
                    continue
                obj_pts.append(self.object_points[tid])
                img_pts.append(det.corners.reshape(-1, 2))
            if not obj_pts:
                pnp_errors.append(0)
                chain_errors.append(0)
                wp_indices.append(i)
                continue
            obj_pts = np.vstack(obj_pts)
            img_pts = np.vstack(img_pts).astype(np.float64)

            # Direct PnP reprojection error
            rvec_pnp, _ = cv2.Rodrigues(ctb[:3, :3])
            tvec_pnp = ctb[:3, 3].reshape(3, 1)
            proj_pnp, _ = cv2.projectPoints(obj_pts, rvec_pnp, tvec_pnp, K, D)
            err_pnp = np.linalg.norm(proj_pnp.reshape(-1, 2) - img_pts, axis=1)
            pnp_errors.append(float(err_pnp.mean()))

            # Chain reprojection: implied cam_T_board from calibrated chain + mean board
            arm_T_ee = np.array(ate_raw, dtype=np.float64)
            # Use per-waypoint arm_T_board but with mean origin to measure consistency
            arm_T_board_i = arm_T_boards[i]
            origin_err = np.linalg.norm(arm_T_board_i[:3, 3] - mean_origin) * 1000
            chain_errors.append(float(origin_err))

            wp_indices.append(i)

        # -- Plot ---
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
        fig.suptitle(f"{side} camera — per-waypoint diagnostics")

        ax1.bar(wp_indices, pnp_errors, color="steelblue", alpha=0.8)
        ax1.set_ylabel("PnP reproj error (px)")
        ax1.set_title("Direct PnP reprojection error (lower = better detection)")
        ax1.axhline(np.mean([e for e in pnp_errors if e > 0]), color="red",
                     ls="--", label=f"mean={np.mean([e for e in pnp_errors if e > 0]):.2f}px")
        ax1.legend()
        ax1.grid(axis="y", alpha=0.3)

        ax2.bar(wp_indices, chain_errors, color="darkorange", alpha=0.8)
        ax2.set_ylabel("Board origin error (mm)")
        ax2.set_xlabel("Waypoint index")
        ax2.set_title("Board origin distance from mean (lower = better FK+vision agreement)")
        ax2.axhline(np.mean([e for e in chain_errors if e > 0]), color="red",
                     ls="--", label=f"mean={np.mean([e for e in chain_errors if e > 0]):.1f}mm")
        ax2.legend()
        ax2.grid(axis="y", alpha=0.3)

        plt.tight_layout()

        # Print worst waypoints
        sorted_by_chain = sorted(zip(wp_indices, chain_errors), key=lambda x: -x[1])
        print(f"\n  {side} — worst waypoints by board origin error:")
        for idx, err in sorted_by_chain[:5]:
            pnp_e = pnp_errors[wp_indices.index(idx)]
            print(f"    wp {idx}: origin err = {err:.1f} mm, PnP reproj = {pnp_e:.2f} px")

    @staticmethod
    def _mj_quat_to_R(w, x, y, z):
        """MuJoCo quaternion (w,x,y,z) to 3x3 rotation matrix."""
        return np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
            [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
            [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)],
        ])

    @classmethod
    def _mj_to_T(cls, pos, quat_wxyz):
        """Build 4x4 transform from MuJoCo pos + quat (w,x,y,z)."""
        T = np.eye(4)
        T[:3, :3] = cls._mj_quat_to_R(*quat_wxyz)
        T[:3, 3] = pos
        return T

    @classmethod
    def cad_ee_T_cam(cls):
        """Compute ee_T_cam from the MuJoCo XML CAD model.

        FK endpoint is hand_tcp.  Both hand_tcp and the camera body are
        children of the hand body, so ee_T_cam = inv(hand_T_tcp) @ hand_T_cam.
        The mounting is identical for left and right.
        """
        hand_T_tcp = cls._mj_to_T([0, 0, 0.08], [1, 0, 0, 0])
        hand_T_cam = cls._mj_to_T(
            [0.065, 0, 0.02],
            [0.6711512, -0.222612, -0.222612, 0.6711512],
        )
        return np.linalg.inv(hand_T_tcp) @ hand_T_cam

    @staticmethod
    def _rotation_angle_deg(R1, R2):
        """Geodesic angle (degrees) between two rotation matrices."""
        R = R1.T @ R2
        cos_a = np.clip((np.trace(R) - 1) / 2, -1, 1)
        return np.degrees(np.arccos(cos_a))

    @staticmethod
    def _R_to_rpy_deg(R):
        """Extract roll-pitch-yaw (XYZ extrinsic) from rotation matrix, in degrees."""
        sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
        singular = sy < 1e-6
        if not singular:
            roll  = np.arctan2(R[2, 1], R[2, 2])
            pitch = np.arctan2(-R[2, 0], sy)
            yaw   = np.arctan2(R[1, 0], R[0, 0])
        else:
            roll  = np.arctan2(-R[1, 2], R[1, 1])
            pitch = np.arctan2(-R[2, 0], sy)
            yaw   = 0.0
        return np.degrees([roll, pitch, yaw])

    def plot_cad_comparison(self, ee_T_cam_left, ee_T_cam_right):
        """Compare calibrated ee_T_cam against the CAD model values."""
        cad = self.cad_ee_T_cam()
        print("\n--- CAD vs calibrated ee_T_cam ---")
        print(f"  CAD ee_T_cam (same for both sides):\n{cad}\n")

        cad_rpy = self._R_to_rpy_deg(cad[:3, :3])
        print(f"  CAD   rpy: [{cad_rpy[0]:.2f}, {cad_rpy[1]:.2f}, {cad_rpy[2]:.2f}] deg")
        print(f"  CAD   t:   {cad[:3,3]}\n")

        for label, cal_T in [("left", ee_T_cam_left), ("right", ee_T_cam_right)]:
            dt = np.linalg.norm(cal_T[:3, 3] - cad[:3, 3])
            dr = self._rotation_angle_deg(cal_T[:3, :3], cad[:3, :3])
            cal_rpy = self._R_to_rpy_deg(cal_T[:3, :3])
            rpy_err = np.array(cal_rpy) - np.array(cad_rpy)
            print(f"  {label}:")
            print(f"    translation error: {dt*1000:.1f} mm")
            print(f"    rotation error:    {dr:.2f} deg (geodesic)")
            print(f"    rpy error:         [{rpy_err[0]:.2f}, {rpy_err[1]:.2f}, {rpy_err[2]:.2f}] deg")
            print(f"    CAD   t: {cad[:3,3]}")
            print(f"    calib t: {cal_T[:3,3]}")
            print(f"    CAD   rpy: [{cad_rpy[0]:.2f}, {cad_rpy[1]:.2f}, {cad_rpy[2]:.2f}] deg")
            print(f"    calib rpy: [{cal_rpy[0]:.2f}, {cal_rpy[1]:.2f}, {cal_rpy[2]:.2f}] deg")

        fig = plt.figure(figsize=(8, 6))
        ax = fig.add_subplot(111, projection="3d")
        ax.set_title("ee_T_cam: CAD vs calibrated")

        self._draw_frame(ax, cad, length=0.03, lw=2)
        ax.text(*cad[:3, 3], "CAD", fontsize=9, color="black")

        self._draw_frame(ax, ee_T_cam_left, length=0.02, lw=1)
        ax.text(*ee_T_cam_left[:3, 3], "left (cal)", fontsize=8, color="purple")

        self._draw_frame(ax, ee_T_cam_right, length=0.02, lw=1)
        ax.text(*ee_T_cam_right[:3, 3], "right (cal)", fontsize=8, color="orange")

        pts = np.array([cad[:3, 3], ee_T_cam_left[:3, 3], ee_T_cam_right[:3, 3]])
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        _set_equal_aspect(ax, pts)
        plt.tight_layout()


def _set_equal_aspect(ax, points):
    """Set equal aspect ratio for a 3D axis based on point cloud."""
    center = points.mean(axis=0)
    max_range = (points.max(axis=0) - points.min(axis=0)).max() / 2.0
    max_range = max(max_range, 0.05)
    ax.set_xlim(center[0] - max_range, center[0] + max_range)
    ax.set_ylim(center[1] - max_range, center[1] + max_range)
    ax.set_zlim(center[2] - max_range, center[2] + max_range)


if __name__ == "__main__":
    cal = CalibrateExtrinsics("/home/hans/projects/openarm/calib_longer")

    print("\nDetecting tags — left images")
    left_dets = cal.detect_april_tags(cal.left_images)
    print("\nDetecting tags — right images")
    right_dets = cal.detect_april_tags(cal.right_images)

    print("\nEstimating board poses (PnP) ...")
    REPROJ_THRESH_PX = 0.5
    left_cam_T_board, left_reproj = zip(*[cal.estimate_board_pose(d, cal.left_K, cal.left_D)
                                           for d in left_dets])
    right_cam_T_board, right_reproj = zip(*[cal.estimate_board_pose(d, cal.right_K, cal.right_D)
                                             for d in right_dets])
    left_cam_T_board = list(left_cam_T_board)
    right_cam_T_board = list(right_cam_T_board)
    left_reproj = list(left_reproj)
    right_reproj = list(right_reproj)

    for i, (l, le, r, re) in enumerate(zip(left_cam_T_board, left_reproj,
                                            right_cam_T_board, right_reproj)):
        lstr = f"t={l[:3,3]} reproj={le:.2f}px" if l is not None else "FAIL"
        rstr = f"t={r[:3,3]} reproj={re:.2f}px" if r is not None else "FAIL"
        print(f"  wp {i}: left {lstr}  |  right {rstr}")

    # Filter out waypoints with high reprojection error
    n_before = sum(1 for t in left_cam_T_board if t is not None)
    for i in range(len(left_cam_T_board)):
        if left_reproj[i] > REPROJ_THRESH_PX:
            left_cam_T_board[i] = None
        if right_reproj[i] > REPROJ_THRESH_PX:
            right_cam_T_board[i] = None
    n_left = sum(1 for t in left_cam_T_board if t is not None)
    n_right = sum(1 for t in right_cam_T_board if t is not None)
    print(f"\n  Reproj filter (>{REPROJ_THRESH_PX}px): "
          f"left {n_before}→{n_left}, right {n_before}→{n_right}")

    # Robust hand-eye: iteratively remove worst FK outliers
    ORIGIN_THRESH_MM = 17.5
    print(f"\n--- Robust left hand-eye (origin thresh = {ORIGIN_THRESH_MM} mm) ---")
    ee_T_cam_left, left_cam_T_board = cal.calibrate_hand_eye_robust(
        cal.left_arm_T_ee, left_cam_T_board, origin_thresh_mm=ORIGIN_THRESH_MM)
    print(f"\n--- Robust right hand-eye (origin thresh = {ORIGIN_THRESH_MM} mm) ---")
    ee_T_cam_right, right_cam_T_board = cal.calibrate_hand_eye_robust(
        cal.right_arm_T_ee, right_cam_T_board, origin_thresh_mm=ORIGIN_THRESH_MM)

    # Now run all methods on the cleaned data to compare
    print("\n--- Left hand-eye calibration (cleaned) ---")
    left_results = cal.calibrate_hand_eye(cal.left_arm_T_ee, left_cam_T_board)
    print("\n--- Right hand-eye calibration (cleaned) ---")
    right_results = cal.calibrate_hand_eye(cal.right_arm_T_ee, right_cam_T_board)

    print("\n--- Method comparison (board origin std in meters) ---")
    print(f"  {'Method':<12} {'Left std':>10} {'Right std':>10}")
    print(f"  {'-'*12} {'-'*10} {'-'*10}")
    best_left_method, best_left_std = None, float("inf")
    best_right_method, best_right_std = None, float("inf")
    for method in left_results:
        l_std = cal._board_origin_std(cal.left_arm_T_ee, left_cam_T_board,
                                      left_results[method])
        r_std = cal._board_origin_std(cal.right_arm_T_ee, right_cam_T_board,
                                      right_results[method])
        print(f"  {method:<12} {l_std:>10.4f} {r_std:>10.4f}")
        if l_std < best_left_std:
            best_left_std = l_std
            best_left_method = method
        if r_std < best_right_std:
            best_right_std = r_std
            best_right_method = method

    print(f"\n  Best left:  {best_left_method} ({best_left_std:.4f} m)")
    print(f"  Best right: {best_right_method} ({best_right_std:.4f} m)")

    ee_T_cam_left = left_results[best_left_method]
    ee_T_cam_right = right_results[best_right_method]
    print(f"\n  ee_T_cam_left ({best_left_method}):\n{ee_T_cam_left}")
    print(f"\n  ee_T_cam_right ({best_right_method}):\n{ee_T_cam_right}")

    cal.plot_board_poses("left", cal.left_arm_T_ee, left_cam_T_board,
                         ee_T_cam_left, cal.body_T_left_arm)
    cal.plot_board_poses("right", cal.right_arm_T_ee, right_cam_T_board,
                         ee_T_cam_right, cal.body_T_right_arm)
    cal.plot_cad_comparison(ee_T_cam_left, ee_T_cam_right)

    cal.plot_reprojection_errors("left", left_dets, cal.left_arm_T_ee,
                                 left_cam_T_board, ee_T_cam_left,
                                 cal.left_K, cal.left_D)
    cal.plot_reprojection_errors("right", right_dets, cal.right_arm_T_ee,
                                 right_cam_T_board, ee_T_cam_right,
                                 cal.right_K, cal.right_D)

    # Save calibrated ee_T_cam to YAML
    out_path = os.path.join(cal.cal_folder, "ee_T_cam.yaml")
    ee_T_cam_data = {
        "left": {
            "ee_T_cam": ee_T_cam_left.tolist(),
            "method": best_left_method,
            "board_origin_std_m": float(best_left_std),
        },
        "right": {
            "ee_T_cam": ee_T_cam_right.tolist(),
            "method": best_right_method,
            "board_origin_std_m": float(best_right_std),
        },
    }
    with open(out_path, "w") as f:
        yaml.dump(ee_T_cam_data, f, default_flow_style=None, sort_keys=False)
    print(f"\nSaved ee_T_cam → {out_path}")

    plt.show()