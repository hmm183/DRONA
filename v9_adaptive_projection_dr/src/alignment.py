"""
v9_adaptive_projection_dr/src/alignment.py
-------------------------------------------
Stage 8: Coordinate Alignment & Phone-to-Vehicle Frame Calibration.

Handles arbitrary smartphone orientation inside the vehicle:
1. Static gravity vector extraction during stationary periods.
2. Principal Component Analysis (PCA) on horizontal acceleration plane during
   longitudinal motion to align vehicle forward and lateral axes.
3. Continuous mount-shift detector (>5 deg shift sustained for >2 s).
4. Alignment matrix R_phone_to_veh to transform raw IMU to [a_fwd, a_lat, a_vert]
   and [w_roll, w_pitch, w_yaw].
"""

from __future__ import annotations

import numpy as np
from typing import Optional, Tuple, Dict, Any


class PhoneVehicleAligner:
    """
    Learns and maintains the SO(3) rotation matrix R_phone_to_veh that transforms
    raw smartphone IMU observations into vehicle coordinate frames.
    """

    def __init__(self, sample_rate_hz: float = 10.0):
        self.sample_rate_hz = sample_rate_hz
        self.dt = 1.0 / sample_rate_hz
        self.R_phone_to_veh: np.ndarray = np.eye(3, dtype=np.float64)
        self.is_calibrated: bool = False

        # Reference gravity vector in phone frame
        self.g_ref: Optional[np.ndarray] = None

        # Mount-shift monitoring: >5 deg (0.0872 rad) for >2 seconds (20 samples)
        self.mount_shift_angle_thresh_rad: float = np.radians(5.0)
        self.mount_shift_dur_thresh_samples: int = int(2.0 * sample_rate_hz)
        self.shift_counter: int = 0
        self.mount_shift_detected: bool = False

        # Buffers for stationary calibration
        self.stationary_acc_buf: list[np.ndarray] = []
        self.motion_acc_buf: list[np.ndarray] = []

    def reset(self):
        """Resets the aligner to uncalibrated identity state."""
        self.R_phone_to_veh = np.eye(3, dtype=np.float64)
        self.is_calibrated = False
        self.g_ref = None
        self.shift_counter = 0
        self.mount_shift_detected = False
        self.stationary_acc_buf.clear()
        self.motion_acc_buf.clear()

    def update_calibration(self, acc_phone: np.ndarray, gyro_phone: np.ndarray,
                           speed_mps: float = 0.0) -> bool:
        """
        Updates calibration estimates based on current motion regime.
        - If stationary (speed < 0.1 m/s, ||gyro|| < 0.05 rad/s, ||acc|| in [9.0, 10.6]):
          accumulates gravity vector.
        - If accelerating/cruising:
          accumulates horizontal dynamics for PCA.
        """
        acc_norm = np.linalg.norm(acc_phone)
        gyro_norm = np.linalg.norm(gyro_phone)

        # Stationary gravity identification
        if speed_mps < 0.1 and gyro_norm < 0.05 and (9.0 <= acc_norm <= 10.6):
            self.stationary_acc_buf.append(acc_phone)
            if len(self.stationary_acc_buf) > 50:
                self.stationary_acc_buf.pop(0)

            # Recompute gravity vector if enough stationary samples
            if len(self.stationary_acc_buf) >= 10:
                g_mean = np.mean(self.stationary_acc_buf, axis=0)
                self.g_ref = g_mean / np.linalg.norm(g_mean)

        # Check for mount shift if already calibrated
        if self.g_ref is not None:
            if speed_mps < 0.1 and gyro_norm < 0.05 and (9.0 <= acc_norm <= 10.6):
                cur_g = acc_phone / acc_norm
                cos_ang = np.clip(np.dot(self.g_ref, cur_g), -1.0, 1.0)
                angle_diff = np.arccos(cos_ang)
                if angle_diff > self.mount_shift_angle_thresh_rad:
                    self.shift_counter += 1
                    if self.shift_counter >= self.mount_shift_dur_thresh_samples:
                        self.mount_shift_detected = True
                else:
                    self.shift_counter = max(0, self.shift_counter - 1)
                    self.mount_shift_detected = False

        # Accumulate motion samples for horizontal PCA alignment
        if speed_mps > 2.0 and acc_norm > 0.3:
            self.motion_acc_buf.append(acc_phone)
            if len(self.motion_acc_buf) > 100:
                self.motion_acc_buf.pop(0)

        # Compute full R_phone_to_veh if we have gravity and motion samples
        if self.g_ref is not None and len(self.motion_acc_buf) >= 20:
            self._compute_alignment_matrix()

        return self.is_calibrated

    def _compute_alignment_matrix(self):
        """
        Computes orthonormal rotation matrix R_phone_to_veh such that:
        - z_veh is aligned with vertical (anti-gravity: [0, 0, 1])
        - x_veh is aligned with forward longitudinal acceleration
        - y_veh completes right-handed system (lateral: y = z x x)
        """
        # Vertical axis z_veh in phone frame is opposite to gravity
        z_phone = -self.g_ref
        z_phone = z_phone / np.linalg.norm(z_phone)

        # Project motion accelerations onto horizontal plane
        proj_motion = []
        for a in self.motion_acc_buf:
            a_vert = np.dot(a, z_phone) * z_phone
            a_horiz = a - a_vert
            if np.linalg.norm(a_horiz) > 0.2:
                proj_motion.append(a_horiz)

        if len(proj_motion) < 10:
            return

        proj_arr = np.array(proj_motion)
        # PCA via SVD on horizontal components
        cov = np.dot(proj_arr.T, proj_arr) / len(proj_arr)
        eigvals, eigvecs = np.linalg.eigh(cov)
        x_phone = eigvecs[:, -1]  # Dominant longitudinal axis
        x_phone = x_phone / np.linalg.norm(x_phone)

        # Ensure x_phone is orthogonal to z_phone
        x_phone = x_phone - np.dot(x_phone, z_phone) * z_phone
        x_phone = x_phone / np.linalg.norm(x_phone)

        # Lateral axis y_phone
        y_phone = np.cross(z_phone, x_phone)
        y_phone = y_phone / np.linalg.norm(y_phone)

        # Construct rotation matrix (rows are the new vehicle coordinate axes in phone frame)
        self.R_phone_to_veh = np.vstack([x_phone, y_phone, z_phone])
        self.is_calibrated = True

    def align_imu(self, acc_phone: np.ndarray, gyro_phone: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Transforms raw phone IMU vectors into vehicle coordinates:
        acc_veh = [a_fwd, a_lat, a_vert]
        gyro_veh = [w_roll, w_pitch, w_yaw]
        """
        acc_veh = np.dot(self.R_phone_to_veh, acc_phone)
        gyro_veh = np.dot(self.R_phone_to_veh, gyro_phone)
        return acc_veh, gyro_veh
