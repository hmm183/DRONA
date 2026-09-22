"""
v9_adaptive_projection_dr/src/fusion_engine_v9.py
--------------------------------------------------
Stage 14: Error-State Extended Kalman Filter (ES-EKF) Engine (Fix #3).

Matches v3 physical coordinate & displacement dynamics:
  - Nominal state: x_nom = [x, y, s_step, psi, b_a, b_g]^T \in R^6
    where (x, y) are 2D local position coordinates (m),
    s_step is forward displacement per step (m),
    psi is heading angle (rad),
    b_a is step displacement bias (m),
    b_g is step heading bias (rad).
  - Error state: \delta x = [\delta x, \delta y, \delta s, \delta \psi, \delta b_a, \delta b_g]^T \in R^6
  - Projection measurement update (Fix #3):
      z_proj = \hat{\delta x}
      r = z_proj - H \delta x
      R(c) = R_0 / max(c, 0.05)
      K = P H^T (H P H^T + R)^-1
      \delta x = K r
      Inject into nominal state: x_nom <- x_nom + \delta x
      Joseph form covariance update: P <- (I - KH) P (I - KH)^T + K R K^T
      Reset: \delta x <- 0
  - ZUPT velocity update when stationary
"""

from __future__ import annotations

import numpy as np
from typing import Dict, Any, Tuple, Optional
from dataclasses import dataclass


@dataclass
class EKFState:
    pos_x: float
    pos_y: float
    speed_mps: float
    yaw_rad: float
    bias_acc: float
    bias_gyro: float
    covariance: np.ndarray

    @property
    def pos_e(self) -> float:
        return self.pos_x

    @property
    def pos_n(self) -> float:
        return self.pos_y


class ESEKFFusionEngine:
    """
    6-state Error-State Extended Kalman Filter for Dead Reckoning Fusion.
    """

    def __init__(self, dt: float = 0.1):
        self.dt = dt

        # Nominal state: [x, y, s_step, psi, b_a, b_g]
        self.x_nom = np.zeros(6, dtype=np.float64)

        # Error state: \delta x = [\delta x, \delta y, \delta s, \delta \psi, \delta b_a, \delta b_g]
        self.delta_x = np.zeros(6, dtype=np.float64)

        # Covariance matrix P (6x6)
        self.P = np.diag([
            1.0**2,              # x (1 m)
            1.0**2,              # y (1 m)
            0.1**2,              # step disp (0.1 m)
            np.radians(2.0)**2,  # psi (2 deg)
            0.01**2,             # b_a (disp bias)
            0.001**2,            # b_g (gyro bias)
        ]).astype(np.float64)

        # Process noise covariance Q (6x6)
        self.Q = np.diag([
            0.02**2,             # pos x noise
            0.02**2,             # pos y noise
            0.05**2,             # step disp noise
            np.radians(0.3)**2,  # heading noise
            1e-5**2,             # acc bias random walk
            1e-6**2,             # gyro bias random walk
        ]).astype(np.float64)

        # Base measurement covariance for projection R_0
        self.R_0 = np.diag([
            2.5**2,              # pos x (2.5 m)
            2.5**2,              # pos y (2.5 m)
            0.2**2,              # step disp (0.2 m)
            np.radians(2.5)**2,  # heading (2.5 deg)
            0.02**2,             # bias acc
            0.002**2,            # bias gyro
        ]).astype(np.float64)

    @staticmethod
    def wrap_angle(rad: float) -> float:
        return float((rad + np.pi) % (2.0 * np.pi) - np.pi)

    def initialize(self, init_x: float, init_y: float, init_disp: float, init_yaw: float):
        """Initializes nominal state and resets error state and covariance."""
        self.x_nom = np.array([init_x, init_y, init_disp, init_yaw, 0.0, 0.0], dtype=np.float64)
        self.delta_x = np.zeros(6, dtype=np.float64)
        self.P = np.diag([
            1.0**2, 1.0**2, 0.1**2, np.radians(2.0)**2, 0.01**2, 0.001**2
        ]).astype(np.float64)

    def propagate(self, disp_step: float, w_yaw: float) -> EKFState:
        """
        Propagates nominal state and error covariance P forward by 1 step.
        """
        pos_x, pos_y, s_prev, psi, b_a, b_g = self.x_nom

        # Bias-corrected motion inputs
        s_corr = max(0.0, disp_step - b_a)
        w_corr = w_yaw - b_g

        new_psi = self.wrap_angle(psi + w_corr)
        new_x = pos_x + s_corr * np.cos(new_psi)
        new_y = pos_y + s_corr * np.sin(new_psi)
        new_s = s_corr

        self.x_nom[0] = new_x
        self.x_nom[1] = new_y
        self.x_nom[2] = new_s
        self.x_nom[3] = new_psi

        # Error state Jacobian F = d(f)/d(x)
        F = np.eye(6, dtype=np.float64)
        F[0, 2] = np.cos(new_psi)
        F[0, 3] = -s_corr * np.sin(new_psi)
        F[1, 2] = np.sin(new_psi)
        F[1, 3] = s_corr * np.cos(new_psi)
        F[0, 4] = -np.cos(new_psi)
        F[1, 4] = -np.sin(new_psi)
        F[0, 5] = s_corr * np.sin(new_psi)
        F[1, 5] = -s_corr * np.cos(new_psi)
        F[2, 4] = -1.0
        F[3, 5] = -1.0

        # Propagate covariance: P = F * P * F^T + Q
        self.P = np.dot(np.dot(F, self.P), F.T) + self.Q

        return self.get_state()

    def update_projection(self, hat_delta_x: np.ndarray, confidence: float) -> Tuple[EKFState, np.ndarray]:
        """
        Fix #3: ES-EKF measurement update using neural error-state projection.
          z_proj = hat_delta_x
          r = z_proj - H * delta_x
          R = R_0 / max(confidence, 0.05)
          K = P * H^T * (H * P * H^T + R)^-1
          delta_x = K * r
          Inject: x_nom <- x_nom + delta_x
          P <- (I - K*H)*P*(I - K*H)^T + K*R*K^T
          delta_x <- 0
        """
        c = float(np.clip(confidence, 0.05, 1.0))
        R = self.R_0 / c

        # Observation matrix H (6x6 identity)
        H = np.eye(6, dtype=np.float64)

        # Innovation r = z_proj - H * delta_x
        z_proj = hat_delta_x.astype(np.float64)
        r = z_proj - np.dot(H, self.delta_x)

        # Innovation covariance S = H * P * H^T + R
        S = np.dot(np.dot(H, self.P), H.T) + R

        # Kalman gain K = P * H^T * S^-1
        K = np.dot(np.dot(self.P, H.T), np.linalg.inv(S))

        # Error state update
        delta_x_up = np.dot(K, r)

        # Soft injection into nominal state
        self.x_nom[0] += delta_x_up[0]
        self.x_nom[1] += delta_x_up[1]
        self.x_nom[2] = max(0.0, self.x_nom[2] + delta_x_up[2])
        self.x_nom[3] = self.wrap_angle(self.x_nom[3] + delta_x_up[3])
        self.x_nom[4] = float(np.clip(self.x_nom[4] + delta_x_up[4], -0.2, 0.2))
        self.x_nom[5] = float(np.clip(self.x_nom[5] + delta_x_up[5], -0.05, 0.05))

        # Joseph form covariance update
        I_KH = np.eye(6, dtype=np.float64) - np.dot(K, H)
        self.P = np.dot(np.dot(I_KH, self.P), I_KH.T) + np.dot(np.dot(K, R), K.T)

        # Reset error state to zero
        self.delta_x = np.zeros(6, dtype=np.float64)

        return self.get_state(), delta_x_up

    def update_zupt(self) -> EKFState:
        """
        Zero-velocity pseudo-measurement update.
        Direct observation: step displacement = 0.
        """
        H_zupt = np.zeros((1, 6), dtype=np.float64)
        H_zupt[0, 2] = 1.0

        r_zupt = np.array([0.0 - self.x_nom[2]])
        R_zupt = np.array([[0.01**2]])

        S = np.dot(np.dot(H_zupt, self.P), H_zupt.T) + R_zupt
        K = np.dot(np.dot(self.P, H_zupt.T), np.linalg.inv(S))

        delta_x = np.dot(K, r_zupt)
        self.x_nom[0] += delta_x[0]
        self.x_nom[1] += delta_x[1]
        self.x_nom[2] = 0.0
        self.x_nom[3] = self.wrap_angle(self.x_nom[3] + delta_x[3])
        self.x_nom[4] += delta_x[4]
        self.x_nom[5] += delta_x[5]

        I_KH = np.eye(6, dtype=np.float64) - np.dot(K, H_zupt)
        self.P = np.dot(np.dot(I_KH, self.P), I_KH.T) + np.dot(np.dot(K, R_zupt), K.T)
        self.delta_x = np.zeros(6, dtype=np.float64)

        return self.get_state()

    def get_state(self) -> EKFState:
        return EKFState(
            pos_x=float(self.x_nom[0]),
            pos_y=float(self.x_nom[1]),
            speed_mps=float(self.x_nom[2]),
            yaw_rad=float(self.x_nom[3]),
            bias_acc=float(self.x_nom[4]),
            bias_gyro=float(self.x_nom[5]),
            covariance=self.P.copy(),
        )
