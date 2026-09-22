"""
v9_adaptive_projection_dr/src/state_reference.py
-------------------------------------------------
Stage 11: Physical Consistency Reference Synthesizer (Fix #1 & #2).

Separates physical reference estimation from neural correction prediction.
Computes an independent kinematic reference trajectory over a window using:
1. IMU forward acceleration integration with Non-Holonomic Constraints (NHC).
2. Centripetal speed balance: v_cent = sqrt(|a_lat / w_yaw|) during sustained turns.
3. Zero-velocity enforcement during stops.
4. Computes true physical discrepancy:
   d_p = ||x_ref - x_DR|| = sqrt((E_ref - E_DR)^2 + (N_ref - N_DR)^2)
"""

from __future__ import annotations

import numpy as np
from typing import Dict, Any, Tuple, Optional
from dataclasses import dataclass


@dataclass
class PhysicalReferenceState:
    pos_e: float
    pos_n: float
    speed_mps: float
    yaw_rad: float
    discrepancy_m: float  # d_p = ||x_ref - x_DR||
    discrepancy_speed_mps: float
    discrepancy_yaw_rad: float


class StateReferenceSynthesizer:
    """
    Synthesizes independent physical consistency reference from calibrated IMU,
    NHC, and centripetal kinematics.
    """

    def __init__(self, dt: float = 0.1):
        self.dt = dt

    @staticmethod
    def wrap_angle(rad: float) -> float:
        """Wraps angle into [-pi, pi]."""
        return float((rad + np.pi) % (2.0 * np.pi) - np.pi)

    def synthesize_reference(
        self,
        start_state: Dict[str, float],  # {pos_e, pos_n, speed_mps, yaw_rad} at window start
        imu_history: np.ndarray,         # (T, 4): [a_fwd, w_yaw, a_lat, raw_speed_prior]
        dr_current_state: Dict[str, float], # current DR state at window end
        is_stopped: bool = False,
    ) -> PhysicalReferenceState:
        """
        Integrates kinematics forward from start_state using calibrated IMU dynamics
        and centripetal balance over the window of length T.
        """
        T = len(imu_history)
        if T == 0:
            return PhysicalReferenceState(
                pos_e=dr_current_state["pos_e"],
                pos_n=dr_current_state["pos_n"],
                speed_mps=dr_current_state["speed_mps"],
                yaw_rad=dr_current_state["yaw_rad"],
                discrepancy_m=0.0,
                discrepancy_speed_mps=0.0,
                discrepancy_yaw_rad=0.0,
            )

        e_ref = float(start_state.get("pos_e", start_state.get("pos_x", 0.0)))
        n_ref = float(start_state.get("pos_n", start_state.get("pos_y", 0.0)))
        s_ref = float(start_state.get("speed_mps", 10.0))  # step displacement in meters
        yaw_ref = float(start_state.get("yaw_rad", 0.0))

        dr_e = float(dr_current_state.get("pos_e", dr_current_state.get("pos_x", 0.0)))
        dr_n = float(dr_current_state.get("pos_n", dr_current_state.get("pos_y", 0.0)))
        dr_s = float(dr_current_state.get("speed_mps", 0.0))
        dr_yaw = float(dr_current_state.get("yaw_rad", 0.0))

        for t in range(T):
            a_fwd = float(imu_history[t, 0])
            w_yaw = float(imu_history[t, 1])
            a_lat = float(imu_history[t, 2])

            if is_stopped:
                s_ref = 0.0
            else:
                # Longitudinal acceleration integration matching dataset format
                s_acc = max(0.0, s_ref + a_fwd)

                # Centripetal consistency check during sustained turning (a_lat ~ v * w_yaw)
                if abs(w_yaw) > 0.12 and abs(a_lat) > 1.0:
                    s_cent = abs(a_lat) / max(0.10, abs(w_yaw))
                    s_ref = 0.85 * s_acc + 0.15 * np.clip(s_cent, 0.0, 45.0)
                else:
                    s_ref = s_acc

                s_ref = float(np.clip(s_ref, 0.0, 45.0))

            # Update heading with gentle gyro integration
            yaw_ref = self.wrap_angle(yaw_ref + w_yaw)

            # Local 2D propagation matching v3 (cos for axis 0, sin for axis 1)
            e_ref += s_ref * np.cos(yaw_ref)
            n_ref += s_ref * np.sin(yaw_ref)

        # Discrepancy metric d_p = ||x_ref - x_DR||
        delta_e = e_ref - dr_e
        delta_n = n_ref - dr_n
        d_p = float(np.hypot(delta_e, delta_n))

        d_s = float(abs(s_ref - dr_s))
        d_yaw = float(abs(self.wrap_angle(yaw_ref - dr_yaw)))

        return PhysicalReferenceState(
            pos_e=e_ref,
            pos_n=n_ref,
            speed_mps=s_ref,
            yaw_rad=yaw_ref,
            discrepancy_m=d_p,
            discrepancy_speed_mps=d_s,
            discrepancy_yaw_rad=d_yaw,
        )
