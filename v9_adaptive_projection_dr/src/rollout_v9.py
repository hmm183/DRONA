"""
v9_adaptive_projection_dr/src/rollout_v9.py
--------------------------------------------
Stage 16: Closed-Loop Simulation & Trajectory Rollout Engine.

Supports both:
  1. simulate_journey_sequence(journey, start_idx, outage_len, mode, ...):
     Identical signature and kinematics to v3_adapter.simulate_sequence,
     enabling exact 1:1 baseline comparison and audit.
  2. run_full_journey(journey, horizon_steps, mode, ...):
     Full journey simulation.
"""

from __future__ import annotations

import sys
import numpy as np
from pathlib import Path
from typing import Dict, Any, List, Optional, Tuple

BASE_DIR = Path(__file__).resolve().parent.parent
if str(BASE_DIR) not in sys.path:
    sys.path.insert(0, str(BASE_DIR))

try:
    from .v3_adapter import V3Adapter, WINDOW
    from .event_detector import EventDetector, DrivingEvent, TemporalContextBuffer
    from .fusion_engine_v9 import ESEKFFusionEngine
    from .projection_controller import ProjectionController
    from .state_projection import StateProjectionPredictor
except ImportError:
    from src.v3_adapter import V3Adapter, WINDOW
    from src.event_detector import EventDetector, DrivingEvent, TemporalContextBuffer
    from src.fusion_engine_v9 import ESEKFFusionEngine
    from src.projection_controller import ProjectionController
    from src.state_projection import StateProjectionPredictor


class ClosedLoopRolloutEngine:
    """
    Executes closed-loop simulation over driving trajectories.
    """

    def __init__(
        self,
        v3_adapter: V3Adapter,
        projection_predictor: Optional[StateProjectionPredictor] = None,
        dt: float = 0.1,
    ):
        self.v3_adapter = v3_adapter
        self.dt = dt
        self.event_detector = EventDetector(sample_rate_hz=1.0 / dt)
        self.context_buffer = TemporalContextBuffer(window_size=50)
        self.ekf = ESEKFFusionEngine(dt=dt)
        self.controller = ProjectionController(
            predictor=projection_predictor,
            checkpoint_interval_steps=50,
            min_confidence=0.20,
            log_csv=False,
        )

    def simulate_journey_sequence(
        self,
        journey: Dict[str, Any],
        start_idx: int,
        outage_len: int = 10,
        mode: str = "v9_full",  # "v3_only", "ekf_only", "v9_full", "forced_overwrite"
        override_threshold: Optional[float] = None,
        scenario_name: Optional[str] = None,
    ) -> Dict[str, Any]:
        """
        Executes dead reckoning over [start_idx, start_idx + outage_len].
        Matches v3 coordinate frame and kinematics exactly.
        """
        a_fwd = journey["a_fwd"]
        w_yaw = journey["w_yaw"]
        a_lat = journey["a_lat"]
        x_gps = journey["x_gps"]
        w_gps = journey["w_gps"]
        headings = journey["headings"]

        # History initialization
        history_v = list(x_gps[start_idx - WINDOW: start_idx])
        pos_gt = [(0.0, 0.0)]
        pos_est = [(0.0, 0.0)]

        psi_gt = float(np.radians(headings[start_idx]))
        psi_est = psi_gt

        # Reset states
        self.v3_adapter.zupt_gate.reset()
        self.event_detector.reset()
        self.context_buffer.reset()
        init_s = float(x_gps[start_idx - 1])
        self.ekf.initialize(0.0, 0.0, init_s, psi_est)
        self.controller.reset(0.0, 0.0, init_s, psi_est)

        # Dynamic checkpoint interval: for outages < 50 steps, checkpoint at midpoint
        chk_interval = min(50, max(5, outage_len // 2))
        self.controller.checkpoint_interval = chk_interval

        if override_threshold is not None:
            for ev in self.controller.thresholds:
                self.controller.thresholds[ev] = override_threshold

        v_preds, w_preds = [], []
        errors = [0.0]
        projections_fired = 0

        for k in range(outage_len):
            cur = start_idx + k

            # Prepare V3 input window (10, 4)
            ch_a_fwd = np.clip(a_fwd[cur - WINDOW + 1: cur + 1], -8.0, 8.0)
            ch_w_yaw = np.clip(w_yaw[cur - WINDOW + 1: cur + 1], -1.0, 1.0)
            ch_a_lat = np.clip(a_lat[cur - WINDOW + 1: cur + 1], -8.0, 8.0)
            ch_v_prev = np.clip(np.array(history_v[-WINDOW:]), 0.0, 45.0)

            window = np.stack([ch_a_fwd, ch_w_yaw, ch_a_lat, ch_v_prev], axis=-1)
            pred = self.v3_adapter.predict_step(window)
            v_p = pred["v_pred"]
            w_p = pred["w_pred"]
            is_stopped = pred["is_stopped"]

            history_v.append(v_p)
            v_preds.append(v_p)
            w_preds.append(w_p)

            # Ground truth motion propagation
            true_x = float(x_gps[cur])
            true_w = float(w_gps[cur])
            psi_gt = float((psi_gt + true_w + np.pi) % (2.0 * np.pi) - np.pi)
            new_gt_x = pos_gt[-1][0] + true_x * np.cos(psi_gt)
            new_gt_y = pos_gt[-1][1] + true_x * np.sin(psi_gt)
            pos_gt.append((new_gt_x, new_gt_y))

            # IMU current sample
            raw_acc = np.array([a_fwd[cur], a_lat[cur], 0.0])
            event, feats = self.event_detector.classify(
                a_fwd=a_fwd[cur], a_lat=a_lat[cur], w_yaw=w_yaw[cur],
                speed_mps=v_p * 10.0, raw_acc=raw_acc
            )
            self.context_buffer.append(feats, event)

            # Dead reckoning propagation by mode
            if mode == "v3_only":
                # Config A: Pure Neural V3 (exact reproduction)
                psi_est = float((psi_est + w_p + np.pi) % (2.0 * np.pi) - np.pi)
                new_est_x = pos_est[-1][0] + v_p * np.cos(psi_est)
                new_est_y = pos_est[-1][1] + v_p * np.sin(psi_est)
                pos_est.append((new_est_x, new_est_y))

            elif mode == "ekf_only":
                # Config B: V3 + EKF
                self.ekf.propagate(disp_step=v_p, w_yaw=w_p)
                if is_stopped:
                    self.ekf.update_zupt()
                st = self.ekf.get_state()
                pos_est.append((st.pos_x, st.pos_y))

            else:
                # Config C: v9 Full Adaptive Projection DR
                self.ekf.propagate(disp_step=v_p, w_yaw=w_p)
                if is_stopped:
                    self.ekf.update_zupt()

                st = self.ekf.get_state()
                dr_state = {
                    "pos_e": st.pos_x,
                    "pos_n": st.pos_y,
                    "speed_mps": st.speed_mps,
                    "yaw_rad": st.yaw_rad,
                }
                calib_row = np.array([a_fwd[cur], w_yaw[cur], a_lat[cur], v_p])

                chk_res = self.controller.step(
                    step_idx=k + 1,
                    calibrated_imu=calib_row,
                    dr_state=dr_state,
                    context_buffer=self.context_buffer,
                    active_event=event,
                    ekf_engine=self.ekf,
                    is_stopped=is_stopped,
                    scenario_name=scenario_name,
                )

                if chk_res["projection_fired"]:
                    projections_fired += 1
                    if mode == "forced_overwrite":
                        dx = chk_res["correction_applied"]
                        self.ekf.x_nom[0] += dx[0]
                        self.ekf.x_nom[1] += dx[1]

                st = self.ekf.get_state()
                pos_est.append((st.pos_x, st.pos_y))

            err = float(np.hypot(pos_est[-1][0] - pos_gt[-1][0], pos_est[-1][1] - pos_gt[-1][1]))
            errors.append(err)

        fde = float(np.hypot(pos_est[-1][0] - pos_gt[-1][0], pos_est[-1][1] - pos_gt[-1][1]))
        rmse = float(np.sqrt(np.mean(np.square(errors))))

        return {
            "drift_m": fde,
            "fde_m": fde,
            "rmse_m": rmse,
            "errors": errors,
            "pos_gt": np.array(pos_gt),
            "pos_est": np.array(pos_est),
            "v_preds": v_preds,
            "w_preds": w_preds,
            "projections_fired": projections_fired,
        }
