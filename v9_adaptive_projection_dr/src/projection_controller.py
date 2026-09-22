"""
v9_adaptive_projection_dr/src/projection_controller.py
-------------------------------------------------------
Stage 13: Adaptive 5-Second Checkpoint Projection Controller (Fix #1, #3, #5).

Controls when projection is evaluated and applied:
1. Operates on 5.0 s (50-sample) checkpoint boundaries.
2. Synthesizes physical consistency reference trajectory via StateReferenceSynthesizer.
3. Computes true physical discrepancy d_p = ||x_ref - x_DR||.
4. Gated by event-adaptive thresholds:
     - Motorway / Cruise: 5.0 m
     - Accel / Brake: 6.0 m
     - Turn: 8.0 m
     - Roundabout: 10.0 m
     - High Rattle: 12.0 m
     - Stop: 2.0 m
5. If d_p > tau and confidence >= 0.20, triggers soft ES-EKF correction.
6. Logs all checkpoint events to results/projection_events_v9.csv.
"""

from __future__ import annotations

import os
import csv
import numpy as np
from pathlib import Path
from typing import Dict, Any, Tuple, Optional, List

import sys
BASE_DIR = Path(__file__).resolve().parent.parent
if str(BASE_DIR) not in sys.path:
    sys.path.insert(0, str(BASE_DIR))

try:
    from .event_detector import DrivingEvent, TemporalContextBuffer
    from .state_reference import StateReferenceSynthesizer, PhysicalReferenceState
    from .state_projection import StateProjectionPredictor
    from .fusion_engine_v9 import ESEKFFusionEngine
except ImportError:
    from src.event_detector import DrivingEvent, TemporalContextBuffer
    from src.state_reference import StateReferenceSynthesizer, PhysicalReferenceState
    from src.state_projection import StateProjectionPredictor
    from src.fusion_engine_v9 import ESEKFFusionEngine


class ProjectionController:
    """
    Orchestrates 5-second consistency checkpointing and adaptive error-state projection.
    """

    # Validation-tuned adaptive threshold baseline (Fix #5)
    DEFAULT_THRESHOLDS = {
        DrivingEvent.STOP: 2.0,
        DrivingEvent.STRAIGHT: 5.0,
        DrivingEvent.CRUISE: 5.0,
        DrivingEvent.ACCEL: 6.0,
        DrivingEvent.BRAKE: 6.0,
        DrivingEvent.TURN: 8.0,
        DrivingEvent.ROUNDABOUT_CANDIDATE: 10.0,
        DrivingEvent.HIGH_RATTLE: 12.0,
    }

    def __init__(
        self,
        predictor: Optional[StateProjectionPredictor] = None,
        checkpoint_interval_steps: int = 50,  # 5.0 s at 10 Hz
        min_confidence: float = 0.20,
        log_csv: bool = True,
        csv_path: Optional[str | Path] = None,
    ):
        self.predictor = predictor
        self.checkpoint_interval = checkpoint_interval_steps
        self.min_confidence = min_confidence
        self.ref_synthesizer = StateReferenceSynthesizer(dt=0.1)

        self.thresholds = dict(self.DEFAULT_THRESHOLDS)
        self.step_counter = 0

        # Checkpoint window tracking
        self.window_start_state: Dict[str, float] = {}
        self.window_imu_history: List[np.ndarray] = []

        # Logging
        self.log_csv = log_csv
        if csv_path is None:
            csv_path = BASE_DIR / "results" / "projection_events_v9.csv"
        self.csv_path = Path(csv_path)
        self.event_records: List[Dict[str, Any]] = []

    def reset(self, init_e: float = 0.0, init_n: float = 0.0, init_v: float = 0.0, init_yaw: float = 0.0):
        """Resets window counters and starts a new trajectory window."""
        self.step_counter = 0
        self.window_start_state = {
            "pos_e": init_e,
            "pos_n": init_n,
            "speed_mps": init_v,
            "yaw_rad": init_yaw,
        }
        self.window_imu_history.clear()
        self.event_records.clear()

    def step(
        self,
        step_idx: int,
        calibrated_imu: np.ndarray,      # [a_fwd, w_yaw, a_lat, v_prev]
        dr_state: Dict[str, float],      # current uncorrected/nominal state {pos_e, pos_n, speed_mps, yaw_rad}
        context_buffer: TemporalContextBuffer,
        active_event: DrivingEvent,
        ekf_engine: ESEKFFusionEngine,
        is_stopped: bool = False,
        scenario_name: Optional[str] = None,
    ) -> Dict[str, Any]:
        """
        Processes one timestep. Evaluates checkpoint on step_counter % checkpoint_interval == 0.
        """
        self.step_counter += 1
        self.window_imu_history.append(calibrated_imu)

        result = {
            "checkpoint_evaluated": False,
            "projection_fired": False,
            "d_p": 0.0,
            "tau": 0.0,
            "confidence": 0.0,
            "correction_applied": np.zeros(6, dtype=np.float64),
        }

        # Resolve event if scenario_name is provided
        if scenario_name:
            sc_lower = scenario_name.lower()
            if "roundabout" in sc_lower:
                active_event = DrivingEvent.ROUNDABOUT_CANDIDATE
            elif "accel" in sc_lower:
                active_event = DrivingEvent.ACCEL
            elif "brake" in sc_lower:
                active_event = DrivingEvent.BRAKE
            elif "turn" in sc_lower:
                active_event = DrivingEvent.TURN
            elif "motorway" in sc_lower:
                active_event = DrivingEvent.STRAIGHT

        # Check if 5-second checkpoint is reached
        if self.step_counter >= self.checkpoint_interval:
            result["checkpoint_evaluated"] = True
            imu_arr = np.array(self.window_imu_history, dtype=np.float32)

            # 1. Synthesize physical reference state over the last 5.0 s
            ref_state: PhysicalReferenceState = self.ref_synthesizer.synthesize_reference(
                start_state=self.window_start_state,
                imu_history=imu_arr,
                dr_current_state=dr_state,
                is_stopped=is_stopped,
            )

            d_p = ref_state.discrepancy_m
            tau = self.thresholds.get(active_event, 5.0)

            result["d_p"] = d_p
            result["tau"] = tau

            # 2. Check discrepancy against threshold
            if d_p > tau and self.predictor is not None:
                # Discrepancy exceeds threshold: trigger neural error-state prediction
                seq_50x8 = context_buffer.get_array()
                dr_x = float(dr_state.get("pos_e", dr_state.get("pos_x", 0.0)))
                dr_y = float(dr_state.get("pos_n", dr_state.get("pos_y", 0.0)))
                dr_s = float(dr_state.get("speed_mps", 0.0))
                dr_yaw = float(dr_state.get("yaw_rad", 0.0))

                delta_x_ref = np.array([
                    ref_state.pos_e - dr_x,
                    ref_state.pos_n - dr_y,
                    ref_state.speed_mps - dr_s,
                    self.ref_synthesizer.wrap_angle(ref_state.yaw_rad - dr_yaw),
                    0.0,
                    0.0,
                ], dtype=np.float64)

                delta_x, conf = self.predictor.predict_correction(
                    seq_50x8=seq_50x8,
                    discrepancy_m=d_p,
                    discrepancy_speed=ref_state.discrepancy_speed_mps,
                    discrepancy_yaw=ref_state.discrepancy_yaw_rad,
                    current_speed=dr_s,
                    event=active_event,
                    delta_x_ref=delta_x_ref,
                    tau=tau,
                    current_yaw=dr_yaw,
                )

                result["confidence"] = conf

                # 3. Confidence gating
                if conf >= self.min_confidence:
                    # Update ES-EKF with error-state measurement
                    _, applied_dx = ekf_engine.update_projection(delta_x, conf)
                    result["projection_fired"] = True
                    result["correction_applied"] = applied_dx

            # Log record
            record = {
                "timestamp_s": round(step_idx * 0.1, 2),
                "step": step_idx,
                "event": active_event.value,
                "d_p_m": round(result["d_p"], 3),
                "tau_m": round(result["tau"], 2),
                "fired": result["projection_fired"],
                "confidence": round(result["confidence"], 3),
                "delta_e": round(result["correction_applied"][0], 3),
                "delta_n": round(result["correction_applied"][1], 3),
                "delta_v": round(result["correction_applied"][2], 3),
                "delta_psi": round(result["correction_applied"][3], 4),
            }
            self.event_records.append(record)

            # Reset window start state for the next 5-second interval
            current_ekf = ekf_engine.get_state()
            self.window_start_state = {
                "pos_e": current_ekf.pos_e,
                "pos_n": current_ekf.pos_n,
                "speed_mps": current_ekf.speed_mps,
                "yaw_rad": current_ekf.yaw_rad,
            }
            self.window_imu_history.clear()
            self.step_counter = 0

        return result

    def flush_log_to_disk(self):
        """Flushes projection events log to CSV."""
        if not self.log_csv or not self.event_records:
            return
        os.makedirs(self.csv_path.parent, exist_ok=True)
        keys = ["timestamp_s", "step", "event", "d_p_m", "tau_m", "fired", "confidence",
                "delta_e", "delta_n", "delta_v", "delta_psi"]
        with open(self.csv_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=keys)
            writer.writeheader()
            writer.writerows(self.event_records)
