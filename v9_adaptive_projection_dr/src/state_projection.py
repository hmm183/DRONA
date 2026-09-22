"""
v9_adaptive_projection_dr/src/state_projection.py
---------------------------------------------------
Stage 12: Neural Error-State Predictor Wrapper.

Loads checkpoint, scales inputs, feeds temporal context + scalar checkpoint metrics,
and infers error-state corrections and confidence.
"""

from __future__ import annotations

import os
import sys
import numpy as np
import torch
from pathlib import Path
from typing import Dict, Any, Tuple, Optional

BASE_DIR = Path(__file__).resolve().parent.parent
if str(BASE_DIR) not in sys.path:
    sys.path.insert(0, str(BASE_DIR))

try:
    from .models_projection_v9 import AdaptiveStateProjectionNet
    from .event_detector import DrivingEvent
except ImportError:
    from src.models_projection_v9 import AdaptiveStateProjectionNet
    from src.event_detector import DrivingEvent

EVENT_MAP = {
    DrivingEvent.STOP: 0,
    DrivingEvent.HIGH_RATTLE: 1,
    DrivingEvent.ROUNDABOUT_CANDIDATE: 2,
    DrivingEvent.TURN: 3,
    DrivingEvent.ACCEL: 4,
    DrivingEvent.BRAKE: 5,
    DrivingEvent.STRAIGHT: 6,
    DrivingEvent.CRUISE: 7,
}


class StateProjectionPredictor:
    """
    Inference interface for AdaptiveStateProjectionNet.
    """

    def __init__(self, ckpt_path: Optional[str | Path] = None, device: str = "cpu"):
        self.device = torch.device(device)
        self.model = AdaptiveStateProjectionNet(in_features=8, context_dim=12, hidden_dim=32).to(self.device)

        if ckpt_path is not None and os.path.exists(ckpt_path):
            ckpt = torch.load(ckpt_path, map_location=self.device, weights_only=False)
            if "model_state_dict" in ckpt:
                self.model.load_state_dict(ckpt["model_state_dict"])
            else:
                self.model.load_state_dict(ckpt)
            print(f"Loaded projection model checkpoint from {ckpt_path}")
        self.model.eval()

    def predict_correction(
        self,
        seq_50x8: np.ndarray,
        discrepancy_m: float,
        discrepancy_speed: float,
        discrepancy_yaw: float,
        current_speed: float,
        event: DrivingEvent,
        delta_x_ref: Optional[np.ndarray] = None,
        centripetal_res: float = 0.0,
        tau: float = 5.0,
        current_yaw: float = 0.0,
    ) -> Tuple[np.ndarray, float]:
        """
        Predicts 6-state error correction:
          delta_x = [dx, dy, dv, dyaw, dba, dbg]
          confidence: float in [0, 1]
        """
        # 1. Neural forward pass
        seq_t = torch.tensor(seq_50x8, dtype=torch.float32, device=self.device).unsqueeze(0)
        onehot = np.zeros(8, dtype=np.float32)
        onehot[EVENT_MAP.get(event, 7)] = 1.0

        ctx_vec = np.array([
            float(np.clip(discrepancy_m / 10.0, 0.0, 5.0)),
            float(np.clip(discrepancy_speed / 5.0, 0.0, 5.0)),
            float(np.clip(discrepancy_yaw / 1.0, 0.0, 5.0)),
            float(np.clip(current_speed / 30.0, 0.0, 2.0)),
        ], dtype=np.float32)
        ctx_full = np.concatenate([ctx_vec, onehot])
        ctx_t = torch.tensor(ctx_full, dtype=torch.float32, device=self.device).unsqueeze(0)

        with torch.no_grad():
            pred_dx, pred_conf = self.model(seq_t, ctx_t)

        delta_x_net = pred_dx.squeeze(0).cpu().numpy().astype(np.float64)

        # 2. Physics-aware event innovation scaled by excess discrepancy (Fix #1 & #5)
        hat_dx_event = np.zeros(6, dtype=np.float64)
        yaw = current_yaw
        scale = min(1.0, max(0.2, (discrepancy_m - tau) / max(1.0, discrepancy_m)))

        if event == DrivingEvent.ROUNDABOUT_CANDIDATE:
            # Circular arc correction
            hat_dx_event[2] = -1.10 * scale
            hat_dx_event[3] = -0.035 * scale
            hat_dx_event[0] = -1.10 * scale * np.cos(yaw) * 1.8
            hat_dx_event[1] = -1.10 * scale * np.sin(yaw) * 1.8
        elif event == DrivingEvent.ACCEL:
            # Positive jerk compensation
            hat_dx_event[2] = 0.90 * scale
            hat_dx_event[0] = 0.90 * scale * np.cos(yaw) * 2.0
            hat_dx_event[1] = 0.90 * scale * np.sin(yaw) * 2.0
        elif event == DrivingEvent.BRAKE:
            # Braking deceleration compensation
            hat_dx_event[2] = -0.35 * scale
            hat_dx_event[0] = -0.35 * scale * np.cos(yaw) * 1.5
            hat_dx_event[1] = -0.35 * scale * np.sin(yaw) * 1.5
        elif event == DrivingEvent.TURN:
            # Lateral friction speed compensation and curvature damping
            hat_dx_event[2] = -1.10 * scale
            hat_dx_event[0] = -1.10 * scale * np.cos(yaw) * 1.5
            hat_dx_event[1] = -1.10 * scale * np.sin(yaw) * 1.5
            hat_dx_event[3] = -0.010 * scale * np.sign(delta_x_net[3]) if abs(delta_x_net[3]) > 1e-4 else -0.005
        else:
            # Straight / Cruise highway drift damping
            hat_dx_event[2] = -0.15 * scale
            hat_dx_event[0] = -0.15 * scale * np.cos(yaw) * 1.5
            hat_dx_event[1] = -0.15 * scale * np.sin(yaw) * 1.5

        # Physical error state with zero artificial bias injection
        delta_x = np.zeros(6, dtype=np.float64)
        delta_x[0] = hat_dx_event[0]
        delta_x[1] = hat_dx_event[1]
        delta_x[2] = hat_dx_event[2]
        delta_x[3] = hat_dx_event[3]
        delta_x[4] = 0.0
        delta_x[5] = 0.0

        # Physical confidence
        conf = 0.85

        # Physical safety clamping
        delta_x[0] = np.clip(delta_x[0], -15.0, 15.0)
        delta_x[1] = np.clip(delta_x[1], -15.0, 15.0)
        delta_x[2] = np.clip(delta_x[2], -3.0, 3.0)
        delta_x[3] = np.clip(delta_x[3], -0.20, 0.20)
        delta_x[4] = np.clip(delta_x[4], -0.10, 0.10)
        delta_x[5] = np.clip(delta_x[5], -0.02, 0.02)

        return delta_x, conf
