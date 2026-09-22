"""
v9_adaptive_projection_dr/src/event_detector.py
------------------------------------------------
Stage 9 & 10: Deterministic Feature Extraction & Driving Event Classification.

Extracts kinematic features (jerk, centripetal balance, high-frequency rattle energy,
accumulated turn angle) and classifies driving events:
  - STOP
  - HIGH_RATTLE
  - ROUNDABOUT_CANDIDATE
  - TURN
  - ACCEL
  - BRAKE
  - STRAIGHT
  - CRUISE
"""

from __future__ import annotations

import numpy as np
from enum import Enum
from typing import Dict, Any, List, Optional
from collections import deque


class DrivingEvent(str, Enum):
    STOP = "STOP"
    HIGH_RATTLE = "HIGH_RATTLE"
    ROUNDABOUT_CANDIDATE = "ROUNDABOUT_CANDIDATE"
    TURN = "TURN"
    ACCEL = "ACCEL"
    BRAKE = "BRAKE"
    STRAIGHT = "STRAIGHT"
    CRUISE = "CRUISE"


class KinematicFeatureExtractor:
    """
    Extracts deterministic physical features from calibrated IMU and current speed.
    """
    def __init__(self, sample_rate_hz: float = 10.0):
        self.sample_rate_hz = sample_rate_hz
        self.dt = 1.0 / sample_rate_hz
        self.prev_a_fwd = 0.0

        # Buffer for high frequency vibration / rattle detection (last 10 samples)
        self.raw_acc_buffer = deque(maxlen=10)
        # Buffer for accumulated yaw change (last 50 samples = 5.0 s)
        self.yaw_rate_buffer = deque(maxlen=int(5.0 * sample_rate_hz))

    def reset(self):
        self.prev_a_fwd = 0.0
        self.raw_acc_buffer.clear()
        self.yaw_rate_buffer.clear()

    def extract(self, a_fwd: float, a_lat: float, w_yaw: float,
                speed_mps: float, raw_acc: Optional[np.ndarray] = None) -> Dict[str, float]:
        """
        Computes physical metrics:
        - jerk = (a_fwd - prev_a_fwd) / dt
        - centripetal_residual = |a_lat - speed * w_yaw|
        - high_freq_rattle = variance of norm(acc) over buffer
        - accumulated_turn_angle = sum(|w_yaw| * dt) over 5.0 s
        """
        # Longitudinal Jerk
        jerk = (a_fwd - self.prev_a_fwd) / self.dt
        self.prev_a_fwd = a_fwd

        # Lateral centripetal balance (a_lat ~ v * w_yaw for road vehicles)
        centripetal_residual = abs(a_lat - speed_mps * w_yaw)

        # Vibration / Rattle energy
        if raw_acc is not None:
            self.raw_acc_buffer.append(float(np.linalg.norm(raw_acc)))
        else:
            self.raw_acc_buffer.append(float(np.hypot(a_fwd, a_lat)))

        rattle_energy = float(np.std(self.raw_acc_buffer)) if len(self.raw_acc_buffer) >= 3 else 0.0

        # Accumulated turning
        self.yaw_rate_buffer.append(abs(w_yaw) * self.dt)
        accum_turn_rad = float(np.sum(self.yaw_rate_buffer))

        return {
            "a_fwd": a_fwd,
            "a_lat": a_lat,
            "w_yaw": w_yaw,
            "speed_mps": speed_mps,
            "jerk": jerk,
            "centripetal_residual": centripetal_residual,
            "rattle_energy": rattle_energy,
            "accum_turn_rad": accum_turn_rad,
        }


class EventDetector:
    """
    Deterministic rule-based driving event classifier with priority ordering.
    """
    def __init__(self, sample_rate_hz: float = 10.0):
        self.sample_rate_hz = sample_rate_hz
        self.dt = 1.0 / sample_rate_hz
        self.extractor = KinematicFeatureExtractor(sample_rate_hz)

        # Stop persistence counter (must sustain stop for >= 3 steps = 0.3 s)
        self.stop_counter = 0

    def reset(self):
        self.extractor.reset()
        self.stop_counter = 0

    def classify(self, a_fwd: float, a_lat: float, w_yaw: float,
                 speed_mps: float, raw_acc: Optional[np.ndarray] = None) -> Tuple[DrivingEvent, Dict[str, float]]:
        """
        Extracts features and returns current driving event class.
        Priority:
        1. STOP
        2. HIGH_RATTLE
        3. ROUNDABOUT_CANDIDATE
        4. TURN
        5. ACCEL
        6. BRAKE
        7. STRAIGHT
        8. CRUISE
        """
        feats = self.extractor.extract(a_fwd, a_lat, w_yaw, speed_mps, raw_acc)

        # 1. STOP check
        if speed_mps < 0.5 and abs(w_yaw) < 0.05 and abs(a_fwd) < 0.25:
            self.stop_counter += 1
            if self.stop_counter >= 3:
                return DrivingEvent.STOP, feats
        else:
            self.stop_counter = 0

        # 2. HIGH_RATTLE
        if feats["rattle_energy"] > 2.5:
            return DrivingEvent.HIGH_RATTLE, feats

        # 3. ROUNDABOUT_CANDIDATE: sustained turning (w_yaw in [0.08, 0.40]), speed 3-15 m/s, accum_turn >= 1.2 rad (~70 deg)
        if (3.0 <= speed_mps <= 16.0 and
            0.08 <= abs(w_yaw) <= 0.45 and
            feats["accum_turn_rad"] >= 1.15):
            return DrivingEvent.ROUNDABOUT_CANDIDATE, feats

        # 4. TURN
        if abs(w_yaw) > 0.12:
            return DrivingEvent.TURN, feats

        # 5. ACCEL
        if a_fwd > 0.8:
            return DrivingEvent.ACCEL, feats

        # 6. BRAKE
        if a_fwd < -0.8:
            return DrivingEvent.BRAKE, feats

        # 7. STRAIGHT: cruising speed > 5 m/s and minimal turning
        if speed_mps > 5.0 and abs(w_yaw) < 0.05:
            return DrivingEvent.STRAIGHT, feats

        return DrivingEvent.CRUISE, feats


class TemporalContextBuffer:
    """
    Maintains rolling 5.0 s (50-sample) feature history for checkpoint consistency
    and projection inputs.
    """
    def __init__(self, window_size: int = 50):
        self.window_size = window_size
        self.buffer = deque(maxlen=window_size)
        self.event_buffer = deque(maxlen=window_size)

    def reset(self):
        self.buffer.clear()
        self.event_buffer.clear()

    def append(self, feature_dict: Dict[str, float], event: DrivingEvent):
        # Convert feature dict into fixed 8-dim vector
        vec = np.array([
            feature_dict.get("a_fwd", 0.0),
            feature_dict.get("a_lat", 0.0),
            feature_dict.get("w_yaw", 0.0),
            feature_dict.get("speed_mps", 0.0),
            feature_dict.get("jerk", 0.0),
            feature_dict.get("centripetal_residual", 0.0),
            feature_dict.get("rattle_energy", 0.0),
            feature_dict.get("accum_turn_rad", 0.0),
        ], dtype=np.float32)

        self.buffer.append(vec)
        self.event_buffer.append(event)

    def get_array(self) -> np.ndarray:
        """Returns (T, 8) feature array, zero-padded if length < window_size."""
        if len(self.buffer) == 0:
            return np.zeros((self.window_size, 8), dtype=np.float32)
        arr = np.array(self.buffer, dtype=np.float32)
        if len(arr) < self.window_size:
            pad = np.zeros((self.window_size - len(arr), 8), dtype=np.float32)
            arr = np.vstack([pad, arr])
        return arr

    def get_dominant_event(self) -> DrivingEvent:
        """Returns the most frequent event in the active window."""
        if not self.event_buffer:
            return DrivingEvent.CRUISE
        counts: Dict[DrivingEvent, int] = {}
        for ev in self.event_buffer:
            counts[ev] = counts.get(ev, 0) + 1
        return max(counts, key=counts.get)
