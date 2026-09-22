"""
v9_adaptive_projection_dr/src/v3_adapter.py
-------------------------------------------
Stage 1 & 2: V3 PINO-DR Model Adapter & Reproduction Protocol.

Wraps the verified v3 PINO-DR checkpoint to serve as the immutable
motion-intelligence backbone. Exposes predict_step and predict_sequence APIs.
"""

from __future__ import annotations

import json
import math
import pickle
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

BASE_DIR = Path(__file__).resolve().parent.parent
REPO_ROOT = BASE_DIR.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import numpy as np
import torch
import torch.nn as nn

# Constants matching v3 physical limits
DISP_LO, DISP_HI = 0.0, 45.0
ORI_LO, ORI_HI = -1.2, 1.2
WINDOW = 10

# Fixed reference benchmark targets for v3
V3_TARGETS = {
    "motorway": 7.13,
    "quick_accel": 21.11,
    "hard_brake": 17.15,
    "sharp_turns": 39.26,
    "roundabout": 75.31,
}


class _ConvBlock(nn.Module):
    def __init__(self, in_c: int, out_c: int, kernel: int = 3, dilation: int = 1, padding: int = 1):
        super().__init__()
        self.conv = nn.Conv1d(in_c, out_c, kernel_size=kernel, dilation=dilation, padding=padding)
        self.bn = nn.BatchNorm1d(out_c)
        self.act = nn.GELU()

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.act(self.bn(self.conv(x)))


class _TemporalAttention(nn.Module):
    def __init__(self, feat_dim: int):
        super().__init__()
        self.query = nn.Parameter(torch.randn(feat_dim))
        self.scale = math.sqrt(feat_dim)

    def forward(self, h: torch.Tensor) -> torch.Tensor:
        scores = torch.matmul(h, self.query) / self.scale
        weights = torch.softmax(scores, dim=1).unsqueeze(-1)
        return (h * weights).sum(dim=1)


class PINODeadReckoningNet(nn.Module):
    def __init__(self, in_channels: int = 4, conv_channels: int = 32,
                 gru_hidden: int = 32, num_gru_layers: int = 1, dropout: float = 0.20):
        super().__init__()
        self.conv1 = _ConvBlock(in_channels, conv_channels, kernel=3, dilation=1, padding=1)
        self.conv2 = _ConvBlock(conv_channels, conv_channels, kernel=3, dilation=2, padding=2)
        self.conv_drop = nn.Dropout(dropout)
        self.gru = nn.GRU(
            input_size=conv_channels,
            hidden_size=gru_hidden,
            num_layers=num_gru_layers,
            batch_first=True,
            bidirectional=True,
            dropout=0.0
        )
        gru_out_dim = gru_hidden * 2
        self.attention = _TemporalAttention(gru_out_dim)
        self.disp_head = nn.Sequential(
            nn.Linear(gru_out_dim, 32),
            nn.GELU(),
            nn.Dropout(dropout),
            nn.Linear(32, 1)
        )
        self.ori_head = nn.Sequential(
            nn.Linear(gru_out_dim, 32),
            nn.GELU(),
            nn.Dropout(dropout),
            nn.Linear(32, 1)
        )
        self.zupt_head = nn.Sequential(
            nn.Linear(gru_out_dim, 16),
            nn.GELU(),
            nn.Dropout(dropout),
            nn.Linear(16, 1)
        )

    def forward(self, x: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        h = x.permute(0, 2, 1)
        h1 = self.conv1(h)
        h2 = self.conv2(h1)
        h = self.conv_drop(h1 + h2)
        h = h.permute(0, 2, 1)
        h, _ = self.gru(h)
        context = self.attention(h)
        delta_v = self.disp_head(context)
        v_prev_last = x[:, -1, 3:4]
        disp = torch.clamp(v_prev_last + delta_v, 0.0, 1.0)
        ori = self.ori_head(context)
        zupt = self.zupt_head(context)
        return disp, ori, zupt


class ZUPTHysteresisGate:
    """Hysteresis Schmitt trigger for Zero-Velocity Updates."""
    def __init__(self, threshold_high: float = 0.70, threshold_low: float = 0.30,
                 n_enter: int = 3, n_exit: int = 2):
        self.threshold_high = threshold_high
        self.threshold_low = threshold_low
        self.n_enter = n_enter
        self.n_exit = n_exit
        self.is_stopped = False
        self.high_count = 0
        self.low_count = 0

    def reset(self):
        self.is_stopped = False
        self.high_count = 0
        self.low_count = 0

    def update(self, p_stop: float) -> bool:
        if not self.is_stopped:
            if p_stop > self.threshold_high:
                self.high_count += 1
                self.low_count = 0
                if self.high_count >= self.n_enter:
                    self.is_stopped = True
            else:
                self.high_count = 0
        else:
            if p_stop < self.threshold_low:
                self.low_count += 1
                self.high_count = 0
                if self.low_count >= self.n_exit:
                    self.is_stopped = False
            else:
                self.low_count = 0
        return self.is_stopped


class V3Adapter:
    """
    Production wrapper around the frozen v3 PINO-DR model.
    Handles scaling, inference, clipping, and ZUPT gating.
    """
    def __init__(self, ckpt_path: Optional[Path] = None,
                 scalers_path: Optional[Path] = None,
                 device: str = "cpu"):
        self.device = torch.device(device)
        if ckpt_path is None:
            ckpt_path = BASE_DIR / "checkpoints" / "v3_base_reference.pth"
            if not ckpt_path.exists():
                ckpt_path = REPO_ROOT / "checkpoints_v3" / "best_model.pth"

        if scalers_path is None:
            scalers_path = BASE_DIR / "data" / "scalers_v3.pkl"
            if not scalers_path.exists():
                scalers_path = REPO_ROOT / "data" / "preprocessed" / "v3" / "scalers_v3.pkl"
            if not scalers_path.exists():
                scalers_path = REPO_ROOT / "v3_pino_dr" / "data" / "scalers_v3.pkl"

        with open(scalers_path, "rb") as f:
            scalers = pickle.load(f)

        self.s_X = scalers["X"]
        self.s_yd = scalers["y_disp"]
        self.s_yo = scalers["y_ori"]

        ckpt = torch.load(ckpt_path, map_location=self.device, weights_only=False)
        cfg = ckpt.get("config", {})

        self.model = PINODeadReckoningNet(
            in_channels=cfg.get("in_channels", 4),
            conv_channels=cfg.get("conv_channels", 32),
            gru_hidden=cfg.get("gru_hidden", 32),
            num_gru_layers=cfg.get("num_gru_layers", 1),
            dropout=cfg.get("dropout", 0.20)
        ).to(self.device)
        self.model.load_state_dict(ckpt["model_state_dict"])
        self.model.eval()

        # Freeze all weights
        for p in self.model.parameters():
            p.requires_grad = False

        self.zupt_gate = ZUPTHysteresisGate()

    def predict_step(self, window_10x4: np.ndarray) -> Dict[str, float]:
        """
        Inference on a single (10, 4) window [a_fwd, w_yaw, a_lat, v_prev].
        Returns unscaled physical predictions: velocity (m/s), yaw_rate (rad/s), p_stop, is_stopped.
        """
        w_flat = window_10x4.reshape(1, -1)
        w_scaled = self.s_X.transform(w_flat).reshape(1, 10, 4).astype(np.float32)

        with torch.no_grad():
            inp = torch.tensor(w_scaled, dtype=torch.float32, device=self.device)
            d_s, o_s, z_l = self.model(inp)

        v_raw = self.s_yd.inverse_transform([[d_s.item()]])[0, 0]
        w_raw = self.s_yo.inverse_transform([[o_s.item()]])[0, 0]
        v_pred = float(np.clip(v_raw, DISP_LO, DISP_HI))
        w_pred = float(np.clip(w_raw, ORI_LO, ORI_HI))
        p_stop = float(torch.sigmoid(z_l).item())

        is_stopped = self.zupt_gate.update(p_stop)
        if is_stopped:
            v_pred = 0.0
            w_pred = 0.0

        return {
            "v_pred": v_pred,
            "w_pred": w_pred,
            "p_stop": p_stop,
            "is_stopped": is_stopped,
        }

    def simulate_sequence(self, journey: Dict[str, Any], start_idx: int,
                          outage_len: int = 10) -> Dict[str, Any]:
        """Runs closed-loop v3 simulation on a given journey sequence."""
        a_fwd = journey["a_fwd"]
        w_yaw = journey["w_yaw"]
        a_lat = journey["a_lat"]
        x_gps = journey["x_gps"]
        w_gps = journey["w_gps"]
        headings = journey["headings"]

        history_v = list(x_gps[start_idx - WINDOW: start_idx])
        pos_gt = [(0.0, 0.0)]
        pos_v3 = [(0.0, 0.0)]
        pos_ins = [(0.0, 0.0)]

        psi_gt = np.radians(headings[start_idx])
        psi_v3 = psi_gt
        psi_ins = psi_gt
        v_ins = x_gps[start_idx - 1]

        zupt = ZUPTHysteresisGate()
        v_preds, w_preds = [], []

        for k in range(outage_len):
            cur = start_idx + k
            ch_a_fwd = np.clip(a_fwd[cur - WINDOW + 1: cur + 1], -8.0, 8.0)
            ch_w_yaw = np.clip(w_yaw[cur - WINDOW + 1: cur + 1], -1.0, 1.0)
            ch_a_lat = np.clip(a_lat[cur - WINDOW + 1: cur + 1], -8.0, 8.0)
            ch_v_prev = np.clip(np.array(history_v[-WINDOW:]), 0.0, 45.0)

            window = np.stack([ch_a_fwd, ch_w_yaw, ch_a_lat, ch_v_prev], axis=-1)
            pred = self.predict_step(window)
            v_p = pred["v_pred"]
            w_p = pred["w_pred"]

            history_v.append(v_p)
            v_preds.append(v_p)
            w_preds.append(w_p)

            true_x = x_gps[cur]
            true_w = w_gps[cur]
            raw_w = w_yaw[cur]

            psi_gt += true_w
            psi_v3 += w_p
            psi_ins += raw_w
            v_ins = max(0.0, v_ins + a_fwd[cur])

            pos_gt.append((pos_gt[-1][0] + true_x * np.cos(psi_gt),
                           pos_gt[-1][1] + true_x * np.sin(psi_gt)))
            pos_v3.append((pos_v3[-1][0] + v_p * np.cos(psi_v3),
                           pos_v3[-1][1] + v_p * np.sin(psi_v3)))
            pos_ins.append((pos_ins[-1][0] + v_ins * np.cos(psi_ins),
                           pos_ins[-1][1] + v_ins * np.sin(psi_ins)))

        drift_v3 = float(np.hypot(pos_v3[-1][0] - pos_gt[-1][0], pos_v3[-1][1] - pos_gt[-1][1]))
        drift_ins = float(np.hypot(pos_ins[-1][0] - pos_gt[-1][0], pos_ins[-1][1] - pos_gt[-1][1]))

        return {
            "drift_v3": drift_v3,
            "drift_ins": drift_ins,
            "pos_gt": np.array(pos_gt),
            "pos_v3": np.array(pos_v3),
            "pos_ins": np.array(pos_ins),
            "v_preds": v_preds,
            "w_preds": w_preds,
        }


def run_v3_reproduction_audit(tolerance_pct: float = 5.0) -> Dict[str, Any]:
    """
    Executes Stage 2: Auditable v3 reproduction protocol across the 5 reference scenarios.
    Saves results to results/v3_reproduction_report.json.
    """
    print("=" * 80)
    print("STAGE 2: AUDITABLE V3 BASELINE REPRODUCTION VERIFICATION")
    print("=" * 80)

    adapter = V3Adapter()
    scenarios_path = BASE_DIR / "data" / "test_scenarios_v3.pkl"
    if not scenarios_path.exists():
        scenarios_path = REPO_ROOT / "v3_pino_dr" / "data" / "test_scenarios_v3.pkl"
    if not scenarios_path.exists():
        scenarios_path = BASE_DIR / "data" / "test_scenarios_v9.pkl"
    with open(scenarios_path, "rb") as f:
        test_scenarios = pickle.load(f)

    report: Dict[str, Any] = {
        "timestamp": None,
        "tolerance_pct": tolerance_pct,
        "all_passed": True,
        "scenarios": {},
    }

    print(f"{'Scenario':<15} | {'Seqs':<5} | {'Expected (m)':<12} | {'Actual (m)':<11} | {'Abs Diff (m)':<12} | {'Diff (%)':<9} | {'Status'}")
    print("-" * 85)

    all_scen_pass = True

    for scen_name, target in V3_TARGETS.items():
        journeys = test_scenarios[scen_name]
        drifts = []
        for j in journeys:
            n = len(j["x_gps"])
            for st in range(WINDOW + 1, n - 10, 10):
                res = adapter.simulate_sequence(j, st, outage_len=10)
                drifts.append(res["drift_v3"])

        mean_drift = float(np.mean(drifts))
        abs_diff = abs(mean_drift - target)
        pct_diff = (abs_diff / target) * 100.0
        passed = bool(pct_diff <= tolerance_pct)

        if not passed:
            all_scen_pass = False

        status_str = "PASS" if passed else "FAIL"
        print(f"{scen_name:<15} | {len(drifts):>5} | {target:>10.2f} m | {mean_drift:>9.2f} m | {abs_diff:>10.2f} m | {pct_diff:>7.2f} % | {status_str}")

        report["scenarios"][scen_name] = {
            "n_sequences": len(drifts),
            "expected_drift_m": target,
            "actual_drift_m": mean_drift,
            "absolute_diff_m": abs_diff,
            "percentage_diff": pct_diff,
            "passed": passed,
        }

    report["all_passed"] = all_scen_pass
    out_file = BASE_DIR / "results" / "v3_reproduction_report.json"
    out_file.parent.mkdir(parents=True, exist_ok=True)
    with open(out_file, "w") as f:
        json.dump(report, f, indent=2)

    print(f"\nSaved v3 reproduction report to: {out_file}")
    return report


if __name__ == "__main__":
    run_v3_reproduction_audit()
