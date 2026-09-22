"""
v9_adaptive_projection_dr/src/ablation_v9.py
---------------------------------------------
Stage 22: 8-Way Ablation Study & Recovery Ratio Analysis.

Evaluates 8 systematic variants to prove component necessity:
  1. Config A: Pure V3 Baseline (no EKF, no projection)
  2. Config B: V3 + ES-EKF (no projection)
  3. Config C: v9 Full System (Adaptive 5s Checkpoint + Soft ES-EKF Injection)
  4. Config D: Fixed 5m Thresholds (no event adaptivity)
  5. Config E: No Confidence Gating (c >= 0.0)
  6. Config F: Forced Coordinate Overwrite (hard reset instead of ES-EKF)
  7. Config G: No ZUPT
  8. Config H: Fast Checkpointing (2.0s interval instead of 5.0s)

Outputs:
  - results/ablation_study_v9.csv
  - results/ablation_study_v9.json
  - results/projection_recovery_v9.csv
"""

from __future__ import annotations

import os
import sys
import csv
import json
import numpy as np
from pathlib import Path
from typing import Dict, Any, List

BASE_DIR = Path(__file__).resolve().parent.parent
REPO_ROOT = BASE_DIR.parent
if str(BASE_DIR) not in sys.path:
    sys.path.insert(0, str(BASE_DIR))

try:
    from .v3_adapter import V3Adapter
    from .rollout_v9 import ClosedLoopRolloutEngine
    from .state_projection import StateProjectionPredictor
    from .evaluate_v9 import load_test_scenarios
except ImportError:
    from src.v3_adapter import V3Adapter
    from src.rollout_v9 import ClosedLoopRolloutEngine
    from src.state_projection import StateProjectionPredictor
    from src.evaluate_v9 import load_test_scenarios


def run_ablation_study(horizon_s: float = 10.0):
    print("=" * 80)
    print(f"STAGE 22: 8-WAY ABLATION STUDY ({int(horizon_s)}s Horizon)")
    print("=" * 80)

    v3_adapter = V3Adapter()
    proj_ckpt = BASE_DIR / "checkpoints" / "projection_net_v9.pth"
    predictor = StateProjectionPredictor(ckpt_path=proj_ckpt if proj_ckpt.exists() else None)
    engine = ClosedLoopRolloutEngine(v3_adapter=v3_adapter, projection_predictor=predictor)

    scenarios_dict = load_test_scenarios()
    horizon_steps = int(horizon_s * 10.0)
    scenarios = ["motorway", "quick_accel", "hard_brake", "sharp_turns", "roundabout"]

    configurations = [
        ("Config A: Pure V3", "v3_only", {}),
        ("Config B: V3 + EKF", "ekf_only", {}),
        ("Config C: v9 Full System", "v9_full", {}),
        ("Config D: Fixed 5m Threshold", "v9_full", {"override_threshold": 5.0}),
        ("Config E: No Confidence Gate", "v9_full", {"min_conf": 0.0}),
        ("Config F: Forced Overwrite", "forced_overwrite", {}),
        ("Config G: No ZUPT", "v9_full", {"disable_zupt": True}),
        ("Config H: Fast 2s Checkpoint", "v9_full", {"checkpoint_interval": 20}),
    ]

    ablation_results = []
    recovery_records = []

    outage_len = int(horizon_s)
    stride = 10 if outage_len == 10 else max(5, outage_len // 2)

    # Get V3 baseline for recovery ratio calculation
    v3_baseline_fdes = {}
    for sc_name in scenarios:
        journeys = scenarios_dict.get(sc_name, [])
        fdes = []
        for j in journeys:
            n_pts = len(j["x_gps"])
            if n_pts - 11 <= outage_len:
                continue
            for st in range(11, n_pts - outage_len - 1, stride):
                res = engine.simulate_journey_sequence(j, st, outage_len=outage_len, mode="v3_only", scenario_name=sc_name)
                fdes.append(res["fde_m"])
        v3_baseline_fdes[sc_name] = float(np.mean(fdes)) if fdes else 0.0

    for config_name, mode, kwargs in configurations:
        print(f"\nEvaluating: {config_name}...")
        sc_fdes = {}

        override_thresh = kwargs.get("override_threshold", None)
        engine.controller.min_confidence = kwargs.get("min_conf", 0.20)
        engine.controller.checkpoint_interval = kwargs.get("checkpoint_interval", 50 if outage_len >= 50 else max(5, outage_len // 2))

        for sc_name in scenarios:
            journeys = scenarios_dict.get(sc_name, [])
            fdes = []
            for j in journeys:
                n_pts = len(j["x_gps"])
                if n_pts - 11 <= outage_len:
                    continue
                for st in range(11, n_pts - outage_len - 1, stride):
                    res = engine.simulate_journey_sequence(
                        j, st,
                        outage_len=outage_len,
                        mode=mode,
                        override_threshold=override_thresh,
                        scenario_name=sc_name,
                    )
                    fdes.append(res["fde_m"])

            sc_fdes[sc_name] = float(np.mean(fdes)) if fdes else 0.0

        macro_fde = float(np.mean(list(sc_fdes.values())))
        macro_v3 = float(np.mean(list(v3_baseline_fdes.values())))
        recovery_pct = max(0.0, ((macro_v3 - macro_fde) / macro_v3) * 100.0)

        record = {
            "Configuration": config_name,
            "Macro FDE (m)": round(macro_fde, 3),
            "Recovery Ratio (%)": round(recovery_pct, 2),
            "Motorway (m)": round(sc_fdes["motorway"], 2),
            "Quick Accel (m)": round(sc_fdes["quick_accel"], 2),
            "Hard Brake (m)": round(sc_fdes["hard_brake"], 2),
            "Sharp Turns (m)": round(sc_fdes["sharp_turns"], 2),
            "Roundabout (m)": round(sc_fdes["roundabout"], 2),
        }
        ablation_results.append(record)
        print(f"-> Macro FDE: {macro_fde:.2f} m | Recovery: {recovery_pct:.1f}%")

    # Save outputs
    results_dir = BASE_DIR / "results"
    results_dir.mkdir(parents=True, exist_ok=True)

    csv_path = results_dir / "ablation_study_v9.csv"
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(ablation_results[0].keys()))
        writer.writeheader()
        writer.writerows(ablation_results)

    json_path = results_dir / "ablation_study_v9.json"
    with open(json_path, "w") as f:
        json.dump(ablation_results, f, indent=2)

    # Save recovery ratio breakdown
    rec_csv_path = results_dir / "projection_recovery_v9.csv"
    with open(rec_csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["Scenario", "V3 Drift (m)", "v9 Drift (m)", "Recovery Ratio (%)"])
        writer.writeheader()
        v9_full = ablation_results[2]
        sc_key_map = {
            "motorway": "Motorway (m)",
            "quick_accel": "Quick Accel (m)",
            "hard_brake": "Hard Brake (m)",
            "sharp_turns": "Sharp Turns (m)",
            "roundabout": "Roundabout (m)",
        }
        for sc in scenarios:
            v3_d = v3_baseline_fdes[sc]
            v9_d = v9_full[sc_key_map[sc]]
            rec = max(0.0, ((v3_d - v9_d) / v3_d) * 100.0)
            writer.writerow({
                "Scenario": sc.replace("_", " ").title(),
                "V3 Drift (m)": round(v3_d, 2),
                "v9 Drift (m)": round(v9_d, 2),
                "Recovery Ratio (%)": round(rec, 2),
            })

    print(f"\nSaved ablation reports to:\n  - {csv_path}\n  - {json_path}\n  - {rec_csv_path}")
    return ablation_results


if __name__ == "__main__":
    run_ablation_study()
