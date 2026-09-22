"""
v9_adaptive_projection_dr/src/evaluate_v9.py
---------------------------------------------
Stage 21: Full Benchmark Evaluation Protocol (10s, 30s, 60s).

Evaluates Config A (V3), Config B (V3 + EKF), and Config C (v9 Full Adaptive Projection)
across the 5 benchmark scenarios:
  1. Motorway
  2. Quick Accel
  3. Hard Brake
  4. Sharp Turns
  5. Roundabout

Generates:
  - results/benchmark_comparison_v9.json
  - results/benchmark_10s_v9.csv
  - results/benchmark_30s_v9.csv
  - results/benchmark_60s_v9.csv
"""

from __future__ import annotations

import os
import sys
import csv
import json
import pickle
import numpy as np
from pathlib import Path
from typing import Dict, Any, List, Optional

BASE_DIR = Path(__file__).resolve().parent.parent
REPO_ROOT = BASE_DIR.parent
if str(BASE_DIR) not in sys.path:
    sys.path.insert(0, str(BASE_DIR))

try:
    from .v3_adapter import V3Adapter, V3_TARGETS
    from .rollout_v9 import ClosedLoopRolloutEngine
    from .state_projection import StateProjectionPredictor
except ImportError:
    from src.v3_adapter import V3Adapter, V3_TARGETS
    from src.rollout_v9 import ClosedLoopRolloutEngine
    from src.state_projection import StateProjectionPredictor


def load_test_scenarios() -> Dict[str, List[Dict[str, Any]]]:
    """Loads clean test scenario dictionary."""
    scenarios_path = BASE_DIR / "data" / "test_scenarios_v9.pkl"
    if not scenarios_path.exists():
        scenarios_path = REPO_ROOT / "v3_pino_dr" / "data" / "clean_v3" / "test_scenarios_v3.pkl"
    with open(scenarios_path, "rb") as f:
        return pickle.load(f)


def evaluate_horizon(
    engine: ClosedLoopRolloutEngine,
    scenarios_dict: Dict[str, List[Dict[str, Any]]],
    horizon_s: float = 10.0,
) -> Dict[str, Any]:
    """
    Evaluates Config A, Config B, and Config C across all scenarios for a given horizon.
    horizon_s: 10.0, 30.0, or 60.0 seconds.
    """
    # Slicing: for 10s benchmark matching reference v3 targets, stride = 10
    outage_len = int(horizon_s)
    stride = 10 if outage_len == 10 else max(5, outage_len // 2)

    scenarios = ["motorway", "quick_accel", "hard_brake", "sharp_turns", "roundabout"]
    results_by_scenario = {}

    for sc_name in scenarios:
        journeys = scenarios_dict.get(sc_name, [])
        if not journeys:
            continue

        metrics = {
            "v3_fde": [], "v3_rmse": [],
            "ekf_fde": [], "ekf_rmse": [],
            "v9_fde": [], "v9_rmse": [],
            "v9_projections": [],
        }

        for j in journeys:
            n_pts = len(j["x_gps"])
            if n_pts - 11 <= outage_len:
                continue
            for st in range(11, n_pts - outage_len - 1, stride):
                # 1. Config A: Pure V3
                res_a = engine.simulate_journey_sequence(j, st, outage_len=outage_len, mode="v3_only", scenario_name=sc_name)
                metrics["v3_fde"].append(res_a["fde_m"])
                metrics["v3_rmse"].append(res_a["rmse_m"])

                # 2. Config B: V3 + EKF
                res_b = engine.simulate_journey_sequence(j, st, outage_len=outage_len, mode="ekf_only", scenario_name=sc_name)
                metrics["ekf_fde"].append(res_b["fde_m"])
                metrics["ekf_rmse"].append(res_b["rmse_m"])

                # 3. Config C: v9 Full Adaptive Projection DR
                res_c = engine.simulate_journey_sequence(j, st, outage_len=outage_len, mode="v9_full", scenario_name=sc_name)
                metrics["v9_fde"].append(res_c["fde_m"])
                metrics["v9_rmse"].append(res_c["rmse_m"])
                metrics["v9_projections"].append(res_c["projections_fired"])

        if len(metrics["v3_fde"]) == 0:
            continue

        v3_mean_fde = float(np.mean(metrics["v3_fde"]))
        ekf_mean_fde = float(np.mean(metrics["ekf_fde"]))
        v9_mean_fde = float(np.mean(metrics["v9_fde"]))

        v3_mean_rmse = float(np.mean(metrics["v3_rmse"]))
        ekf_mean_rmse = float(np.mean(metrics["ekf_rmse"]))
        v9_mean_rmse = float(np.mean(metrics["v9_rmse"]))

        # Improvement percentages relative to V3 baseline
        delta_pct = ((v9_mean_fde - v3_mean_fde) / max(0.01, v3_mean_fde)) * 100.0
        recovery_ratio = max(0.0, ((v3_mean_fde - v9_mean_fde) / max(0.01, v3_mean_fde)) * 100.0)
        beats_v3 = bool(v9_mean_fde < v3_mean_fde)

        results_by_scenario[sc_name] = {
            "n_samples": len(metrics["v3_fde"]),
            "v3_fde_m": round(v3_mean_fde, 3),
            "v3_rmse_m": round(v3_mean_rmse, 3),
            "ekf_fde_m": round(ekf_mean_fde, 3),
            "ekf_rmse_m": round(ekf_mean_rmse, 3),
            "v9_fde_m": round(v9_mean_fde, 3),
            "v9_rmse_m": round(v9_mean_rmse, 3),
            "delta_pct": round(delta_pct, 2),
            "recovery_ratio_pct": round(recovery_ratio, 2),
            "beats_v3": beats_v3,
            "mean_projections": round(float(np.mean(metrics["v9_projections"])), 2),
        }

    # Macro averages
    macro_v3_fde = float(np.mean([v["v3_fde_m"] for v in results_by_scenario.values()]))
    macro_ekf_fde = float(np.mean([v["ekf_fde_m"] for v in results_by_scenario.values()]))
    macro_v9_fde = float(np.mean([v["v9_fde_m"] for v in results_by_scenario.values()]))

    overall = {
        "horizon_s": horizon_s,
        "macro_v3_fde_m": round(macro_v3_fde, 3),
        "macro_ekf_fde_m": round(macro_ekf_fde, 3),
        "macro_v9_fde_m": round(macro_v9_fde, 3),
        "macro_improvement_pct": round(((macro_v3_fde - macro_v9_fde) / macro_v3_fde) * 100.0, 2),
        "all_beat_v3": all(v["beats_v3"] for v in results_by_scenario.values()),
        "scenarios": results_by_scenario,
    }

    return overall


def save_horizon_csv(horizon_data: Dict[str, Any], out_path: Path):
    """Saves benchmark results to clean CSV."""
    rows = []
    scs = horizon_data["scenarios"]
    for sc_name, m in scs.items():
        rows.append({
            "Scenario": sc_name.capitalize().replace("_", " "),
            "Samples": m["n_samples"],
            "Config A (V3) FDE (m)": m["v3_fde_m"],
            "Config B (EKF) FDE (m)": m["ekf_fde_m"],
            "Config C (v9 Full) FDE (m)": m["v9_fde_m"],
            "v9 RMSE (m)": m["v9_rmse_m"],
            "Delta vs V3 (%)": f"{m['delta_pct']:+.1f}%",
            "Recovery Ratio (%)": f"{m['recovery_ratio_pct']:.1f}%",
            "Beats V3": "YES" if m["beats_v3"] else "NO",
            "Mean Projections": m["mean_projections"],
        })

    keys = list(rows[0].keys())
    with open(out_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)


def run_full_evaluation():
    """Runs complete benchmark evaluation for 10s, 30s, and 60s horizons."""
    print("=" * 80)
    print("STAGE 21: FULL BENCHMARK EVALUATION (10s, 30s, 60s)")
    print("=" * 80)

    # Initialize components
    v3_adapter = V3Adapter()
    proj_ckpt = BASE_DIR / "checkpoints" / "projection_net_v9.pth"
    predictor = StateProjectionPredictor(ckpt_path=proj_ckpt if proj_ckpt.exists() else None)
    engine = ClosedLoopRolloutEngine(v3_adapter=v3_adapter, projection_predictor=predictor)

    scenarios_dict = load_test_scenarios()
    results_dir = BASE_DIR / "results"
    results_dir.mkdir(parents=True, exist_ok=True)

    full_report = {}

    for horizon in [10.0, 30.0, 60.0]:
        print(f"\n--- Evaluating Horizon: {int(horizon)}s ---")
        h_data = evaluate_horizon(engine, scenarios_dict, horizon_s=horizon)
        full_report[f"{int(horizon)}s"] = h_data

        csv_path = results_dir / f"benchmark_{int(horizon)}s_v9.csv"
        save_horizon_csv(h_data, csv_path)

        # Print clean terminal table
        print(f"{'Scenario':<15} | {'V3 FDE (m)':<10} | {'EKF FDE (m)':<11} | {'v9 FDE (m)':<10} | {'Delta':<8} | {'Beats V3'}")
        print("-" * 75)
        for sc_name, m in h_data["scenarios"].items():
            print(f"{sc_name:<15} | {m['v3_fde_m']:<10.2f} | {m['ekf_fde_m']:<11.2f} | {m['v9_fde_m']:<10.2f} | {m['delta_pct']:>+6.1f}% | {'YES' if m['beats_v3'] else 'NO'}")
        print("-" * 75)
        print(f"{'MACRO MEAN':<15} | {h_data['macro_v3_fde_m']:<10.2f} | {h_data['macro_ekf_fde_m']:<11.2f} | {h_data['macro_v9_fde_m']:<10.2f} | -{h_data['macro_improvement_pct']:.1f}% | {'ALL PASS' if h_data['all_beat_v3'] else 'CHECK'}")

    json_path = results_dir / "benchmark_comparison_v9.json"
    with open(json_path, "w") as f:
        json.dump(full_report, f, indent=2)

    print(f"\nSaved comprehensive benchmark report to: {json_path}")
    return full_report


if __name__ == "__main__":
    run_full_evaluation()
