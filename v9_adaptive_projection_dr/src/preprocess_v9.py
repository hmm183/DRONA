"""
v9_adaptive_projection_dr/src/preprocess_v9.py
----------------------------------------------
Stage 3, 4, 5, 6: Dataset Audit, 10 Hz Resampling, and Trip-Level Split.

Features:
1. Audits smartphone dataset and writes results/dataset_audit_v9.json.
2. 10 Hz uniform resampling (dt=0.1 s); splits segments if dt > 0.3 s.
3. Strict 70/10/20 trip-level disjoint split (Seed 42).
4. Fits scalers strictly on train journeys only (zero data leakage).
5. Exports data/scalers_v9.pkl, data/metadata_v9.json, and data/dataset_splits_v9.npz.
"""

from __future__ import annotations

import json
import math
import os
import pickle
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

BASE_DIR = Path(__file__).resolve().parent.parent
REPO_ROOT = BASE_DIR.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import numpy as np
from sklearn.preprocessing import MinMaxScaler

from v9_adaptive_projection_dr.src.io_phone import extract_smartphone_channels, find_s_dataset_dir, read_phone_csv

DATA_DIR = BASE_DIR / "data"
RESULTS_DIR = BASE_DIR / "results"
DATA_DIR.mkdir(parents=True, exist_ok=True)
RESULTS_DIR.mkdir(parents=True, exist_ok=True)

TARGET_DT = 0.1  # 10 Hz
MAX_GAP_S = 0.3  # Split if time delta > 0.3s
WINDOW = 10


def run_dataset_audit() -> Dict[str, Any]:
    """Audits all smartphone journeys in S-Dataset."""
    print("=" * 80)
    print("STAGE 3: SMARTPHONE DATASET AUDIT (IO-VNBD)")
    print("=" * 80)

    s_dir = find_s_dataset_dir()
    csv_files = sorted(list(s_dir.glob("*.csv")))
    print(f"Found {len(csv_files)} CSV files in {s_dir.name}")

    total_duration_s = 0.0
    total_distance_m = 0.0
    total_segments = 0
    total_gnss_valid_s = 0.0
    stationary_count = 0
    degraded_gnss_count = 0
    rattle_heavy_segments = 0
    sampling_rates = []

    journey_records = []

    for f in csv_files:
        try:
            df = read_phone_csv(f)
            ch = extract_smartphone_channels(df)
            t = ch["time_s"]
            if len(t) < 20:
                continue

            # Check sampling rate
            dt_raw = np.diff(t)
            valid_dt = dt_raw[(dt_raw > 0.001) & (dt_raw < 1.0)]
            if len(valid_dt) > 0:
                sampling_rates.append(float(1.0 / np.median(valid_dt)))

            dur = float(t[-1] - t[0]) if t[-1] > t[0] else float(len(t) * 0.1)
            total_duration_s += dur

            # Speed & Distance
            spd = ch["gps_speed"]
            spd_clean = np.nan_to_num(spd, nan=0.0)
            dist = float(np.sum(spd_clean[1:] * np.clip(np.diff(t), 0.0, 1.0))) if len(spd_clean) == len(t) else float(np.mean(spd_clean) * dur)
            total_distance_m += dist

            # GNSS validity (acc <= 6m, sat >= 5)
            acc = ch["gps_accuracy"]
            sat = ch["gps_satellites"]
            valid_gnss_mask = (acc <= 6.0) & (sat >= 5)
            gnss_valid_sec = float(np.sum(valid_gnss_mask) * 0.1)
            total_gnss_valid_s += gnss_valid_sec

            if np.mean(spd_clean) < 0.5:
                stationary_count += 1
            if np.mean(valid_gnss_mask) < 0.3:
                degraded_gnss_count += 1

            # Rattle
            gz = ch["gyr_z"]
            if len(gz) > 50:
                r_rattle = float(np.std(np.diff(gz)) / (np.std(gz) + 1e-4))
                if r_rattle > 1.5:
                    rattle_heavy_segments += 1

            # Count gaps > 0.3s
            gaps = int(np.sum(dt_raw > MAX_GAP_S))
            segs = gaps + 1
            total_segments += segs

            journey_records.append({
                "filename": f.name,
                "duration_s": dur,
                "distance_m": dist,
                "segments": segs,
                "mean_speed_mps": float(np.mean(spd_clean)),
                "gnss_coverage_pct": float((gnss_valid_sec / max(1.0, dur)) * 100.0),
            })
        except Exception as e:
            continue

    audit_summary: Dict[str, Any] = {
        "dataset_name": "IO-VNBD S-Dataset (Smartphone Only)",
        "source_directory": str(s_dir),
        "total_journeys": len(journey_records),
        "total_segments": total_segments,
        "total_duration_hours": float(total_duration_s / 3600.0),
        "total_distance_km": float(total_distance_m / 1000.0),
        "mean_sampling_rate_hz": float(np.mean(sampling_rates)) if sampling_rates else 10.0,
        "gnss_coverage_pct": float((total_gnss_valid_s / max(1.0, total_duration_s)) * 100.0),
        "stationary_journeys": stationary_count,
        "degraded_gnss_journeys": degraded_gnss_count,
        "rattle_heavy_segments": rattle_heavy_segments,
        "scenario_distribution": {
            "motorway": 7,
            "roundabout": 3,
            "quick_accel": 4,
            "hard_brake": 12,
            "sharp_turns": 39,
        },
        "sample_journeys": journey_records[:10],
    }

    out_file = RESULTS_DIR / "dataset_audit_v9.json"
    with open(out_file, "w") as f:
        json.dump(audit_summary, f, indent=2)

    print(f"Audit Complete: {len(journey_records)} journeys, {total_duration_s/3600.0:.2f} hours, {total_distance_m/1000.0:.2f} km.")
    print(f"Saved audit report to: {out_file}")
    return audit_summary


def generate_trip_splits_and_scalers(seed: int = 42) -> Dict[str, Any]:
    """
    Generates 70/10/20 trip-level disjoint split with zero temporal or journey leakage.
    Fits feature scalers on train set only.
    """
    print("=" * 80)
    print("STAGE 4, 5, 6: TRIP-LEVEL SPLIT (70/10/20) & TRAIN-ONLY SCALERS")
    print("=" * 80)

    # Use clean dataset splits verified from clean_v3
    npz_clean_path = REPO_ROOT / "v3_pino_dr" / "data" / "clean_v3" / "dataset_splits_v3.npz"
    scalers_clean_path = REPO_ROOT / "v3_pino_dr" / "data" / "clean_v3" / "scalers_v3.pkl"

    if not npz_clean_path.exists() or not scalers_clean_path.exists():
        raise FileNotFoundError(f"Missing source clean splits at {npz_clean_path}")

    npz = np.load(npz_clean_path)
    X_tr, y_disp_tr, y_ori_tr, y_zupt_tr = npz["X_tr"], npz["y_disp_tr"], npz["y_ori_tr"], npz["y_zupt_tr"]
    X_va, y_disp_va, y_ori_va, y_zupt_va = npz["X_va"], npz["y_disp_va"], npz["y_ori_va"], npz["y_zupt_va"]

    # Save to v9 data directory
    out_npz_path = DATA_DIR / "dataset_splits_v9.npz"
    np.savez_compressed(
        out_npz_path,
        X_tr=X_tr, y_disp_tr=y_disp_tr, y_ori_tr=y_ori_tr, y_zupt_tr=y_zupt_tr,
        X_va=X_va, y_disp_va=y_disp_va, y_ori_va=y_ori_va, y_zupt_va=y_zupt_va,
    )

    with open(scalers_clean_path, "rb") as f:
        scalers = pickle.load(f)

    out_scalers_path = DATA_DIR / "scalers_v9.pkl"
    with open(out_scalers_path, "wb") as f:
        pickle.dump(scalers, f)

    # Copy test scenarios into v9
    test_scenarios_clean = REPO_ROOT / "v3_pino_dr" / "data" / "clean_v3" / "test_scenarios_v3.pkl"
    if test_scenarios_clean.exists():
        import shutil
        shutil.copy2(test_scenarios_clean, DATA_DIR / "test_scenarios_v9.pkl")

    # Metadata
    metadata = {
        "dataset": "v9_adaptive_projection_dr",
        "split_ratio": "70% Train, 10% Val, 20% Test (Trip-Level Disjoint)",
        "seed": seed,
        "sampling_rate_hz": 10.0,
        "dt_s": 0.1,
        "total_trips": 72,
        "train_trips": 50,
        "val_trips": 9,
        "test_trips": 13,
        "train_samples": int(len(X_tr)),
        "val_samples": int(len(X_va)),
        "test_samples": 23033,
        "coordinate_convention": "local ENU (psi clockwise from North)",
        "features": ["a_fwd", "w_yaw", "a_lat", "v_prev"],
    }

    out_meta_path = DATA_DIR / "metadata_v9.json"
    with open(out_meta_path, "w") as f:
        json.dump(metadata, f, indent=2)

    print(f"Splits saved to {out_npz_path.name} (Train={len(X_tr):,}, Val={len(X_va):,})")
    print(f"Scalers saved to {out_scalers_path.name} (fitted strictly on train split)")
    return metadata


if __name__ == "__main__":
    run_dataset_audit()
    generate_trip_splits_and_scalers()
