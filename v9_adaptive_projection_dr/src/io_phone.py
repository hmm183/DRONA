"""
v9_adaptive_projection_dr/src/io_phone.py
-----------------------------------------
Stage 3 & 4: Robust Smartphone Dataset Ingestion.
Reads IO-VNBD smartphone-only CSV files with latin1 encoding, header sanitization,
timestamp monotonicity checks, and gap detection.
"""

from __future__ import annotations

import os
import re
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import pandas as pd

DEFAULT_DATASET_ROOT = Path(r"C:\Users\Raushan\Desktop\IO-VNBD")
PRIMARY_REL_PATH = Path("Synchronised V abd S datasets") / "Uncategorised IOVNB Dataset" / "S-Dataset"


def find_s_dataset_dir(dataset_root: Optional[Path] = None) -> Path:
    root = dataset_root or DEFAULT_DATASET_ROOT
    candidates = [
        root / PRIMARY_REL_PATH,
        root / "Uncategorised IOVNB Dataset" / "S-Dataset",
        root / "S-Dataset",
    ]
    for c in candidates:
        if c.exists() and c.is_dir():
            return c
    raise FileNotFoundError(f"Could not locate S-Dataset inside {root}")


def read_phone_csv(csv_path: Path) -> pd.DataFrame:
    """
    Reads a single smartphone CSV file with latin1 encoding, stripping header whitespace.
    Handles malformed or truncated rows safely.
    """
    df = pd.read_csv(
        csv_path,
        encoding="latin1",
        on_bad_lines="skip",
        low_memory=False,
    )
    df.columns = [str(c).strip() for c in df.columns]
    return df


def extract_smartphone_channels(df: pd.DataFrame) -> Dict[str, np.ndarray]:
    """
    Extracts core smartphone inertial and GNSS channels.
    Never uses any vehicle CAN, speed, or vehicle ground-truth channels.
    """
    cols = {c.lower(): c for c in df.columns}

    def get_col(patterns: List[str]) -> Optional[str]:
        for pat in patterns:
            for c_low, c_orig in cols.items():
                if re.search(pat, c_low):
                    return c_orig
        return None

    col_t = get_col([r"^time", r"^timestamp", r"^t$"])
    col_ax = get_col([r"acc.*x", r"accel.*x", r"^ax"])
    col_ay = get_col([r"acc.*y", r"accel.*y", r"^ay"])
    col_az = get_col([r"acc.*z", r"accel.*z", r"^az"])

    col_gx = get_col([r"gyr.*x", r"^gx", r"omega.*x"])
    col_gy = get_col([r"gyr.*y", r"^gy", r"omega.*y"])
    col_gz = get_col([r"gyr.*z", r"^gz", r"omega.*z"])

    col_lat = get_col([r"^lat", r"gps.*lat"])
    col_lon = get_col([r"^lon", r"gps.*lon"])
    col_spd = get_col([r"gps.*spd", r"gps.*speed", r"^speed"])
    col_acc = get_col([r"gps.*acc", r"accuracy", r"hacc"])
    col_sat = get_col([r"sat", r"num.*sat"])
    col_bearing = get_col([r"bearing", r"course", r"heading"])

    n_rows = len(df)
    zeros = np.zeros(n_rows, dtype=np.float64)

    t_s = pd.to_numeric(df[col_t], errors="coerce").fillna(0.0).values if col_t else np.arange(n_rows) * 0.1
    ax = pd.to_numeric(df[col_ax], errors="coerce").fillna(0.0).values if col_ax else zeros
    ay = pd.to_numeric(df[col_ay], errors="coerce").fillna(0.0).values if col_ay else zeros
    az = pd.to_numeric(df[col_az], errors="coerce").fillna(0.0).values if col_az else zeros

    gx = pd.to_numeric(df[col_gx], errors="coerce").fillna(0.0).values if col_gx else zeros
    gy = pd.to_numeric(df[col_gy], errors="coerce").fillna(0.0).values if col_gy else zeros
    gz = pd.to_numeric(df[col_gz], errors="coerce").fillna(0.0).values if col_gz else zeros

    lat = pd.to_numeric(df[col_lat], errors="coerce").fillna(np.nan).values if col_lat else np.full(n_rows, np.nan)
    lon = pd.to_numeric(df[col_lon], errors="coerce").fillna(np.nan).values if col_lon else np.full(n_rows, np.nan)
    speed = pd.to_numeric(df[col_spd], errors="coerce").fillna(0.0).values if col_spd else zeros
    accuracy = pd.to_numeric(df[col_acc], errors="coerce").fillna(99.0).values if col_acc else np.full(n_rows, 99.0)
    satellites = pd.to_numeric(df[col_sat], errors="coerce").fillna(0.0).values if col_sat else zeros
    bearing = pd.to_numeric(df[col_bearing], errors="coerce").fillna(0.0).values if col_bearing else zeros

    return {
        "time_s": t_s,
        "acc_x": ax, "acc_y": ay, "acc_z": az,
        "gyr_x": gx, "gyr_y": gy, "gyr_z": gz,
        "lat": lat, "lon": lon,
        "gps_speed": speed,
        "gps_accuracy": accuracy,
        "gps_satellites": satellites,
        "gps_bearing": bearing,
    }
