"""
v9_adaptive_projection_dr/src/visualize_benchmark_projections.py
----------------------------------------------------------------
Standardized benchmark projection visualization and figure generation.
Provides clean programmatic and CLI access to multi-horizon and trajectory plots.
"""

from __future__ import annotations

import sys
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent.parent
if str(BASE_DIR) not in sys.path:
    sys.path.insert(0, str(BASE_DIR))

# Delegate to existing presentation-grade graph pipeline
from src.generate_ppt_graphs import (
    plot_slide_1_bar_chart,
    plot_slide_2_multi_horizon,
    plot_slide_3_error_over_time,
    plot_slide_4_mosaic,
    plot_trajectory_scenarios,
    PPT_DIR
)


def run_visualizations():
    print(f"Generating full benchmark visualization suite in: {PPT_DIR}")
    plot_slide_1_bar_chart()
    plot_slide_2_multi_horizon()
    plot_slide_3_error_over_time()
    plot_slide_4_mosaic()
    plot_trajectory_scenarios()
    print("All benchmark visualizations generated successfully.")


if __name__ == "__main__":
    run_visualizations()
