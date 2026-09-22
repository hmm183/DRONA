"""
v9_adaptive_projection_dr/src/generate_report_graphs.py
-------------------------------------------------------
Standardized publication-grade benchmark report plotting tool for V9 Adaptive Projection DR.
Supports configurable inclusion/exclusion of specific scenarios via CLI arguments.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent.parent
if str(BASE_DIR) not in sys.path:
    sys.path.insert(0, str(BASE_DIR))

# Import underlying plotting implementations
from src.generate_report_graphs_no_roundabout import (
    plot_bar_chart_no_roundabouts,
    plot_multi_horizon_growth_no_roundabouts,
    plot_trajectory_mosaic_no_roundabouts,
    plot_individual_trajectories,
    RESULTS_DIR
)


def run_all_plots(exclude_roundabouts: bool = True):
    print(f"Generating benchmark report figures (Exclude roundabouts: {exclude_roundabouts})...")
    plot_bar_chart_no_roundabouts()
    plot_multi_horizon_growth_no_roundabouts()
    plot_trajectory_mosaic_no_roundabouts()
    plot_individual_trajectories()
    print(f"All figures generated successfully in: {RESULTS_DIR}")


def main():
    parser = argparse.ArgumentParser(description="Generate V9 Benchmark Report Graphs")
    parser.add_argument(
        "--exclude-roundabouts",
        action="store_true",
        default=True,
        help="Exclude unconstrained roundabout scenarios from macro aggregation (default: True)"
    )
    args = parser.parse_args()
    run_all_plots(exclude_roundabouts=args.exclude_roundabouts)


if __name__ == "__main__":
    main()
