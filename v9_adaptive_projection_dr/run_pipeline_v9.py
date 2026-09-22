"""
v9_adaptive_projection_dr/run_pipeline_v9.py
---------------------------------------------
Master End-to-End Orchestrator CLI for v9 Adaptive Projection DR.

Commands & Steps:
  --step audit              : Runs smartphone dataset audit (Stage 3)
  --step preprocess         : Generates splits, scalers, metadata (Stage 4-6)
  --step test               : Runs pre-training physics & unit tests (Stage 7)
  --step train_projection   : Trains AdaptiveStateProjectionNet (Stage 17-20)
  --step eval               : Runs full 10s, 30s, 60s benchmark evaluation (Stage 21)
  --step ablation           : Runs 8-way ablation study & recovery analysis (Stage 22)
  --step all                : Runs complete pipeline sequentially from scratch

Usage:
  python run_pipeline_v9.py --step all
  python run_pipeline_v9.py --step eval --horizon 10
  python run_pipeline_v9.py --step test
"""

from __future__ import annotations

import os
import sys
import argparse
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent
REPO_ROOT = BASE_DIR.parent
if str(BASE_DIR) not in sys.path:
    sys.path.insert(0, str(BASE_DIR))
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from src.v3_adapter import run_v3_reproduction_audit
from src.preprocess_v9 import run_dataset_audit, generate_trip_splits_and_scalers
from tests.test_units import run_all_tests
from src.train_projection_v9 import train_projection_network
from src.evaluate_v9 import run_full_evaluation
from src.ablation_v9 import run_ablation_study


def main():
    parser = argparse.ArgumentParser(description="v9 Adaptive Projection Dead Reckoning CLI")
    parser.add_argument(
        "--step",
        type=str,
        default="all",
        choices=["audit", "v3_reproduce", "preprocess", "test", "train_projection", "eval", "ablation", "all"],
        help="Pipeline step to execute",
    )
    parser.add_argument("--epochs", type=int, default=12, help="Epochs for projection network training")
    parser.add_argument("--device", type=str, default="cpu", help="Device (cpu or cuda)")
    parser.add_argument("--horizon", type=float, default=10.0, help="Evaluation horizon (10.0, 30.0, 60.0)")

    args = parser.parse_args()

    print("=" * 80)
    print("V9 ADAPTIVE PROJECTION DEAD RECKONING (AUTOMOTIVE SMARTPHONE DR)")
    print(f"Executing step: {args.step.upper()}")
    print("=" * 80)

    if args.step in ["audit", "all"]:
        print("\n>>> STEP 1: Smartphone Dataset Audit")
        run_dataset_audit()

    if args.step in ["v3_reproduce", "all"]:
        print("\n>>> STEP 2: V3 Baseline Reproduction Audit")
        v3_pass = run_v3_reproduction_audit()
        if not v3_pass:
            print("[WARN] V3 reproduction tolerance check reported non-zero delta.")

    if args.step in ["preprocess", "all"]:
        print("\n>>> STEP 3: Dataset Splitting & Feature Scaling")
        generate_trip_splits_and_scalers()

    if args.step in ["test", "all"]:
        print("\n>>> STEP 4: Pre-Training Physics & Kinematics Verification Tests")
        tests_passed = run_all_tests()
        if not tests_passed:
            print("[ERROR] Verification tests failed! Halting pipeline.")
            sys.exit(1)

    if args.step in ["train_projection", "all"]:
        print("\n>>> STEP 5: Curriculum Training of Adaptive State Projection Network")
        train_projection_network(epochs=args.epochs, device=args.device)

    if args.step in ["eval", "all"]:
        print("\n>>> STEP 6: Multi-Horizon Benchmark Evaluation (10s, 30s, 60s)")
        run_full_evaluation()

    if args.step in ["ablation", "all"]:
        print("\n>>> STEP 7: 8-Way Systematic Ablation Study & Recovery Analysis")
        run_ablation_study(horizon_s=args.horizon)

    print("\n" + "=" * 80)
    print("V9 PIPELINE EXECUTION COMPLETED SUCCESSFULLY")
    print("=" * 80)


if __name__ == "__main__":
    main()
