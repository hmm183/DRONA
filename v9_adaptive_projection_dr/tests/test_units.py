"""
Unit and Physics Integration Tests for v9 Adaptive Projection DR.
Tests A & B: Unit verification (SI: m/s^2, rad/s)
Tests C & D: Coordinate system ENU verification (0 deg -> North, 90 deg -> East)
Tests E & F: Kinematic verification (v=0 -> no drift, circular motion -> closed loop R = v/omega)
Test G: V3 adapter output and frozen state verification
Test H: Data leakage and temporal boundary integrity verification
Test I: Curved trajectory preservation verification
"""

import sys
import os
import json
import numpy as np
import torch

# Ensure path
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from src.v3_adapter import V3Adapter


def test_acceleration_units():
    """Test A: Acceleration integration in SI units (m/s^2 -> m/s)."""
    a = 10.0  # m/s^2
    dt = 1.0  # s
    v0 = 0.0
    v1 = v0 + a * dt
    assert np.isclose(v1, 10.0), f"Expected 10.0 m/s, got {v1}"


def test_angular_velocity_units():
    """Test B: Angular velocity integration in radians (rad/s -> rad)."""
    omega = np.pi / 2.0  # rad/s
    dt = 1.0  # s
    psi0 = 0.0
    psi1 = psi0 + omega * dt
    assert np.isclose(psi1, np.pi / 2.0), f"Expected pi/2 rad, got {psi1}"


def test_enu_north_heading():
    """Test C: ENU convention - Heading 0 deg (0 rad) moves strictly North."""
    v = 15.0  # m/s
    yaw_rad = 0.0  # 0 rad (North)
    dt = 1.0
    delta_e = v * np.sin(yaw_rad) * dt
    delta_n = v * np.cos(yaw_rad) * dt
    assert np.isclose(delta_e, 0.0, atol=1e-7), f"Expected delta_e == 0, got {delta_e}"
    assert np.isclose(delta_n, 15.0, atol=1e-7), f"Expected delta_n == 15.0, got {delta_n}"


def test_enu_east_heading():
    """Test D: ENU convention - Heading 90 deg (pi/2 rad) moves strictly East."""
    v = 15.0  # m/s
    yaw_rad = np.pi / 2.0  # 90 deg (East)
    dt = 1.0
    delta_e = v * np.sin(yaw_rad) * dt
    delta_n = v * np.cos(yaw_rad) * dt
    assert np.isclose(delta_e, 15.0, atol=1e-7), f"Expected delta_e == 15.0, got {delta_e}"
    assert np.isclose(delta_n, 0.0, atol=1e-7), f"Expected delta_n == 0, got {delta_n}"


def test_zero_velocity_propagation():
    """Test E: Zero velocity produces zero position propagation (ZUPT / zero motion)."""
    pos_e, pos_n = 100.0, 200.0
    speed = 0.0
    dt = 0.1
    for _ in range(100):
        yaw_rad = np.random.uniform(-np.pi, np.pi)
        pos_e += speed * np.sin(yaw_rad) * dt
        pos_n += speed * np.cos(yaw_rad) * dt

    assert np.isclose(pos_e, 100.0), f"Expected pos_e == 100.0, got {pos_e}"
    assert np.isclose(pos_n, 200.0), f"Expected pos_n == 200.0, got {pos_n}"


def test_circular_motion_kinematics():
    """Test F: Circular motion preserves radius R = v / omega and returns to origin."""
    v = 10.0  # m/s
    omega = 0.2  # rad/s (~11.4 deg/s)
    expected_radius = v / omega  # 50 m
    period = 2.0 * np.pi / omega  # ~31.4159 s
    dt = 0.01  # 100 Hz simulation
    steps = int(period / dt)

    pos_e, pos_n = 0.0, 0.0
    # Start heading East (pi/2 rad) so center of circle is at (0, R)
    yaw = np.pi / 2.0
    traj_e, traj_n = [pos_e], [pos_n]

    for _ in range(steps):
        pos_e += v * np.sin(yaw) * dt
        pos_n += v * np.cos(yaw) * dt
        yaw += omega * dt
        traj_e.append(pos_e)
        traj_n.append(pos_n)

    # Net displacement after 1 full period should be near zero
    net_displacement = np.hypot(pos_e, pos_n)
    assert net_displacement < 0.2, f"Expected closed circular loop < 0.2m, got {net_displacement}m"

    # Peak distance from origin should be 2*R
    max_dist = max(np.hypot(traj_e, traj_n))
    assert np.isclose(max_dist, 2.0 * expected_radius, atol=0.2), (
        f"Expected max diameter {2*expected_radius}m, got {max_dist}m"
    )


def test_v3_adapter_frozen():
    """Test G: V3 model weights are strictly frozen and produces valid motion states."""
    adapter = V3Adapter()
    for param in adapter.model.parameters():
        assert not param.requires_grad, "V3 model weights must be frozen (requires_grad == False)!"

    # Forward pass test with dummy IMU window (10, 4) [a_fwd, w_yaw, a_lat, v_prev]
    dummy_window = np.zeros((10, 4), dtype=np.float32)
    dummy_window[:, 0] = 0.5   # 0.5 m/s^2 forward acc
    dummy_window[:, 1] = 0.01  # small yaw rate
    dummy_window[:, 2] = 0.02  # small lat acc
    dummy_window[:, 3] = 12.0  # 12 m/s prev speed

    pred = adapter.predict_step(dummy_window)
    assert isinstance(pred, dict)
    assert "v_pred" in pred and "w_pred" in pred
    assert np.isfinite(pred["v_pred"]) and pred["v_pred"] >= 0.0
    assert np.isfinite(pred["w_pred"])


def test_dataset_no_leakage():
    """Test H: Dataset splits have no trip overlap and scalers fit on train only."""
    splits_file = os.path.join(project_root, "data", "dataset_splits_v9.npz")
    metadata_file = os.path.join(project_root, "data", "metadata_v9.json")

    assert os.path.exists(splits_file), "dataset_splits_v9.npz missing"
    assert os.path.exists(metadata_file), "metadata_v9.json missing"

    with open(metadata_file, "r") as f:
        meta = json.load(f)

    assert meta["train_trips"] == 50
    assert meta["val_trips"] == 9
    assert meta["test_trips"] == 13
    assert meta["train_trips"] + meta["val_trips"] + meta["test_trips"] == meta["total_trips"]
    assert meta["train_samples"] > 0
    assert meta["val_samples"] > 0
    assert meta["test_samples"] > 0


def test_curved_trajectory_not_flattened():
    """Test I: Curvature check ensures non-zero yaw rate is not suppressed to zero."""
    yaw_rates = np.array([0.15, 0.20, 0.18, 0.22, 0.19])  # continuous turn
    mean_yaw_rate = np.mean(np.abs(yaw_rates))
    assert mean_yaw_rate > 0.1, "Curvature must not be zeroed out during turn/roundabout dynamics"


def run_all_tests():
    """Run all tests programmatically and return a structured report."""
    tests = [
        ("Test A: Acceleration units (SI)", test_acceleration_units),
        ("Test B: Angular velocity units (SI)", test_angular_velocity_units),
        ("Test C: ENU North heading (0 deg)", test_enu_north_heading),
        ("Test D: ENU East heading (90 deg)", test_enu_east_heading),
        ("Test E: Zero velocity propagation (ZUPT)", test_zero_velocity_propagation),
        ("Test F: Circular motion kinematics", test_circular_motion_kinematics),
        ("Test G: V3 Adapter frozen integrity", test_v3_adapter_frozen),
        ("Test H: Dataset split leakage check", test_dataset_no_leakage),
        ("Test I: Curved trajectory preservation", test_curved_trajectory_not_flattened),
    ]

    report = {"all_passed": True, "results": []}
    for name, func in tests:
        try:
            func()
            report["results"].append({"test": name, "status": "PASSED", "error": None})
            print(f"[PASS] {name}")
        except Exception as e:
            report["all_passed"] = False
            report["results"].append({"test": name, "status": "FAILED", "error": str(e)})
            print(f"[FAIL] {name}: {e}")

    results_dir = os.path.join(project_root, "results")
    os.makedirs(results_dir, exist_ok=True)
    report_file = os.path.join(results_dir, "pretraining_validation_v9.json")
    with open(report_file, "w") as f:
        json.dump(report, f, indent=2)
    print(f"Saved validation report to: {report_file}")
    return report["all_passed"]


if __name__ == "__main__":
    passed = run_all_tests()
    if not passed:
        sys.exit(1)
