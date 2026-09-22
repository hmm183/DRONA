# v8 Inertial Dead Reckoning Benchmark

This directory contains benchmark data and dataset extraction utilities for the V8 Dead Reckoning model architecture (`v8_dead_reckoning.onnx`).

## Contents
- `outage_results_v8.csv`: Benchmark blackout drift evaluation across standard 10s, 30s, and 60s GNSS outage horizons.
- `src/io_own_field_data.py`: Reader and parser for raw multi-sensor field recordings collected from phone hardware.

## Relationship to V9
V8 represents the pure sample-by-sample neural dead reckoning baseline. In V9 (`v9_adaptive_projection_dr/`), this architecture is enhanced with Periodic Adaptive Trajectory Projection (PATP), multi-horizon error-state EKF innovation, and driving-event adaptive gating.
