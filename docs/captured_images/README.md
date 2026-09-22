# DRONA — Captured Telemetry, Device & Simulation Visuals

This directory contains all **184 visual media captures** recorded across all DRONA development stages, on-device road runs, GNSS-blackout stress tests, and web digital twin simulations.

## Visual Categories

### 1. Web Digital Twin & Simulation Screen Captures
- `initial_page_load_*.png` — Digital twin simulation dashboard initial state, Leaflet OpenStreetMap layer, HUD dials, and route selector.
- `controls_view_*.png` — Simulation controls drawer, speed multipliers (1x–10x), GNSS cut-off switch, and outage configuration dialog.
- `running_sim_map_*.png` — Dynamic real-time playback showing vehicle marker, live dead-reckoning trajectory, and multi-outage segment transitions.
- `running_sim_chart_*.png` & `chart_running_view_*.png` — Real-time comparative error drift chart plotting DRONA (V9 ML + IMM-UKF) against naive unconstrained DR.

### 2. Android Device Telemetry & Live Runs
- `media_*.png` (118 screenshots) — High-resolution screen captures from physical device runs (`adb screencap`), capturing:
  - Full-screen automotive navigation UI with turn-by-turn instruction card.
  - GNSS status badge, satellite constellation SNR graphs, and Dilution of Precision (DOP).
  - Outage simulation blackout triggers, crimson outage pathing, and recovery re-anchoring.
  - Trip history, session metrics, GPX/CSV export workflows, and sensor diagnostic readouts.
- `media_*.jpg` (7 captures) — Photographic and external camera captures recorded during hardware setup and orientation calibration.
- `after_*.png`, `screen_*.png`, `check_*.png` — Key state transitions captured on-device (splash launch, cockpit HUD, demo mode, 2.5x simulation speed, and route selection).

## Location
All files are preserved directly in git at `docs/captured_images/`.
