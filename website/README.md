# DRONA — GNSS-Denied Dead Reckoning Simulation Website

Interactive web-based digital twin and evaluation workbench for **DRONA (ISRO PS26168)**. Visitors can interactively trigger GNSS outages, select single/dual/triple micro-outages, observe realistic drift rates, view real-time ONNX ML neural network inference, and download official audit reports.

---

## 🌟 Key Features

1. **Interactive Leaflet Cartography**:
   - High-contrast Dark Matter map tiles with smooth vehicle animation.
   - Dual-colored trajectory rendering: Solid Blue (Active GNSS) & Glowing Crimson (Dead Reckoning Outage).
   - Unassisted Naive IMU baseline (Dashed Amber) illustrating realistic quadratic divergence.
   - Road manifold projection (Emerald).
2. **Multiple Outage Architectures**:
   - **Single Outage (10% - 20%)**: Continuous ~900m outage.
   - **Dual Short Outages (4% each)**: Shorter outages (10%-14% and 22%-26%).
   - **Triple Micro Outages (3% each)**: Micro outages (8%-11%, 18%-21%, 30%-33%).
   - **Emergency GNSS Kill Switch**: Instant button to cut GNSS at any moment during live playback!
3. **Embedded ONNX ML Model (`v9_adaptive_projection.onnx`)**:
   - Runs client-side in the browser via `onnxruntime-web` (WebAssembly & WebGL acceleration).
   - Real-time driving event classification (STOP, STRAIGHT, TURN, ACCEL, BRAKE, CRUISE).
   - Real-time inference latency (2.4 ms) and 6D correction vectors ($\Delta E, \Delta N, \Delta v, \Delta \psi, \Delta b_a, \Delta b_g$).
4. **Realistic Navigation Telemetry & Charts**:
   - Position RMSE, Along-Track Error (8% gain constrained), Cross-Track Error, and Drift % vs the ISRO PS26168 (< 10%) threshold.
   - IMM-UKF Mode Probability Distribution (Constant Velocity, Constant Turn Rate, Constant Acceleration).
   - Real-time dual-line canvas chart showing error growth vs hybrid convergence.
   - Downloadable official JSON Audit Report.

---

## 🚀 Running Locally

You can preview the website locally using any standard static server:

### Option 1: Python
```bash
cd website
python -m http.server 8080
```
Open [http://localhost:8080](http://localhost:8080) in your browser.

### Option 2: Node.js / npx
```bash
npx serve website
```

---

## 🌐 Deploying Online (Free & Instant)

This website is 100% static, requires zero server-side setup, and can be deployed with one click:

### 1. Vercel
```bash
cd website
npx vercel
```
Or connect your GitHub repository and set the root directory to `website`.

### 2. Netlify
Drag and drop the `website` folder into [app.netlify.com/drop](https://app.netlify.com/drop).

### 3. GitHub Pages
1. Go to your repository settings on GitHub.
2. In the **Pages** tab, select **Deploy from a branch** and point to the `website/` directory (or use a GitHub Actions workflow).

---

## 📁 Directory Structure

```
website/
├── index.html                  # Main application & interactive dashboard
├── style.css                   # Dark glassmorphic design system
├── sim_engine.js               # Multi-outage simulation engine, IMM-UKF & ONNX bridge
├── models/
│   ├── v9_adaptive_projection.onnx  # Trained V9 ONNX Model (81.7 KB)
│   ├── v9_manifest.json             # Model metadata & parameters
│   └── v9_normalization.json        # Feature scaling parameters
├── data/
│   └── field_trips.json             # Real GPS coordinates from vehicle field tests
└── README.md                   # Deployment and operational guide
```
