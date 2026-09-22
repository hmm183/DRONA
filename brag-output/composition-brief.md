# Hyperframes Composition Brief: MARK-V Navigation App

## Objective
Create a short, cinematic launch-style brag video for MARK-V ISRO PS26168 Neural-Inertial Navigation System.

## Output
- Composition directory: `brag-output/composition/`
- Rendered video: `brag-output/brag.mp4`
- Format: landscape — 1920x1080
- Duration: 20.0 seconds

## Source Material
- Project root: `c:\Users\vrish\OneDrive\Pictures\NEW-SIM-dead-reck`
- Primary files read: `README.md`, `v9_adaptive_projection_dr/README.md`, `docs/device_screen.png`
- Product name: MARK-V Dead-Reckoning Navigation System
- Tagline / strongest claim: "0.11% outage drift rate — beating ISRO PS26168 mandate by 9x."
- Key UI to showcase: `assets/images/device_screen.png` (actual Android app cockpit screen) with telemetry overlays.
- Copy that must appear verbatim:
  - "GPS SIGNAL LOST // SATELLITES: 0"
  - "FAILOVER ENGAGED: V9 ADAPTIVE PROJECTION ACTIVE"
  - "PATP Transformer: 16.9K Params"
  - "IMM-UKF + RBPF + Sliding Window FGO"
  - "0.11% Outage Drift Rate"
  - "ISRO PS26168 Mandate: < 10%"
  - "130+ Passing Unit Tests"

## Creative Direction
- Tone preset: `cinematic`
- Creative direction: Mission-critical aerospace inertial navigation system launch
- Interpretation: Serious deep-tech aerospace energy, sleek tactical dark aesthetics, bold telemetry data, rapid high-contrast entrances with settled holds for legibility.
- Hook: The red emergency blackout warning instantly morphing to neon emerald failover lock.
- Outro: MARK-V logo card with "When the sky goes dark, navigation continues."
- Avoid:
  - Generic SaaS language
  - Abstract filler visuals
  - Comic or playful styling

## Visual Identity
- Background: `#0A0E17`
- Secondary Card: `#111927`
- Border Glow: `rgba(0, 230, 118, 0.3)`
- Accent Green: `#00E676`
- Accent Cyan: `#00E5FF`
- Alert Red: `#FF1744`
- Text Primary: `#FFFFFF`
- Text Muted: `#90A4AE`
- Display Font: system-ui, -apple-system, 'Inter', 'Segoe UI', sans-serif
- Monospace Font: 'JetBrains Mono', 'Consolas', monospace

## Storyboard
Use the storyboard in `brag-output/brag-plan.md` as the contract:
1. Scene 1 — The Outage Alert (0.0s – 3.5s)
2. Scene 2 — Deep-Tech Architecture (3.5s – 8.5s)
3. Scene 3 — The Live Cockpit HUD (8.5s – 14.5s)
4. Scene 4 — The ISRO Benchmark Mandate (14.5s – 18.0s)
5. Scene 5 — Mission Outro & GitHub (18.0s – 20.0s)

## Audio
- Audio role: Cinematic driving electronic bed with high-tech synth energy.
- Music: `assets/music/happy-beats-business-moves-vol-12-by-ende-dot-app.mp3`
- Music treatment: Starts at 0.0s, volume 0.85, subtle dip under final title card at 18.0s.
- Music cue guidance: 109.96 BPM; strong cues at 8.74s (HUD cockpit reveal), 13.11s (telemetry stats), 17.47s (benchmark payoff).
- Visual sync:
  - Phone mock entry at 8.74s (// beat-locked: 8.74s)
  - Benchmark 0.11% reveal at 15.29s (// beat-grid)
  - 9x badge hit at 17.47s (// beat-locked: 17.47s)

## Hyperframes Instructions
Build a self-contained, seek-safe HTML5/CSS3/JS composition inside `brag-output/composition/index.html`.
- Canvas dimensions: 1920x1080 (16:9).
- Total duration: 20 seconds (`data-duration="20"`).
- Background: `#0A0E17`.
- Load Google Fonts `Inter` and `JetBrains Mono`.
- Wire the music track `<audio id="bgm" src="assets/music/happy-beats-business-moves-vol-12-by-ende-dot-app.mp3" data-track-index="0" data-volume="0.85"></audio>`.
- Use deterministic time-based animations (driven by requestAnimationFrame or CSS animations mapped to the timeline or Hyperframes tick).
- Ensure high contrast (WCAG compliant) for all text elements.
- Validate with `npx hyperframes check` before rendering.
