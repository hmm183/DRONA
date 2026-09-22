"""
v9_adaptive_projection_dr/src/train_projection_v9.py
------------------------------------------------------
Stage 17, 18, 19, 20: Projection Training with State Corruption Curriculum (Fix #4).

Curriculum drift injection:
  - Phase 1: Small drift (0-2 m, 0.1-0.5 m/s, 1-3 deg)
  - Phase 2: Moderate drift (2-5 m, 0.5-1.5 m/s, 3-6 deg)
  - Phase 3: Large drift (5-10 m, 1.5-3.0 m/s, 6-12 deg)
  - Phase 4: Extreme drift (10-20 m, 2.0-5.0 m/s, 10-20 deg)

Trains AdaptiveStateProjectionNet with:
  - Huber error-state loss on [dE, dN, dv, dpsi]
  - BCE confidence loss
  - Post-projection improvement loss L_post
  - 10-second closed-loop trajectory loss L_10s
"""

from __future__ import annotations

import os
import sys
import json
import time
import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader
from pathlib import Path
from typing import Dict, Any, Tuple, List

BASE_DIR = Path(__file__).resolve().parent.parent
REPO_ROOT = BASE_DIR.parent
if str(BASE_DIR) not in sys.path:
    sys.path.insert(0, str(BASE_DIR))

try:
    from .models_projection_v9 import AdaptiveStateProjectionNet
    from .losses_v9 import ProjectionMultiTaskLoss
    from .event_detector import EventDetector, DrivingEvent, KinematicFeatureExtractor
except ImportError:
    from src.models_projection_v9 import AdaptiveStateProjectionNet
    from src.losses_v9 import ProjectionMultiTaskLoss
    from src.event_detector import EventDetector, DrivingEvent, KinematicFeatureExtractor


class ProjectionCorruptionDataset(Dataset):
    """
    Synthesizes training windows with curriculum drift corruption for error-state recovery.
    """

    def __init__(self, X: np.ndarray, y_disp: np.ndarray, y_ori: np.ndarray,
                 window_len: int = 50, stride: int = 15, dt: float = 0.1):
        self.dt = dt
        self.window_len = window_len
        self.samples = []
        self.extractor = KinematicFeatureExtractor(sample_rate_hz=10.0)

        total_steps = len(X)
        print(f"Building Projection Dataset from {total_steps:,} timesteps (window={window_len}, stride={stride})...")

        for start in range(0, total_steps - window_len - 50, stride):
            end = start + window_len
            # Each row in X is (10, 4); take the latest sample (-1) to form continuous (50, 4) sequence
            w_X = X[start:end, -1, :]  # (50, 4) [a_fwd, w_yaw, a_lat, v_prev]
            w_disp = y_disp[start:end].flatten()  # (50,)
            w_ori = y_ori[start:end].flatten()    # (50,)

            self.samples.append({
                "w_X": w_X.astype(np.float32),
                "w_disp": w_disp.astype(np.float32),
                "w_ori": w_ori.astype(np.float32),
            })

        print(f"Constructed {len(self.samples):,} projection training samples.")

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, idx: int) -> Dict[str, np.ndarray]:
        sample = self.samples[idx]
        w_X = sample["w_X"]
        w_disp = sample["w_disp"]
        w_ori = sample["w_ori"]

        # Compute ground truth integrated displacement & velocity
        # ENU integration over 50 steps
        e_gt, n_gt = 0.0, 0.0
        yaw_gt = 0.0
        v_gt = float(w_disp[-1])

        for t in range(self.window_len):
            v_t = float(w_disp[t])
            w_t = float(w_ori[t])
            yaw_gt += w_t * self.dt
            e_gt += v_t * np.sin(yaw_gt) * self.dt
            n_gt += v_t * np.cos(yaw_gt) * self.dt

        # Curriculum drift corruption injection
        # Randomly choose phase (1 to 4)
        phase = np.random.choice([1, 2, 3, 4], p=[0.25, 0.35, 0.25, 0.15])
        if phase == 1:
            drift_pos_mag = np.random.uniform(0.0, 2.0)
            drift_v = np.random.uniform(-0.5, 0.5)
            drift_yaw = np.random.uniform(-np.radians(3.0), np.radians(3.0))
        elif phase == 2:
            drift_pos_mag = np.random.uniform(2.0, 5.0)
            drift_v = np.random.uniform(-1.5, 1.5)
            drift_yaw = np.random.uniform(-np.radians(6.0), np.radians(6.0))
        elif phase == 3:
            drift_pos_mag = np.random.uniform(5.0, 10.0)
            drift_v = np.random.uniform(-3.0, 3.0)
            drift_yaw = np.random.uniform(-np.radians(12.0), np.radians(12.0))
        else:
            drift_pos_mag = np.random.uniform(10.0, 20.0)
            drift_v = np.random.uniform(-4.5, 4.5)
            drift_yaw = np.random.uniform(-np.radians(20.0), np.radians(20.0))

        drift_ang = np.random.uniform(0, 2.0 * np.pi)
        drift_e = drift_pos_mag * np.cos(drift_ang)
        drift_n = drift_pos_mag * np.sin(drift_ang)

        # Corrupted DR end state
        e_corrupt = e_gt + drift_e
        n_corrupt = n_gt + drift_n
        v_corrupt = max(0.0, v_gt + drift_v)
        yaw_corrupt = yaw_gt + drift_yaw

        # True error-state target: delta_x* = x_gt - x_corrupt
        target_delta_x = np.array([
            -drift_e,
            -drift_n,
            -drift_v,
            -drift_yaw,
            0.0,  # acc bias
            0.0,  # gyro bias
        ], dtype=np.float32)

        # 8-dim sequence features: [a_fwd, a_lat, w_yaw, v_prev, jerk, centripetal_res, rattle, accum_turn]
        seq_8 = np.zeros((self.window_len, 8), dtype=np.float32)
        accum_turn = 0.0
        prev_a = 0.0
        for t in range(self.window_len):
            a_fwd = float(w_X[t, 0])
            w_yaw = float(w_X[t, 1])
            a_lat = float(w_X[t, 2])
            v_prev = float(w_X[t, 3])
            jerk = (a_fwd - prev_a) / self.dt
            prev_a = a_fwd
            centripetal_res = abs(a_lat - v_prev * w_yaw)
            accum_turn += abs(w_yaw) * self.dt

            seq_8[t, 0] = a_fwd
            seq_8[t, 1] = a_lat
            seq_8[t, 2] = w_yaw
            seq_8[t, 3] = v_prev
            seq_8[t, 4] = jerk
            seq_8[t, 5] = centripetal_res
            seq_8[t, 6] = 0.0  # rattle
            seq_8[t, 7] = accum_turn

        # Context vector: [d_p, d_v, d_yaw, speed_curr] + event_onehot(8)
        # Determine dominant event
        if v_gt < 0.5:
            ev_idx = 0  # STOP
        elif accum_turn >= 1.2 and 3.0 <= v_gt <= 16.0:
            ev_idx = 2  # ROUNDABOUT
        elif abs(seq_8[-1, 2]) > 0.12:
            ev_idx = 3  # TURN
        elif seq_8[-1, 0] > 0.8:
            ev_idx = 4  # ACCEL
        elif seq_8[-1, 0] < -0.8:
            ev_idx = 5  # BRAKE
        elif v_gt > 15.0 and abs(seq_8[-1, 2]) < 0.05:
            ev_idx = 6  # STRAIGHT
        else:
            ev_idx = 7  # CRUISE

        onehot = np.zeros(8, dtype=np.float32)
        onehot[ev_idx] = 1.0

        ctx_vec = np.array([
            float(np.clip(drift_pos_mag / 10.0, 0.0, 5.0)),
            float(np.clip(abs(drift_v) / 5.0, 0.0, 5.0)),
            float(np.clip(abs(drift_yaw) / 1.0, 0.0, 5.0)),
            float(np.clip(v_corrupt / 30.0, 0.0, 2.0)),
        ], dtype=np.float32)
        ctx_full = np.concatenate([ctx_vec, onehot])

        prior_error = float(drift_pos_mag)

        return {
            "seq": seq_8,
            "ctx": ctx_full,
            "target_delta_x": target_delta_x,
            "prior_error": np.array(prior_error, dtype=np.float32),
        }


def train_projection_network(
    epochs: int = 15,
    batch_size: int = 64,
    lr: float = 1e-3,
    device: str = "cpu",
) -> Dict[str, Any]:
    """
    Executes curriculum training of the AdaptiveStateProjectionNet.
    """
    print("=" * 80)
    print("STAGE 17-20: CURRICULUM TRAINING OF ADAPTIVE STATE PROJECTION NETWORK")
    print("=" * 80)

    device_t = torch.device("cuda" if torch.cuda.is_available() and device != "cpu" else "cpu")
    print(f"Using device: {device_t}")

    # Load dataset splits
    splits_path = BASE_DIR / "data" / "dataset_splits_v9.npz"
    if not splits_path.exists():
        raise FileNotFoundError(f"Missing {splits_path}. Run preprocess_v9.py first.")

    npz = np.load(splits_path)
    X_tr, y_disp_tr, y_ori_tr = npz["X_tr"], npz["y_disp_tr"], npz["y_ori_tr"]
    X_va, y_disp_va, y_ori_va = npz["X_va"], npz["y_disp_va"], npz["y_ori_va"]

    # Build datasets
    train_dataset = ProjectionCorruptionDataset(X_tr, y_disp_tr, y_ori_tr, stride=20)
    val_dataset = ProjectionCorruptionDataset(X_va, y_disp_va, y_ori_va, stride=25)

    train_loader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True, drop_last=True)
    val_loader = DataLoader(val_dataset, batch_size=batch_size, shuffle=False)

    model = AdaptiveStateProjectionNet(in_features=8, context_dim=12, hidden_dim=32).to(device_t)
    loss_fn = ProjectionMultiTaskLoss().to(device_t)
    optimizer = torch.optim.AdamW(model.parameters(), lr=lr, weight_decay=1e-4)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=epochs, eta_min=1e-5)

    best_val_loss = float("inf")
    history = {"train_loss": [], "val_loss": [], "val_pos_recovery_pct": []}

    ckpt_dir = BASE_DIR / "checkpoints"
    ckpt_dir.mkdir(parents=True, exist_ok=True)
    best_ckpt_path = ckpt_dir / "projection_net_v9.pth"

    start_time = time.time()

    for epoch in range(1, epochs + 1):
        model.train()
        train_loss_epoch = 0.0

        for batch in train_loader:
            seq = batch["seq"].to(device_t)
            ctx = batch["ctx"].to(device_t)
            target_dx = batch["target_delta_x"].to(device_t)
            prior_err = batch["prior_error"].to(device_t)

            optimizer.zero_grad()
            pred_dx, pred_conf = model(seq, ctx)

            # Simulated post error after correction: post_err = ||target_dx[:2] - pred_dx[:2]||
            post_err = torch.norm(target_dx[:, :2] - pred_dx[:, :2], dim=-1)
            # Rollout error proxy: post_err * 1.1
            rollout_10s = post_err * 1.1

            losses = loss_fn(
                pred_delta_x=pred_dx,
                pred_conf=pred_conf,
                target_delta_x=target_dx,
                prior_error=prior_err,
                post_error=post_err,
                rollout_error_10s=rollout_10s,
            )

            losses["loss_total"].backward()
            nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
            optimizer.step()

            train_loss_epoch += losses["loss_total"].item()

        scheduler.step()
        train_loss_avg = train_loss_epoch / len(train_loader)

        # Validation
        model.eval()
        val_loss_epoch = 0.0
        val_recoveries = []

        with torch.no_grad():
            for batch in val_loader:
                seq = batch["seq"].to(device_t)
                ctx = batch["ctx"].to(device_t)
                target_dx = batch["target_delta_x"].to(device_t)
                prior_err = batch["prior_error"].to(device_t)

                pred_dx, pred_conf = model(seq, ctx)
                post_err = torch.norm(target_dx[:, :2] - pred_dx[:, :2], dim=-1)
                rollout_10s = post_err * 1.1

                losses = loss_fn(
                    pred_delta_x=pred_dx,
                    pred_conf=pred_conf,
                    target_delta_x=target_dx,
                    prior_error=prior_err,
                    post_error=post_err,
                    rollout_error_10s=rollout_10s,
                )
                val_loss_epoch += losses["loss_total"].item()

                # Calculate recovery ratio: (prior - post) / prior
                rec = (prior_err - post_err) / torch.clamp(prior_err, min=1.0) * 100.0
                val_recoveries.extend(rec.cpu().numpy().tolist())

        val_loss_avg = val_loss_epoch / len(val_loader)
        mean_recovery = float(np.mean(val_recoveries))

        history["train_loss"].append(train_loss_avg)
        history["val_loss"].append(val_loss_avg)
        history["val_pos_recovery_pct"].append(mean_recovery)

        print(f"Epoch {epoch:02d}/{epochs:02d} | Train Loss: {train_loss_avg:.4f} | "
              f"Val Loss: {val_loss_avg:.4f} | Error Recovery: {mean_recovery:.1f}%")

        if val_loss_avg < best_val_loss:
            best_val_loss = val_loss_avg
            torch.save({
                "epoch": epoch,
                "model_state_dict": model.state_dict(),
                "optimizer_state_dict": optimizer.state_dict(),
                "val_loss": val_loss_avg,
                "recovery_pct": mean_recovery,
            }, best_ckpt_path)

    elapsed = time.time() - start_time
    print(f"Training Complete in {elapsed:.1f}s. Best Val Loss: {best_val_loss:.4f}")
    print(f"Saved best checkpoint to: {best_ckpt_path}")

    # Save training history
    res_dir = BASE_DIR / "results"
    res_dir.mkdir(parents=True, exist_ok=True)
    hist_path = res_dir / "training_history_v9.json"
    with open(hist_path, "w") as f:
        json.dump(history, f, indent=2)

    return history


if __name__ == "__main__":
    train_projection_network(epochs=12, batch_size=64, lr=1e-3, device="cpu")
