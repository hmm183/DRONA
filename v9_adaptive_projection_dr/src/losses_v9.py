"""
v9_adaptive_projection_dr/src/losses_v9.py
-------------------------------------------
Stage 15: Multi-Task Loss Formulation (Fix #4).

Includes:
1. Multi-task Huber loss on error-state vector [\delta E, \delta N, \delta v, \delta psi]
2. BCE confidence loss calibrated against empirical error reduction
3. Post-projection improvement loss: L_post (penalizes worsening state)
4. 10-second closed-loop trajectory loss: L_10s
"""

from __future__ import annotations

import torch
import torch.nn as nn
import torch.nn.functional as F
from typing import Dict, Any, Tuple


class ProjectionMultiTaskLoss(nn.Module):
    """
    Computes loss for training AdaptiveStateProjectionNet.
    """

    def __init__(
        self,
        lambda_pos: float = 1.0,
        lambda_vel: float = 2.0,
        lambda_yaw: float = 5.0,
        lambda_conf: float = 0.5,
        lambda_post: float = 1.0,
        lambda_10s: float = 0.5,
        huber_delta: float = 1.0,
    ):
        super().__init__()
        self.lambda_pos = lambda_pos
        self.lambda_vel = lambda_vel
        self.lambda_yaw = lambda_yaw
        self.lambda_conf = lambda_conf
        self.lambda_post = lambda_post
        self.lambda_10s = lambda_10s
        self.huber_delta = huber_delta

    def forward(
        self,
        pred_delta_x: torch.Tensor,       # (B, 6)
        pred_conf: torch.Tensor,          # (B, 1)
        target_delta_x: torch.Tensor,     # (B, 6)
        prior_error: torch.Tensor,        # (B,) position error prior to projection
        post_error: torch.Tensor,         # (B,) position error after projection
        rollout_error_10s: torch.Tensor,  # (B,) mean position error over next 10s
    ) -> Dict[str, torch.Tensor]:
        # 1. Huber loss on position components (E, N)
        loss_pos_e = F.huber_loss(pred_delta_x[:, 0], target_delta_x[:, 0], delta=self.huber_delta)
        loss_pos_n = F.huber_loss(pred_delta_x[:, 1], target_delta_x[:, 1], delta=self.huber_delta)
        loss_pos = loss_pos_e + loss_pos_n

        # 2. Huber loss on velocity
        loss_vel = F.huber_loss(pred_delta_x[:, 2], target_delta_x[:, 2], delta=0.5)

        # 3. Huber loss on yaw
        loss_yaw = F.huber_loss(pred_delta_x[:, 3], target_delta_x[:, 3], delta=0.1)

        # Base projection error state loss
        l_proj = self.lambda_pos * loss_pos + self.lambda_vel * loss_vel + self.lambda_yaw * loss_yaw

        # 4. Confidence loss (target = 1.0 if post_error < 0.7 * prior_error, else 0.0)
        target_conf = (post_error < (0.75 * prior_error)).float().unsqueeze(-1)
        l_conf = F.binary_cross_entropy(pred_conf, target_conf)

        # 5. Post-projection improvement loss (Fix #4): penalize any increase in error
        # rel_increase = post_error - prior_error
        l_post = torch.mean(F.relu(post_error - prior_error + 0.1))

        # 6. 10-second closed-loop trajectory loss (Fix #4)
        l_10s = torch.mean(rollout_error_10s)

        total_loss = l_proj + self.lambda_conf * l_conf + self.lambda_post * l_post + self.lambda_10s * l_10s

        return {
            "loss_total": total_loss,
            "loss_proj": l_proj,
            "loss_conf": l_conf,
            "loss_post": l_post,
            "loss_10s": l_10s,
        }
