"""
v9_adaptive_projection_dr/src/models_projection_v9.py
------------------------------------------------------
Stage 12: Lightweight Neural Error-State Correction Network (~20k params).

Predicts 6-dimensional error-state correction:
  \hat{\delta x} = [\hat{\delta E}, \hat{\delta N}, \hat{\delta v}, \hat{\delta \psi}, \hat{\delta b_a}, \hat{\delta b_g}]^T
and scalar confidence score:
  c \in [0, 1]

Architecture:
  - Depthwise-separable 1D convolutions over 5.0 s (50-sample) kinematic history
  - Lightweight single-layer GRU for temporal drift context
  - Static context fusion MLP for physical discrepancy and event class
  - Dual heads: Error-state correction head & Sigmoid confidence head
"""

from __future__ import annotations

import torch
import torch.nn as nn
from typing import Tuple, Dict, Any


class DepthwiseSeparableConv1d(nn.Module):
    def __init__(self, in_channels: int, out_channels: int, kernel_size: int = 3, padding: int = 1):
        super().__init__()
        self.depthwise = nn.Conv1d(in_channels, in_channels, kernel_size=kernel_size,
                                   padding=padding, groups=in_channels, bias=False)
        self.pointwise = nn.Conv1d(in_channels, out_channels, kernel_size=1, bias=True)
        self.bn = nn.BatchNorm1d(out_channels)
        self.act = nn.GELU()

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.act(self.bn(self.pointwise(self.depthwise(x))))


class AdaptiveStateProjectionNet(nn.Module):
    """
    Lightweight (~15k - 20k params) Neural Error-State Correction Network.
    """

    def __init__(self, in_features: int = 8, context_dim: int = 12,
                 hidden_dim: int = 32, num_events: int = 8):
        super().__init__()
        self.in_features = in_features
        self.hidden_dim = hidden_dim

        # Temporal sequence feature extractor
        self.conv1 = DepthwiseSeparableConv1d(in_features, hidden_dim, kernel_size=3, padding=1)
        self.conv2 = DepthwiseSeparableConv1d(hidden_dim, hidden_dim, kernel_size=3, padding=1)
        self.gru = nn.GRU(hidden_dim, hidden_dim, num_layers=1, batch_first=True)

        # Static checkpoint context encoder: [d_p, d_v, d_yaw, speed_curr] (4) + event onehot (8) = 12 dims
        self.context_encoder = nn.Sequential(
            nn.Linear(context_dim, hidden_dim),
            nn.GELU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.GELU(),
        )

        # Fusion layer
        self.fusion = nn.Sequential(
            nn.Linear(hidden_dim * 2, hidden_dim * 2),
            nn.GELU(),
            nn.Dropout(0.1),
        )

        # Head 1: 6-dim error state correction [\delta E, \delta N, \delta v, \delta psi, \delta b_a, \delta b_g]
        self.head_error_state = nn.Sequential(
            nn.Linear(hidden_dim * 2, hidden_dim),
            nn.GELU(),
            nn.Linear(hidden_dim, 6),
        )

        # Head 2: Confidence score c in [0, 1]
        self.head_confidence = nn.Sequential(
            nn.Linear(hidden_dim * 2, 16),
            nn.GELU(),
            nn.Linear(16, 1),
            nn.Sigmoid(),
        )

    def forward(self, x_seq: torch.Tensor, x_ctx: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor]:
        """
        Args:
            x_seq: (B, T=50, in_features=8) temporal kinematic sequence
            x_ctx: (B, context_dim=12) checkpoint context vector
        Returns:
            delta_x: (B, 6) predicted error-state correction
            conf: (B, 1) confidence in [0, 1]
        """
        # x_seq: (B, T, C) -> (B, C, T) for Conv1d
        x_trans = x_seq.transpose(1, 2)
        h_conv = self.conv2(self.conv1(x_trans))  # (B, hidden_dim, T)
        h_conv = h_conv.transpose(1, 2)  # (B, T, hidden_dim)

        out_gru, h_n = self.gru(h_conv)
        gru_last = h_n.squeeze(0)  # (B, hidden_dim)

        # Context vector
        ctx_feat = self.context_encoder(x_ctx)  # (B, hidden_dim)

        # Combined representation
        combined = torch.cat([gru_last, ctx_feat], dim=-1)  # (B, hidden_dim * 2)
        feat = self.fusion(combined)

        delta_x = self.head_error_state(feat)
        conf = self.head_confidence(feat)

        return delta_x, conf

    def count_parameters(self) -> int:
        return sum(p.numel() for p in self.parameters() if p.requires_grad)
