"""
Filename: hermes/aidwear/ai_fatigue/utils/models/pf_lstm.py
Author: Diwas Lamsal <diwaslamsal123@hotmail.com>
Date: 2026-06-25
Version: 1.0
Description: Frozen PF (Projection-Fusion) + 2-layer LSTM base for fatigue estimation.
"""

import torch
import torch.nn as nn


class PFFusion(nn.Module):
    """Per-modality projection then concat.

    mod_slices: {modality_name: (col_start, col_end)} into the feature dim F.
    Concatenation order: ecg then imu
    """

    def __init__(
        self,
        mod_slices: dict[str, tuple[int, int]],
        proj_dim: int = 8,
        dropout: float = 0.3,
    ):
        super().__init__()
        self.slices = dict(mod_slices)
        self.order = list(mod_slices.keys())
        self.proj = nn.ModuleDict()
        for name, (a, b) in mod_slices.items():
            self.proj[name] = nn.Sequential(
                nn.Linear(b - a, proj_dim), nn.ReLU(), nn.Dropout(dropout)
            )
        self.out_dim = proj_dim * len(self.order)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        outs = []
        for name in self.order:
            a, b = self.slices[name]
            outs.append(self.proj[name](x[..., a:b]))
        return torch.cat(outs, dim=-1)


class LSTMBackbone(nn.Module):
    """Stacked LSTM with inter-cell dropout; final LayerNorm optional."""

    def __init__(
        self,
        in_dim: int,
        arch=(64, 32),
        inter_cell_dropout: float = 0.3,
        head_layernorm: bool = False,
        head_dropout: float = 0.0,
    ):
        super().__init__()
        self.layers = nn.ModuleList()
        prev = in_dim
        for h in arch:
            self.layers.append(nn.LSTM(prev, h, batch_first=True))
            prev = h
        self.inter_drop = nn.Dropout(inter_cell_dropout)
        self.norm = nn.LayerNorm(prev) if head_layernorm else nn.Identity()
        self.head_drop = nn.Dropout(head_dropout)
        self.out_dim = prev

    def step(self, x_t: torch.Tensor, state: list | None = None):
        """Single-timestep forward that carries the per-layer (h, c) LSTM state."""
        if state is None:
            state = [None] * len(self.layers)
        new_state = []
        h = x_t
        for i, lstm in enumerate(self.layers):
            h, st = lstm(h, state[i])
            new_state.append(st)
            if i < len(self.layers) - 1:
                h = self.inter_drop(h)
        return self.head_drop(self.norm(h)), new_state


class PFSubstrate(nn.Module):
    """fusion -> backbone. encode_step(x_t, state) -> (hidden (B, 1, out_dim), state)."""

    def __init__(
        self,
        mod_slices,
        proj_dim=8,
        fusion_dropout=0.3,
        arch=(64, 32),
        inter_cell_dropout=0.3,
        head_layernorm=False,
        head_dropout=0.0,
    ):
        super().__init__()
        self.fusion = PFFusion(mod_slices, proj_dim, fusion_dropout)
        self.backbone = LSTMBackbone(
            self.fusion.out_dim, arch, inter_cell_dropout, head_layernorm, head_dropout
        )
        self.out_dim = self.backbone.out_dim

    def encode_step(self, x_t: torch.Tensor, state: list | None = None):
        h, state = self.backbone.step(self.fusion(x_t), state)
        return h, state


class SupervisedHead(nn.Module):
    """Per-timestep RPE head: Linear -> ReLU -> Linear -> 1."""

    def __init__(self, in_dim: int, hidden: int = 32):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(in_dim, hidden), nn.ReLU(), nn.Linear(hidden, 1)
        )

    def forward(self, h: torch.Tensor) -> torch.Tensor:
        return self.net(h).squeeze(-1)


class PFBase(nn.Module):
    """Frozen PF base = PFSubstrate + SupervisedHead."""

    def __init__(
        self,
        mod_slices: dict[str, tuple[int, int]],
        proj_dim: int = 8,
        arch=(64, 32),
        fusion_dropout: float = 0.3,
        inter_cell_dropout: float = 0.3,
        head_layernorm: bool = False,
        head_dropout: float = 0.0,
        head_hidden: int = 32,
        **_,
    ):
        super().__init__()
        self.substrate = PFSubstrate(
            mod_slices,
            proj_dim=proj_dim,
            fusion_dropout=fusion_dropout,
            arch=tuple(arch),
            inter_cell_dropout=inter_cell_dropout,
            head_layernorm=head_layernorm,
            head_dropout=head_dropout,
        )
        self.head = SupervisedHead(self.substrate.out_dim, head_hidden)
