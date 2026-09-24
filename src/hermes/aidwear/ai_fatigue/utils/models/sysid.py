"""
Filename: hermes/aidwear/ai_fatigue/utils/models/sysid.py
Author: Diwas Lamsal <diwaslamsal123@hotmail.com>
Date: 2026-06-25
Version: 1.0
Description: Learned System Identification personalization adapter on the frozen PF base.
             Running best algorithm so far.
"""

import torch
import torch.nn as nn


def _mlp(din: int, dh: int, dout: int) -> nn.Sequential:
    return nn.Sequential(nn.Linear(din, dh), nn.ReLU(), nn.Linear(dh, dout))


class SysIdUpdater(nn.Module):
    """Report-conditioned recurrent dynamics adapter (two timescales: fast z, persistent theta)."""

    def __init__(
        self,
        h_dim: int,
        w_dim: int = 8,
        hidden: int = 64,
        z_dim: int = 32,
        theta_dim: int = 8,
        base_relative: bool = False,
        theta_to: str = "both",
    ):
        super().__init__()
        assert theta_to in ("both", "film", "head", "none")
        self.enc = _mlp(h_dim, hidden, hidden)
        e = hidden
        self.z_dim = z_dim
        self.theta_dim = theta_dim
        self.base_relative = base_relative
        self.theta_to = theta_to
        self.W_iz = nn.Linear(e, z_dim)
        self.W_hz = nn.Linear(z_dim, z_dim)  # update gate
        self.W_ir = nn.Linear(e, z_dim)
        self.W_hr = nn.Linear(z_dim, z_dim)  # reset gate
        self.W_in = nn.Linear(e, z_dim)
        self.W_hn = nn.Linear(z_dim, z_dim)  # candidate
        self.film = _mlp(theta_dim, hidden, 2 * z_dim)
        nn.init.zeros_(self.film[-1].weight)
        nn.init.zeros_(self.film[-1].bias)
        self.theta_gru = nn.GRUCell(5, theta_dim)
        self.head = _mlp(z_dim + theta_dim, hidden, 1)
        if base_relative:
            nn.init.zeros_(self.head[-1].weight)
            nn.init.zeros_(self.head[-1].bias)

    def _mgru(self, x, z, dz, dn):
        r = torch.sigmoid(self.W_ir(x) + self.W_hr(z))
        zg = torch.sigmoid(self.W_iz(x) + self.W_hz(z) + dz)
        n = torch.tanh(self.W_in(x) + r * self.W_hn(z) + dn)
        return (1.0 - zg) * n + zg * z


def rebuild(cfg: dict, state: dict, device) -> SysIdUpdater:
    """Reconstruct a SysIdUpdater from a checkpoint cfg + state_dict (na_dense_eval.rebuild)."""
    m = SysIdUpdater(
        cfg["h_dim"],
        hidden=cfg["hidden"],
        z_dim=cfg["z_dim"],
        theta_dim=cfg["theta_dim"],
        base_relative=cfg["base_relative"],
        theta_to=cfg["theta_to"],
    ).to(device)
    m.load_state_dict(state)
    m.eval()
    return m


class SysIdStreamer:
    """Stateful per-window driver around a (frozen-weights) SysIdUpdater."""

    def __init__(self, model: SysIdUpdater, device: torch.device):
        self.m = model
        self.device = device
        z_dim, th_dim = model.z_dim, model.theta_dim
        self.z = torch.zeros(1, z_dim, device=device)
        self.theta = torch.zeros(1, th_dim, device=device)
        self._zero_theta = torch.zeros(1, th_dim, device=device)
        self.prev_innov = torch.zeros(1, device=device)
        self.base_anchor = None
        self.last_rt = -1.0
        self.rcount = 0.0
        self.t = -1
        self._last_pred = None
        self._last_base = None

    @torch.no_grad()
    def step(self, h_t: torch.Tensor, base_t: torch.Tensor) -> float:
        """Advance z one window and return the SysID prediction. h_t (1,H), base_t (1,)."""
        self.t += 1
        m = self.m
        e_t = m.enc(h_t)
        th_film = self.theta if m.theta_to in ("film", "both") else self._zero_theta
        th_head = self.theta if m.theta_to in ("head", "both") else self._zero_theta
        dz, dn = m.film(th_film).chunk(2, -1)
        self.z = m._mgru(e_t, self.z, dz, dn)
        out = m.head(torch.cat([self.z, th_head], dim=-1)).squeeze(-1)  # (1,)
        pred = base_t + out if m.base_relative else out
        if self.base_anchor is None:
            self.base_anchor = base_t.detach().clone()
        self._last_pred = pred
        self._last_base = base_t
        return float(pred.item())

    @torch.no_grad()
    def ingest_report(self, r: float) -> None:
        """Persistent theta update from a true RPE report (scored against the last pred)."""
        if self._last_pred is None:
            return
        m = self.m
        r_t = torch.tensor([float(r)], device=self.device)
        preerr = r_t - self._last_pred
        elapsed = max(1.0, self.t - self.last_rt)
        innov_trend = (preerr - self.prev_innov) / elapsed
        base_trend = (self._last_base - self.base_anchor) / elapsed
        rcount_feat = torch.tensor([self.rcount / 5.0], device=self.device)
        ev = torch.stack(
            [preerr, self.prev_innov, innov_trend, base_trend, rcount_feat], dim=-1
        )
        self.theta = m.theta_gru(ev, self.theta)
        self.prev_innov = preerr
        self.base_anchor = self._last_base.detach().clone()
        self.last_rt = float(self.t)
        self.rcount += 1.0
