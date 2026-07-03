"""
core/tire/pinn_model.py — Pacejka MF tire model (import-only library).

This module is the tire authority for the moto_sim product.  It must be
importable with ZERO dependency on tire_rig, ws_server, or any UI layer.

════════════════════════════════════════════════════════════════════════════════
LATERAL FORCE  (Pacejka MF, pure lateral + camber thrust)
────────────────────────────────────────────────────────
    D      = μ · Fz
    B      = k_alpha·Fz / (C_pac·D + ε)
    φ      = B·α' − E·(B·α' − atan(B·α'))
    Fy_pac = D · sin(C_pac · atan(φ))
    Fy_0   = clamp(Fy_pac + k_gamma·Fz·γ, −D, D)      [pure-slip lateral]

LONGITUDINAL FORCE  (simplified MF, pure slip)
────────────────────────────────────────────────
    Fx_0 = μ·Fz·tanh(Kx·κ)                             [pure-slip longitudinal]

COMBINED SLIP  (Pacejka 2012, §4.3.2 — C∞ smooth, no branching)
────────────────────────────────────────────────────────────────
    Gxα = cos(Cxα · atan(Bxα · α))                     [Fx reduction from α]
    Gyκ = cos(Cyκ · atan(Byκ · κ))                     [Fy reduction from κ]

    Fx  = Gxα · Fx_0                                   [combined longitudinal]
    Fy  = Gyκ · Fy_0                                   [combined lateral]

    With Cxα = Cyκ = 1.0: atan range = (−π/2, π/2),
    cos is always ≥ 0 → both G-factors are non-negative at all inputs.

ALIGNING TORQUE  (Pacejka MF pneumatic trail × COMBINED lateral force)
────────────────────────────────────────────────────────────────────────
    tp   = tp0 · cos(Ct · atan(Bt · α'))               [pneumatic trail]
    Mz   = −Fy · tp  +  K_gamma_Mz · γ · Fz

    Using COMBINED Fy (not pure-slip Fy_pac) corrects the B3 Mz-drift bug:
    Mz and Fy are now consistent under any combined-slip condition.

    Limiting behaviour:
        α'→ 0 :  tp = tp0  (max trail),  |Mz| rises with slip
        α' ≈ α_peak:  tp → 0  — steering "goes light"
        α' > α_reversal = tan(π/(2·Ct))/Bt:  tp < 0  (Mz reversal)

SLIP-BASED RELAXATION  (handled by TireModelWithRelaxation / tire_interface)
────────────────────────────────────────────────────────────────────────────
    ZOH (A-stable):  α'[k+1] = α + (α'[k]−α)·exp(−Vx·dt/σ)
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import NamedTuple

import torch
import torch.nn as nn
import yaml


# ─────────────────────────────────────────────────────────────────────────────
class TireCoeffs(NamedTuple):
    """All Pacejka / combined-slip / geometry coefficients for one compound."""
    # Friction and cornering
    mu:         float = 0.90
    k_alpha:    float = 12.0
    k_gamma:    float = 0.90
    # Pacejka MF shape
    C_pac:      float = 1.80
    E_pac:      float = 0.00
    # Longitudinal slip stiffness
    Kx:         float = 12.0
    # Pneumatic trail
    tp0:        float = 0.030
    Bt:         float = 10.0
    Ct:         float = 1.50
    K_gamma_Mz: float = 0.020
    # Combined-slip weighting (Pacejka 2012 §4.3.2)
    Bxa:        float = 10.0   # Gxα stiffness  [1/rad]
    Cxa:        float = 1.00   # Gxα shape      [–]  keep ≤ 1 for non-negative G
    Byk:        float = 8.00   # Gyκ stiffness  [–]
    Cyk:        float = 1.00   # Gyκ shape      [–]  keep ≤ 1 for non-negative G
    # Relaxation
    sigma:      float = 0.30   # relaxation length [m]   motorcycle nominal
    # ── M5: Thermal model ────────────────────────────────────────────
    T_optimal:    float = 85.0   # optimal tread temperature [°C]
    T_width:      float = 25.0   # Gaussian half-width [°C]
    T_cold_grip:  float = 0.65   # grip fraction at cold temperature [–]
    tau_tread:    float = 30.0   # tread thermal time constant [s]
    tau_carcass:  float = 120.0  # carcass coupling time constant [s]
    heat_scale:   float = 4.0e-3 # tread temp rise per Watt [(°C·s)/J]
    T_ambient:    float = 20.0   # ambient temperature [°C]
    # ── M5: Wear model ───────────────────────────────────────────────
    k_wear:        float = 1.8e-8 # wear rate coefficient [1/(J^wear_exp)]
    wear_exp:      float = 1.50   # wear power-law exponent [–]
    grip_wear_max: float = 0.30   # max grip loss at full wear (W=1) [–]
    # ── M5: Pressure model ────────────────────────────────────────────
    p_cold:       float = 230.0  # cold inflation pressure [kPa]
    p_optimal:    float = 240.0  # optimal operating pressure [kPa]

    @classmethod
    def from_yaml(cls, path: str | Path) -> "TireCoeffs":
        with open(path) as f:
            d = yaml.safe_load(f)
        # Only pass keys that exist in the NamedTuple
        valid = {k: d[k] for k in cls._fields if k in d}
        return cls(**valid)

    def to_yaml(self, path: str | Path) -> None:
        with open(path, "w") as f:
            yaml.dump(self._asdict(), f, default_flow_style=False)


# ─────────────────────────────────────────────────────────────────────────────
class DummyTirePINN(nn.Module):
    """
    Pacejka MF tire model wrapped in a PyTorch nn.Module.

    Architecture
    ────────────
    Placeholder PINN: forward() runs analytical Pacejka.
    Production path: load trained weights, replace forward() with self.net(x).

    Parameters are plain Python floats (not nn.Parameter) — the model is
    deterministic and stateless; no training required for the analytical path.

    Parameters
    ──────────
    See TireCoeffs for field descriptions.
    """

    def __init__(self, coeffs: TireCoeffs | None = None, **kwargs) -> None:
        super().__init__()

        if coeffs is None:
            coeffs = TireCoeffs(**{k: v for k, v in kwargs.items()
                                   if k in TireCoeffs._fields})
        self.coeffs = coeffs

        # Expose individual fields as attributes for backward compatibility
        for field, val in coeffs._asdict().items():
            setattr(self, field, val)

        # Placeholder network (unused in analytical mode)
        self.net = nn.Sequential(
            nn.Linear(3, 32), nn.Tanh(),
            nn.Linear(32, 32), nn.Tanh(),
            nn.Linear(32, 2),
        )

    # ── Pure-slip forward (vectorised, for tire characterisation) ─────────────
    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """
        Pure-slip lateral force + aligning torque.

        Parameters
        ----------
        x : Tensor[..., 3]  — [Fz [N],  α' [rad],  γ [rad]]

        Returns
        -------
        Tensor[..., 2]  — [Fy [N],  Mz [N·m]]

        Notes
        -----
        Uses PURE-SLIP Fy_pac in Mz (correct for pure-slip characterisation).
        For combined-slip (with κ), call predict_combined() which uses the
        combined Fy in Mz — the B3-fix path.
        """
        Fz    = x[..., 0]
        alpha = x[..., 1]
        gamma = x[..., 2]

        # Pacejka MF
        D   = self.mu * Fz
        B   = (self.k_alpha * Fz) / (self.C_pac * D + 1e-8)
        Ba  = B * alpha
        phi = Ba - self.E_pac * (Ba - torch.atan(Ba))
        Fy_pac = D * torch.sin(self.C_pac * torch.atan(phi))

        # Camber thrust + clamp
        Fy = torch.clamp(Fy_pac + self.k_gamma * Fz * gamma, -D, D)

        # Pneumatic trail (symmetric in α: cos is even)
        tp = self.tp0 * torch.cos(self.Ct * torch.atan(self.Bt * alpha))

        # Mz — pure-slip version uses Fy_pac (pre-clamp, pre-combined)
        Mz = -Fy_pac * tp + self.K_gamma_Mz * gamma * Fz

        return torch.stack([Fy, Mz], dim=-1)

    # ── Fast scalar hot-path (pure math, zero torch allocation) ──────────────
    def _predict_scalar(
        self, Fz: float, alpha: float, gamma: float
    ) -> tuple[float, float, float]:
        """
        Pure-slip Pacejka evaluation using only math module — no torch,
        no tensor allocation, no GC pressure.

        Returns (Fy_pac, Fy, tp) where:
          Fy_pac = raw Pacejka lateral (pre-clamp, used for Mz)
          Fy     = clamped Fy_pac + camber thrust
          tp     = pneumatic trail
        Called by predict_combined() in the 1 kHz hot path.
        """
        D   = self.mu * Fz
        B   = (self.k_alpha * Fz) / (self.C_pac * D + 1e-8)
        Ba  = B * alpha
        phi = Ba - self.E_pac * (Ba - math.atan(Ba))
        Fy_pac = D * math.sin(self.C_pac * math.atan(phi))
        Fy_cam = self.k_gamma * Fz * gamma
        Fy     = max(-D, min(D, Fy_pac + Fy_cam))
        tp     = self.tp0 * math.cos(self.Ct * math.atan(self.Bt * alpha))
        return Fy_pac, Fy, tp

    # ── Scalar pure-slip convenience (torch path — for batch/autograd use) ────
    @torch.no_grad()
    def predict(
        self, Fz: float, slip_angle: float, camber: float = 0.0,
    ) -> tuple[float, float]:
        """
        Pure-slip (Fy, Mz).  Uses torch path — suitable for autograd,
        batch sweeps, and the vectorised forward().
        For the 1 kHz hot path, predict_combined() calls _predict_scalar().
        """
        x   = torch.tensor([Fz, slip_angle, camber], dtype=torch.float32)
        out = self.forward(x)
        return float(out[0].item()), float(out[1].item())

    # ── Combined-slip scalar (B3 + B4 fixed) ─────────────────────────────────
    # No @torch.no_grad() — this method is now pure math (no tensors).
    def predict_combined(
        self,
        Fz:         float,
        slip_angle: float,   # relaxed α' [rad]
        kappa:      float,   # longitudinal slip ratio [-]
        camber:     float = 0.0,
    ) -> tuple[float, float, float]:
        """
        Combined-slip forces with Pacejka-native Gxα / Gyκ weighting.

        Physics (B4 fix)
        ────────────────
        Pure-slip forces:
            Fx0 = μ·Fz·tanh(Kx·κ)
            Fy0 = Pacejka MF pure-slip (with camber thrust + clamp)

        Combined-slip weighting  (Pacejka 2012 §4.3.2, C∞ smooth):
            Gxα = cos(Cxα · atan(Bxα · α'))    ← reduces Fx due to lateral slip
            Gyκ = cos(Cyκ · atan(Byκ · κ))     ← reduces Fy due to long. slip

            With Cxα = Cyκ = 1.0: cos argument ∈ (−π/2, π/2) → G ≥ 0 always.
            No if/else branches → C∞ continuous across the entire (α, κ) space.

        Combined forces:
            Fx = Gxα · Fx0
            Fy = Gyκ · Fy0

        Mz (B3 fix)
        ───────────
        Mz = −Fy · tp + K_gamma_Mz · γ · Fz

        Using the combined Fy (not pure-slip Fy_pac) ensures Mz and Fy are
        always physically consistent, eliminating drift under combined slip.

        Returns
        -------
        (Fx, Fy, Mz)
        """
        # ── Pure-slip forces (zero-allocation scalar path) ───────────────────
        F_peak         = self.mu * Fz
        Fx0            = F_peak * math.tanh(self.Kx * kappa)
        _, Fy0, tp     = self._predict_scalar(Fz, slip_angle, camber)

        # ── Pacejka combined-slip weighting (C∞, no branching) ──────────────
        # Gxα: longitudinal force reduction due to lateral slip
        Gxa = math.cos(self.Cxa * math.atan(self.Bxa * slip_angle))
        # Gyκ: lateral force reduction due to longitudinal slip
        Gyk = math.cos(self.Cyk * math.atan(self.Byk * kappa))

        Fx = Gxa * Fx0
        Fy = Gyk * Fy0

        # ── Aligning torque (B3 fix: uses combined Fy; tp from _predict_scalar)
        Mz = -Fy * tp + self.K_gamma_Mz * camber * Fz

        return Fx, Fy, Mz

    # ── Analytical helpers ────────────────────────────────────────────────────
    def peak_slip_angle_deg(self, Fz: float = 1000.0) -> float:
        """Fy peak angle [°] via dense scan 0–30°."""
        alphas = torch.linspace(0.0, math.radians(30.0), 3000)
        x = torch.stack([
            torch.full_like(alphas, Fz),
            alphas,
            torch.zeros_like(alphas),
        ], dim=-1)
        with torch.no_grad():
            Fy = self.forward(x)[..., 0]
        return float(math.degrees(alphas[Fy.argmax()].item()))

    def trail_reversal_deg(self) -> float:
        """Slip angle at tp = 0 [°].  Returns inf if Ct ≤ 1 (no reversal)."""
        if self.Ct <= 1.0:
            return float("inf")
        return math.degrees(math.tan(math.pi / (2.0 * self.Ct)) / self.Bt)

    def cornering_stiffness(self, Fz: float = 1470.0) -> float:
        """dFy/dα at α=0 [N/rad] — should equal B·C·D."""
        D = self.mu * Fz
        B = (self.k_alpha * Fz) / (self.C_pac * D + 1e-8)
        return B * self.C_pac * D

    # ── Factory ───────────────────────────────────────────────────────────────
    @classmethod
    def from_yaml(cls, path: str | Path) -> "DummyTirePINN":
        return cls(coeffs=TireCoeffs.from_yaml(path))

    @classmethod
    def from_coeffs(cls, coeffs: TireCoeffs) -> "DummyTirePINN":
        return cls(coeffs=coeffs)


# ─────────────────────────────────────────────────────────────────────────────
class TireModelWithRelaxation:
    """
    ZOH relaxation wrapper around DummyTirePINN.

    Kept for backward compatibility with tire_rig.py.
    Prefer tire_interface.TireModel for new code.

    Physical basis (slip-based formulation):
        σ/Vx · dα'/dt + α' = α     (τ = σ/Vx)
        ZOH: α'[k+1] = α + (α'[k]−α)·exp(−Vx·dt/σ)
    """

    def __init__(self, pinn: DummyTirePINN, sigma: float = 0.3) -> None:
        if sigma <= 0.0:
            raise ValueError("sigma must be positive [m].")
        self.pinn         = pinn
        self.sigma        = sigma
        self._alpha_prime = 0.0

    def reset(self, alpha0: float = 0.0) -> None:
        self._alpha_prime = alpha0

    def step(
        self,
        Fz: float, slip_angle: float, vx: float, dt: float, camber: float = 0.0,
    ) -> tuple[float, float]:
        vx_safe = max(abs(vx), 1e-3)
        decay   = math.exp(-vx_safe * dt / self.sigma)
        self._alpha_prime = slip_angle + (self._alpha_prime - slip_angle) * decay
        return self.pinn.predict(Fz=Fz, slip_angle=self._alpha_prime, camber=camber)

    @property
    def alpha_prime(self) -> float:
        return self._alpha_prime
